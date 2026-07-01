// spirit_driver: onboard ROS2 <-> Gremsy PayloadSDK bridge.
//
// This node is the ONLY thing that talks to the Gremsy over UDP. It exposes the
// full payload SDK to ROS2:
//   * subscribes a command topic for every payload feature (see gremsy_ros_topics.h)
//     and forwards each to the SDK, and
//   * registers the SDK callbacks and publishes all telemetry (gimbal attitude,
//     every payload parameter, camera info, streaming uri, storage, capture).
//
// Ground clients (ui_demo_ros2) therefore need only ROS2 + a running spirit_driver
// on the drone -- they never see the Gremsy network directly.
//
// The Gremsy IP/port and namespace are ROS parameters / launch args, so nothing
// about the deployment is hardcoded in the build. Topic names are relative and
// can be remapped at launch without rebuilding.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "drone_msgs/msg/command_gimbal.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "payloadSdkInterface.h"
#include "gremsy_ros_topics.h"
#if defined MB1
    #include "mb1_sdk.h"
#elif defined VIO
    #include "vio_sdk.h"
#elif defined ORUSL
    #include "orusl_sdk.h"
#endif

using namespace gremsy_topics;

static PayloadSdkInterface* my_payload = nullptr;

class SpiritDriver : public rclcpp::Node
{
public:
  SpiritDriver() : Node("spirit_driver")
  {
    // ---- Connection parameters (never hardcoded in the build) ----
    const std::string gremsy_ip = this->declare_parameter<std::string>("gremsy_ip", "192.168.70.23");
    const int gremsy_port = this->declare_parameter<int>("gremsy_port", 14566);
    // Pre-existing external-interface topics. These predate this driver and are
    // consumed by other systems (the basestation publishes gimbal_command; the
    // bag logs gimbal_orientation / move_gimbal_angle), so they are NOT under the
    // gremsy namespace -- they default to their original absolute names and are
    // overridden per-drone from config/<drone>.yaml.
    gimbal_command_topic_ = this->declare_parameter<std::string>("gimbal_command_topic", "/spiritnx3/gimbal/command");
    const std::string gimbal_orientation_topic =
      this->declare_parameter<std::string>("gimbal_orientation_topic", "/gimbal_orientation");
    move_gimbal_angle_topic_ =
      this->declare_parameter<std::string>("move_gimbal_angle_topic", "/move_gimbal_angle");

    // ---- Telemetry publishers ----
    gimbal_orientation_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(gimbal_orientation_topic, 10);
    cam_param_pub_ = this->create_publisher<std_msgs::msg::String>(TLM_CAM_PARAM, 10);
    stream_uri_pub_ = this->create_publisher<std_msgs::msg::String>(TLM_STREAM_URI, rclcpp::QoS(1).transient_local());
    has_video_pub_ = this->create_publisher<std_msgs::msg::Bool>(TLM_HAS_VIDEO, rclcpp::QoS(1).transient_local());
    storage_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(TLM_STORAGE, 10);
    capture_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(TLM_CAPTURE, 10);

    // One Float64 topic per payload parameter (LRF, zoom, target/payload GPS,
    // IR temps, tracker box, gimbal mode, ...). Derived from the SDK table so it
    // stays in sync automatically. e.g. params/lrf_range, params/target_lon.
    for (const auto & p : payloadParams) {
      std::string id(p.id);
      std::transform(id.begin(), id.end(), id.begin(), [](unsigned char c) { return std::tolower(c); });
      param_pubs_[p.index] = this->create_publisher<std_msgs::msg::Float64>(std::string(TLM_PARAMS_PREFIX) + id, 10);
    }

    // ---- Command subscriptions (one per payload feature) ----
    setup_command_subscriptions();

    // ---- Connect to the Gremsy ----
    RCLCPP_INFO(this->get_logger(), "Connecting to Gremsy payload at %s:%d", gremsy_ip.c_str(), gremsy_port);
    conn_.type = CONTROL_UDP;
    conn_.device.udp.ip = strdup(gremsy_ip.c_str());
    conn_.device.udp.port = gremsy_port;
    my_payload = new PayloadSdkInterface(conn_);
    my_payload->sdkInitConnection();
    my_payload->regPayloadStatusChanged(
      std::bind(&SpiritDriver::onPayloadStatusChanged, this, std::placeholders::_1, std::placeholders::_2));
    my_payload->regPayloadParamChanged(
      std::bind(&SpiritDriver::onPayloadParamChanged, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    my_payload->regPayloadStreamChanged(
      std::bind(&SpiritDriver::onPayloadStreamChanged, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    my_payload->checkPayloadConnection();
    my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_RC_MODE, PAYLOAD_CAMERA_RC_MODE_STANDARD, PARAM_TYPE_UINT32);

    request_param_rates();
    RCLCPP_INFO(this->get_logger(), "spirit_driver ready; full payload API exposed on ROS2.");
  }

  ~SpiritDriver() override
  {
    if (my_payload != nullptr) {
      try { my_payload->sdkQuit(); } catch (...) {}
      delete my_payload;
      my_payload = nullptr;
    }
  }

private:
  // ======================= Telemetry (SDK -> ROS2) =======================
  void onPayloadStatusChanged(int event, double* param)
  {
    switch (event) {
      case PAYLOAD_GB_ATTITUDE: {
        // param[0]=pitch, param[1]=roll, param[2]=yaw. Publish as (roll,pitch,yaw).
        geometry_msgs::msg::Vector3 v;
        v.x = param[1];
        v.y = param[0];
        v.z = param[2];
        gimbal_orientation_pub_->publish(v);
        break;
      }
      case PAYLOAD_PARAMS: {
        const int idx = static_cast<int>(param[0]);
        auto it = param_pubs_.find(idx);
        if (it != param_pubs_.end()) {
          std_msgs::msg::Float64 m;
          m.data = param[1];
          it->second->publish(m);
        }
        break;
      }
      case PAYLOAD_CAM_STORAGE_INFO: {
        std_msgs::msg::Float64MultiArray m;
        m.data = {param[0], param[1], param[2], param[3]};  // total, used, available, status
        storage_pub_->publish(m);
        break;
      }
      case PAYLOAD_CAM_CAPTURE_STATUS: {
        std_msgs::msg::Int32MultiArray m;
        m.data = {static_cast<int>(param[0]), static_cast<int>(param[1]),
                  static_cast<int>(param[2]), static_cast<int>(param[3])};
        capture_pub_->publish(m);
        break;
      }
      case PAYLOAD_CAM_INFO: {
        const bool has_video = (static_cast<int16_t>(param[0]) & CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM) != 0;
        std_msgs::msg::Bool b;
        b.data = has_video;
        has_video_pub_->publish(b);
        if (has_video && my_payload != nullptr) {
          my_payload->getPayloadCameraStreamingInformation();
        }
        break;
      }
      default: break;
    }
  }

  void onPayloadParamChanged(int event, char* param_char, double* param)
  {
    if (event == PAYLOAD_CAM_PARAMS && param_char != nullptr) {
      std_msgs::msg::String m;
      std::ostringstream oss;
      oss << param_char << " " << param[1];
      m.data = oss.str();
      cam_param_pub_->publish(m);
    }
  }

  void onPayloadStreamChanged(int event, char* param_char, double* /*param_double*/)
  {
    if (event == PAYLOAD_CAM_STREAMINFO && param_char != nullptr) {
      std_msgs::msg::String m;
      m.data = param_char;
      stream_uri_pub_->publish(m);
    }
  }

  void request_param_rates()
  {
    if (my_payload == nullptr) return;
    my_payload->setParamRate(PARAM_EO_ZOOM_LEVEL, 1000);
    my_payload->setParamRate(PARAM_IR_ZOOM_LEVEL, 1000);
    my_payload->setParamRate(PARAM_LRF_RANGE, 100);
    my_payload->setParamRate(PARAM_LRF_OFSET_X, 100);
    my_payload->setParamRate(PARAM_LRF_OFSET_Y, 100);
    my_payload->setParamRate(PARAM_TARGET_COOR_LON, 1000);
    my_payload->setParamRate(PARAM_TARGET_COOR_LAT, 1000);
    my_payload->setParamRate(PARAM_TARGET_COOR_ALT, 1000);
    my_payload->setParamRate(PARAM_PAYLOAD_GPS_LON, 1000);
    my_payload->setParamRate(PARAM_PAYLOAD_GPS_LAT, 1000);
    my_payload->setParamRate(PARAM_PAYLOAD_GPS_ALT, 1000);
    my_payload->setParamRate(PARAM_CAM_VIEW_MODE, 1000);
    my_payload->setParamRate(PARAM_CAM_REC_SOURCE, 1000);
    my_payload->setParamRate(PARAM_CAM_IR_TYPE, 1000);
    my_payload->setParamRate(PARAM_CAM_IR_PALETTE_ID, 1000);
    my_payload->setParamRate(PARAM_CAM_IR_FFC_MODE, 1000);
    my_payload->setParamRate(PARAM_GIMBAL_MODE, 1000);
    my_payload->setParamRate(PARAM_IR_TEMP_MAX, 1000);
    my_payload->setParamRate(PARAM_IR_TEMP_MIN, 1000);
    my_payload->setParamRate(PARAM_IR_TEMP_MEAN, 1000);
    my_payload->setParamRate(PARAM_TRACK_POS_X, 100);
    my_payload->setParamRate(PARAM_TRACK_POS_Y, 100);
    my_payload->setParamRate(PARAM_TRACK_POS_W, 100);
    my_payload->setParamRate(PARAM_TRACK_POS_H, 100);
    my_payload->setParamRate(PARAM_TRACK_STATUS, 100);
  }

  // ======================= Commands (ROS2 -> SDK) =======================
  // Small helpers to keep the many subscriptions terse.
  template <typename MsgT, typename F>
  typename rclcpp::Subscription<MsgT>::SharedPtr sub(const char* topic, F fn)
  {
    return this->create_subscription<MsgT>(topic, 10, [fn](const typename MsgT::SharedPtr m) {
      if (my_payload != nullptr) fn(*m);
    });
  }

  void setCamParam(const char* id, int value, uint8_t type = PARAM_TYPE_UINT32)
  {
    my_payload->setPayloadCameraParam(const_cast<char*>(id), value, type);
  }

  void setup_command_subscriptions()
  {
    using std_msgs::msg::Bool;
    using std_msgs::msg::Empty;
    using std_msgs::msg::Float64;
    using std_msgs::msg::Int32;
    using std_msgs::msg::String;
    using geometry_msgs::msg::Vector3;

    // Camera source / capture / record
    subs_.push_back(sub<Int32>(CMD_VIEW_SRC, [this](const Int32& m){ setCamParam(PAYLOAD_CAMERA_VIEW_SRC, m.data); }));
    subs_.push_back(sub<Bool>(CMD_TOGGLE_EO_IR, [this](const Bool& m){
      setCamParam(PAYLOAD_CAMERA_VIEW_SRC, m.data ? PAYLOAD_CAMERA_VIEW_IR : PAYLOAD_CAMERA_VIEW_EO); }));
    subs_.push_back(sub<Bool>(CMD_DUAL_STREAM, [this](const Bool& m){
      setCamParam(PAYLOAD_CAMERA_VIEW_SRC, m.data ? PAYLOAD_CAMERA_VIEW_EOIR : PAYLOAD_CAMERA_VIEW_EO); }));
    subs_.push_back(sub<Int32>(CMD_RECORD_SRC, [this](const Int32& m){ setCamParam(PAYLOAD_CAMERA_RECORD_SRC, m.data); }));
    subs_.push_back(sub<Empty>(CMD_CAPTURE, [](const Empty&){ my_payload->setPayloadCameraCaptureImage(); }));
    subs_.push_back(sub<Bool>(CMD_RECORD, [](const Bool& m){
      if (m.data) my_payload->setPayloadCameraRecordVideoStart();
      else        my_payload->setPayloadCameraRecordVideoStop(); }));

    // Zoom / focus
    subs_.push_back(sub<Int32>(CMD_EO_ZOOM_SPEED, [this](const Int32& m){ setCamParam(PAYLOAD_CAMERA_EO_ZOOM_SPEED, m.data); }));
    subs_.push_back(sub<Float64>(CMD_ZOOM_CONTINUOUS, [](const Float64& m){ my_payload->setCameraZoom(ZOOM_TYPE_CONTINUOUS, m.data); }));
    subs_.push_back(sub<Float64>(CMD_ZOOM_STEP, [](const Float64& m){ my_payload->setCameraZoom(ZOOM_TYPE_STEP, m.data); }));
    subs_.push_back(sub<Float64>(CMD_ZOOM_RANGE, [](const Float64& m){ my_payload->setCameraZoom(ZOOM_TYPE_RANGE, m.data); }));
    subs_.push_back(sub<Int32>(CMD_EO_FOCUS_SPEED, [this](const Int32& m){ setCamParam(PAYLOAD_CAMERA_EO_FOCUS_SPEED, m.data); }));
    subs_.push_back(sub<Empty>(CMD_FOCUS_AUTO, [](const Empty&){ my_payload->setCameraFocus(FOCUS_TYPE_AUTO); }));
    subs_.push_back(sub<Float64>(CMD_FOCUS_CONTINUOUS, [](const Float64& m){ my_payload->setCameraFocus(FOCUS_TYPE_CONTINUOUS, m.data); }));

    // Exposure / image
    subs_.push_back(sub<Int32>(CMD_AE_MODE, [this](const Int32& m){ setCamParam(PAYLOAD_CAMERA_VIDEO_AUTO_EXPOSURE, m.data); }));
    subs_.push_back(sub<Int32>(CMD_SHUTTER, [this](const Int32& m){ setCamParam(PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED, m.data); }));
    subs_.push_back(sub<Int32>(CMD_IRIS, [this](const Int32& m){ setCamParam(PAYLOAD_CAMERA_VIDEO_APERTURE_VALUE, m.data); }));
    subs_.push_back(sub<Int32>(CMD_GAIN, [this](const Int32& m){ setCamParam(PAYLOAD_CAMERA_EO_GAIN_LS, m.data); }));
    subs_.push_back(sub<Int32>(CMD_WHITE_BALANCE, [this](const Int32& m){ setCamParam(PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE, m.data); }));
    subs_.push_back(sub<Empty>(CMD_WHITE_BALANCE_TRIGGER, [](const Empty&){ my_payload->setPayloadCameraWBOnePushTrigg(); }));
    subs_.push_back(sub<Int32>(CMD_IMAGE_FLIP, [this](const Int32& m){ setCamParam(PAYLOAD_CAMERA_VIDEO_FLIP, m.data); }));
    subs_.push_back(sub<Int32>(CMD_OSD_MODE, [this](const Int32& m){ setCamParam(PAYLOAD_CAMERA_VIDEO_OSD_MODE, m.data); }));

    // IR
    subs_.push_back(sub<Int32>(CMD_IR_PALETTE, [this](const Int32& m){ setCamParam(PAYLOAD_CAMERA_IR_PALETTE, m.data); }));
    subs_.push_back(sub<Int32>(CMD_IR_FFC_MODE, [](const Int32& m){ my_payload->setPayloadCameraFFCMode(m.data); }));
    subs_.push_back(sub<Empty>(CMD_IR_FFC_TRIGGER, [](const Empty&){ my_payload->setPayloadCameraFFCTrigg(); }));

    // LRF
    subs_.push_back(sub<Int32>(CMD_LRF_MODE, [this](const Int32& m){ setCamParam(PAYLOAD_LRF_MODE, m.data); }));

    // Tracking
    subs_.push_back(sub<Int32>(CMD_TRACK_MODE, [this](const Int32& m){ setCamParam(PAYLOAD_CAMERA_TRACKING_MODE, m.data); }));
    subs_.push_back(sub<Vector3>(CMD_TRACK_TOUCH, [](const Vector3& m){
      my_payload->setPayloadObjectTrackingPosition(static_cast<int>(m.x), static_cast<int>(m.y), 128, 128); }));
    subs_.push_back(sub<Int32>(CMD_TRACK, [](const Int32& m){ my_payload->setPayloadObjectTrackingMode(m.data); }));

    // Gimbal
    subs_.push_back(sub<Float64>(CMD_GIMBAL_TILT, [](const Float64& m){ my_payload->setGimbalSpeed(m.data, 0, 0, INPUT_SPEED); }));
    subs_.push_back(sub<Float64>(CMD_GIMBAL_PAN, [](const Float64& m){ my_payload->setGimbalSpeed(0, 0, m.data, INPUT_SPEED); }));
    subs_.push_back(sub<Vector3>(CMD_GIMBAL_ANGLE, [](const Vector3& m){
      my_payload->setGimbalSpeed(m.x, m.y, m.z, INPUT_ANGLE); }));  // (pitch, roll, yaw)
    subs_.push_back(sub<Int32>(CMD_GIMBAL_MODE, [this](const Int32& m){ setCamParam(PAYLOAD_CAMERA_GIMBAL_MODE, m.data); }));

    // Misc
    subs_.push_back(sub<Empty>(CMD_QUERY_PARAMS, [](const Empty&){
      my_payload->getPayloadCameraInformation();
      my_payload->getPayloadCameraSettingList(); }));

    // Generic full-API passthrough: "PARAM_ID VALUE [int32|uint32]"
    set_camera_param_sub_ = this->create_subscription<String>(
      CMD_SET_CAMERA_PARAM, 10,
      std::bind(&SpiritDriver::onSetCameraParam, this, std::placeholders::_1));

    // Structured gimbal orientation command (drone_msgs/CommandGimbal) -- unchanged
    // external interface from the basestation (absolute topic, not namespaced).
    gimbal_command_sub_ = this->create_subscription<drone_msgs::msg::CommandGimbal>(
      gimbal_command_topic_, 10,
      std::bind(&SpiritDriver::onGimbalCommand, this, std::placeholders::_1));

    // Redundant-but-consistent alias under the gremsy namespace
    // (/<ns>/gremsy/gimbal_command), so the same command is reachable both ways.
    // Skipped only if the configured topic already resolves to this relative name.
    if (gimbal_command_topic_ != "gimbal_command") {
      gimbal_command_ns_sub_ = this->create_subscription<drone_msgs::msg::CommandGimbal>(
        "gimbal_command", 10,
        std::bind(&SpiritDriver::onGimbalCommand, this, std::placeholders::_1));
    }

    // Pre-existing raw angle command (geometry_msgs/Vector3 = pitch,roll,yaw deg),
    // restored at its original absolute name for backward compatibility.
    move_gimbal_angle_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      move_gimbal_angle_topic_, 10,
      [](const geometry_msgs::msg::Vector3::SharedPtr m) {
        if (my_payload != nullptr) my_payload->setGimbalSpeed(m->x, m->y, m->z, INPUT_ANGLE);
      });
  }

  void onSetCameraParam(const std_msgs::msg::String& msg)
  {
    if (my_payload == nullptr) return;
    std::istringstream iss(msg.data);
    std::string id, type_str;
    double value = 0.0;
    if (!(iss >> id >> value)) {
      RCLCPP_WARN(this->get_logger(), "set_camera_param: bad message '%s' (expected 'ID VALUE [int32|uint32]')", msg.data.c_str());
      return;
    }
    iss >> type_str;
    const uint8_t type = (type_str == "int32") ? PARAM_TYPE_INT32 : PARAM_TYPE_UINT32;
    my_payload->setPayloadCameraParam(const_cast<char*>(id.c_str()), static_cast<int>(value), type);
    RCLCPP_INFO(this->get_logger(), "set_camera_param %s = %g (%s)", id.c_str(), value,
                type == PARAM_TYPE_INT32 ? "int32" : "uint32");
  }

  void onGimbalCommand(const drone_msgs::msg::CommandGimbal& msg)
  {
    const auto & cmd = msg.gimbal_command;
    if (cmd.command_type != drone_msgs::msg::GimbalCommand::ORIENTATION || my_payload == nullptr) {
      return;
    }
    tf2::Quaternion q;
    tf2::fromMsg(cmd.orientation, q);
    q.normalize();
    double roll_rad = 0.0, pitch_rad = 0.0, yaw_rad = 0.0;
    tf2::Matrix3x3(q).getRPY(roll_rad, pitch_rad, yaw_rad);
    const float roll_deg = static_cast<float>(roll_rad * 180.0 / M_PI);
    const float pitch_deg = static_cast<float>(pitch_rad * 180.0 / M_PI);
    const float yaw_deg = static_cast<float>(yaw_rad * 180.0 / M_PI);
    my_payload->setGimbalSpeed(pitch_deg, roll_deg, yaw_deg, INPUT_ANGLE);
  }

  // ---- members ----
  T_ConnInfo conn_{};
  std::string gimbal_command_topic_;
  std::string move_gimbal_angle_topic_;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gimbal_orientation_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cam_param_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stream_uri_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr has_video_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr storage_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr capture_pub_;
  std::map<int, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> param_pubs_;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr set_camera_param_sub_;
  rclcpp::Subscription<drone_msgs::msg::CommandGimbal>::SharedPtr gimbal_command_sub_;
  rclcpp::Subscription<drone_msgs::msg::CommandGimbal>::SharedPtr gimbal_command_ns_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr move_gimbal_angle_sub_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SpiritDriver>());
  rclcpp::shutdown();
  return 0;
}
