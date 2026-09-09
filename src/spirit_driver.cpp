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
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
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
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "drone_msgs/msg/command_gimbal.hpp"
#include "lion_ros2_bridge/msg/position.hpp"
#include "lion_ros2_bridge/msg/gimbal_state.hpp"
#include "lion_ros2_bridge/msg/lrf_tracking_data.hpp"
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
    robot_name_ = this->declare_parameter<std::string>("drone", "spiritnx3");
    // Drone FC GPS (mavros NavSatFix). The payload GPS params are the FC
    // solution forwarded at ~1 Hz and ~0.7 s stale; Position publishes from
    // this topic only. Override per-drone in config/<drone>.yaml.
    const std::string fc_gps_topic = this->declare_parameter<std::string>(
      "fc_gps_topic", "/robot_3/interface/mavros/global_position/global");
    fc_gps_max_age_sec_ = this->declare_parameter<double>("fc_gps_max_age_sec", 1.0);
    attitude_max_age_sec_ = this->declare_parameter<double>("attitude_max_age_sec", 1.0);
    // LRF validity gate: exclude on-ground readings and no-return sentinels.
    lrf_valid_min_m_ = this->declare_parameter<double>("lrf_valid_min_m", 5.0);
    lrf_valid_max_m_ = this->declare_parameter<double>("lrf_valid_max_m", 300.0);
    // Gimbal control mode asserted before every ORIENTATION command. An absolute
    // angle only HOLDS if the payload is in a mode that accepts it: left in OFF
    // (the power-on default) the gimbal tracks the command briefly and then
    // decays back to its resting attitude, which reads as "it always ends up
    // pointing down". Measured on nx3, 2026-08-18.
    //
    // MUST agree with spirit_task_executor's `gimbal_yaw_frame`, which decides
    // whether the yaw we send is body-relative or earth-referenced:
    //     follow <-> gimbal_yaw_frame: body       (yaw relative to the airframe)
    //     lock   <-> gimbal_yaw_frame: absolute   (yaw is a compass heading)
    // Flipping one without the other points the camera at the wrong bearing.
    const std::string mode_name = this->declare_parameter<std::string>("gimbal_mode", "follow");
    if (mode_name == "lock") {
      gimbal_mode_ = PAYLOAD_CAMERA_GIMBAL_MODE_LOCK;
    } else if (mode_name == "follow") {
      gimbal_mode_ = PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW;
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "gimbal_mode must be 'follow' or 'lock', got '%s'", mode_name.c_str());
      throw std::runtime_error("invalid gimbal_mode");
    }
    // How often the mode is re-asserted while commands stream in. The pointer
    // publishes at 5 Hz; writing GB_MODE that often would flood the payload
    // link for no gain, so assert on a slow cadence and let it recover if the
    // payload is power-cycled or another operator changes the mode underneath us.
    gimbal_mode_reassert_sec_ = this->declare_parameter<double>("gimbal_mode_reassert_sec", 2.0);
    // Zoom is re-asserted on the same principle -- see applyZoom().
    zoom_reassert_sec_ = this->declare_parameter<double>("zoom_reassert_sec", 3.0);

    // ---- Telemetry publishers ----
    gimbal_orientation_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(gimbal_orientation_topic, 10);
    // Fused payload GPS + gimbal attitude, stamped on the drone clock, for reid
    // ingestion (relative name -> /<ns>/gremsy/position; reader keys on ".../position").
    position_pub_ = this->create_publisher<lion_ros2_bridge::msg::Position>("position", 10);
    // Gimbal state carries the EO zoom_level UFM needs to select camera intrinsics
    // (relative name -> /<ns>/gremsy/gimbal_state). Modeled on the lion gimbal/state.
    gimbal_state_pub_ = this->create_publisher<lion_ros2_bridge::msg::GimbalState>("gimbal_state", 10);
    // LRF range -> lrf_samples; UFM derives ground_z from it (relative -> /<ns>/gremsy/lrf).
    lrf_pub_ = this->create_publisher<lion_ros2_bridge::msg::LrfTrackingData>("lrf", 10);
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

    // FC GPS subscription (best-effort sensor QoS, matches mavros publishers).
    fc_gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      fc_gps_topic, rclcpp::SensorDataQoS(),
      std::bind(&SpiritDriver::onFcGps, this, std::placeholders::_1));

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
    // Absolute zoom targets (setCameraZoomTarget) address the COMBINE range;
    // set the mode once here so GimbalCommand.zoom levels always mean the
    // same thing, whatever the payload was left in.
    my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_VIDEO_ZOOM_MODE,
                                      PAYLOAD_CAMERA_VIDEO_ZOOM_MODE_COMBINE, PARAM_TYPE_UINT32);
    // ...and immediately undo what that write did to the SDK's gimbal state.
    // ZOOM_MODE_COMBINE is 0, which is also GIMBAL_MODE_OFF, so the line above
    // leaves the SDK convinced the gimbal is OFF and every attitude command
    // from here on would carry GIMBAL_DEVICE_FLAGS_RETRACT. See setCamParam().
    gimbal_mode_dirty_ = true;
    ensureGimbalMode();

    request_param_rates();

    // Publish the fused Position + GimbalState at 10 Hz (matches the Lion cadence).
    position_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { publish_position(); publish_gimbal_state(); publish_lrf(); });

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
        // Cache for the fused /position message.
        roll_deg_.store(param[1]);
        pitch_deg_.store(param[0]);
        yaw_deg_.store(param[2]);
        ori_rx_ns_.store(this->now().nanoseconds());
        have_orientation_.store(true);
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
        // Cache payload GPS for the fused /position message.
        if (idx == PARAM_PAYLOAD_GPS_LAT) { pay_lat_.store(param[1]); have_lat_.store(true); }
        else if (idx == PARAM_PAYLOAD_GPS_LON) { pay_lon_.store(param[1]); have_lon_.store(true); }
        else if (idx == PARAM_PAYLOAD_GPS_ALT) { pay_alt_.store(param[1]); have_alt_.store(true); }
        else if (idx == PARAM_EO_ZOOM_LEVEL) { eo_zoom_.store(param[1]); have_zoom_.store(true); }
        else if (idx == PARAM_LRF_RANGE) { lrf_range_.store(param[1]); have_lrf_.store(true); }
        // The mode was already polled at 1 Hz (request_param_rates) but thrown
        // away, so GimbalState.mode reported UNSPECIFIED and the one thing that
        // explains a decaying gimbal was invisible from ROS. Keep it.
        else if (idx == PARAM_GIMBAL_MODE) { gb_mode_.store(static_cast<int>(param[1])); have_mode_.store(true); }
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

  void onFcGps(const sensor_msgs::msg::NavSatFix& msg)
  {
    if (msg.status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
        !is_real_fix(msg.latitude, msg.longitude)) {
      fc_fix_.store(false);
      return;
    }
    fc_lat_.store(msg.latitude);
    fc_lon_.store(msg.longitude);
    fc_alt_.store(msg.altitude);
    fc_rx_ns_.store(this->now().nanoseconds());
    fc_fix_.store(true);
  }

  // (0, 0) is the SDK's pre-fix placeholder, never a real position.
  static bool is_real_fix(double lat, double lon)
  {
    return std::fabs(lat) > 1e-6 || std::fabs(lon) > 1e-6;
  }

  // Publish Position from the FC GPS only (single receiver, single datum —
  // no payload fallback, so the trajectory can never mix sources). Altitude
  // passes through as reported by fc_gps_topic (ellipsoidal for mavros
  // global_position/global; switch to the FC AMSL topic once the interface
  // exposes it). Both the FC fix and the gimbal attitude must be present and
  // fresh — nothing is published otherwise (no 0,0,0 orientation defaults, no
  // stale attitude held past attitude_max_age_sec).
  void publish_position()
  {
    const int64_t now_ns = this->now().nanoseconds();
    const bool fc_fresh = fc_fix_.load() &&
      (now_ns - fc_rx_ns_.load()) < static_cast<int64_t>(fc_gps_max_age_sec_ * 1e9);
    const bool ori_fresh = ori_rx_ns_.load() != 0 &&
      (now_ns - ori_rx_ns_.load()) < static_cast<int64_t>(attitude_max_age_sec_ * 1e9);
    if (!fc_fresh || !ori_fresh) {
      return;
    }
    lion_ros2_bridge::msg::Position msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "gremsy";
    msg.vehicle_id = robot_name_;
    msg.lla.latitude_deg = fc_lat_.load();
    msg.lla.longitude_deg = fc_lon_.load();
    msg.lla.altitude_m = fc_alt_.load();
    msg.orientation.roll_deg = static_cast<float>(roll_deg_.load());
    msg.orientation.pitch_deg = static_cast<float>(pitch_deg_.load());
    msg.orientation.yaw_deg = static_cast<float>(yaw_deg_.load());
    position_pub_->publish(msg);
  }

  // Publish gimbal telemetry (drone-clock stamped) so the reader writes
  // gimbal_samples; UFM reads zoom_level from there to select camera intrinsics.
  // Only the zoom_level is consumed by UFM today; angles are filled for context.
  void publish_gimbal_state()
  {
    if (!have_zoom_.load()) {
      return;  // wait for a real EO zoom so gimbal_samples never carries a bogus default
    }
    lion_ros2_bridge::msg::GimbalState msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "gremsy";
    msg.vehicle_id = robot_name_;
    msg.gimbal_id = "gremsy";
    if (have_orientation_.load()) {
      msg.pan_deg = static_cast<float>(yaw_deg_.load());
      msg.tilt_deg = static_cast<float>(pitch_deg_.load());
      msg.roll_deg = static_cast<float>(roll_deg_.load());
    }
    msg.zoom_level = static_cast<float>(eo_zoom_.load());
    msg.mode = have_mode_.load() ? gimbalStateMode(gb_mode_.load())
                                 : lion_ros2_bridge::msg::GimbalState::MODE_UNSPECIFIED;
    gimbal_state_pub_->publish(msg);
  }

  // Publish LRF range (drone-clock stamped) -> reader writes lrf_samples; UFM
  // derives ground_z from cruise-window altitude + LRF range.
  void publish_lrf()
  {
    if (!have_lrf_.load()) {
      return;  // wait for a real LRF reading
    }
    lion_ros2_bridge::msg::LrfTrackingData msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "gremsy";
    msg.vehicle_id = robot_name_;
    const double range = lrf_range_.load();
    msg.lrf_range_meters = static_cast<float>(range);
    msg.lrf_data_valid = (range > lrf_valid_min_m_ && range < lrf_valid_max_m_);
    if (have_zoom_.load()) msg.zoom_level = static_cast<float>(eo_zoom_.load());
    lrf_pub_->publish(msg);
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

  // EVERY camera-parameter write corrupts the gimbal attitude flags. The SDK
  // assigns `current_gimbal_mode = param_value` for ANY parameter
  // (payloadSdkInterface.cpp:184), and setGimbalSpeed then derives its
  // GIMBAL_DEVICE flags from that same variable (ibid. 1135-1150) -- so writing
  // an unrelated camera parameter decides how the next attitude command is
  // interpreted. The value collision is total: ZOOM_MODE_COMBINE,
  // ZOOM_COMBINE_1X and GIMBAL_MODE_OFF are all 0, and OFF maps to
  // GIMBAL_DEVICE_FLAGS_RETRACT -- the stow command. That is why the gimbal
  // tracked a commanded angle briefly and then returned to pointing down.
  //
  // Mark the mode dirty on every write so the next attitude command re-asserts
  // it. Rate-limiting alone is not enough: between two re-asserts, any camera
  // write silently re-arms RETRACT.
  void setCamParam(const char* id, int value, uint8_t type = PARAM_TYPE_UINT32)
  {
    my_payload->setPayloadCameraParam(const_cast<char*>(id), value, type);
    gimbal_mode_dirty_ = true;
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

    // Gimbal. All assert the mode first: any camera write clobbers the SDK's
    // current_gimbal_mode, and OFF(0) maps to RETRACT. See the README.
    subs_.push_back(sub<Float64>(CMD_GIMBAL_TILT, [this](const Float64& m){
      ensureGimbalMode();
      my_payload->setGimbalSpeed(m.data, 0, 0, INPUT_SPEED); }));
    subs_.push_back(sub<Float64>(CMD_GIMBAL_PAN, [this](const Float64& m){
      ensureGimbalMode();
      my_payload->setGimbalSpeed(0, 0, m.data, INPUT_SPEED); }));
    // Two axes at once; tilt/pan above each send a full 3-axis setpoint and
    // cancel. No command timeout in the payload: the publisher owns stopping.
    subs_.push_back(sub<Vector3>(CMD_GIMBAL_RATE, [this](const Vector3& m){
      ensureGimbalMode();
      my_payload->setGimbalSpeed(m.x, m.y, m.z, INPUT_SPEED); }));  // (pitch, roll, yaw) deg/s
    subs_.push_back(sub<Vector3>(CMD_GIMBAL_ANGLE, [this](const Vector3& m){
      ensureGimbalMode();
      my_payload->setGimbalSpeed(m.x, m.y, m.z, INPUT_ANGLE); }));  // (pitch, roll, yaw)
    // An operator setting the mode also becomes the mode we ASSERT from here
    // on. Without this the next attitude command would re-assert the configured
    // mode 200 ms later and silently undo them.
    subs_.push_back(sub<Int32>(CMD_GIMBAL_MODE, [this](const Int32& m){
      gimbal_mode_ = m.data;
      setCamParam(PAYLOAD_CAMERA_GIMBAL_MODE, m.data);
      gimbal_mode_dirty_ = false;
      last_mode_sent_ = this->get_clock()->now();
      RCLCPP_INFO(this->get_logger(), "gimbal mode set to %d by operator", m.data);
    }));

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
      [this](const geometry_msgs::msg::Vector3::SharedPtr m) {
        if (my_payload == nullptr) return;
        // Same reason as onGimbalCommand: an absolute angle only holds if the
        // payload is in a mode that accepts one.
        ensureGimbalMode();
        my_payload->setGimbalSpeed(m->x, m->y, m->z, INPUT_ANGLE);
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
    gimbal_mode_dirty_ = true;  // any camera write clobbers it; see setCamParam()
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
    // ORDER IS LORE-BEARING. applyZoom writes a camera parameter, which
    // clobbers the SDK's current_gimbal_mode and therefore the flags
    // setGimbalSpeed will send. Zoom first, then assert the mode, then command
    // the angle -- so the mode is correct at the instant it is read. Zooming
    // last (as this did) left RETRACT armed for the following command.
    applyZoom(cmd.zoom);
    ensureGimbalMode();
    my_payload->setGimbalSpeed(pitch_deg, roll_deg, yaw_deg, INPUT_ANGLE);
  }

  // Gremsy's GB_MODE vocabulary and lion_ros2_bridge/GimbalState's are NOT the
  // same numbers -- Gremsy is OFF=0, LOCK=1, FOLLOW=2 while the message is
  // UNSPECIFIED=0, FOLLOW=1, LOCK=2. Passing one through as the other reports
  // exactly the opposite mode, so translate explicitly.
  static uint8_t gimbalStateMode(int gremsy_mode)
  {
    using GS = lion_ros2_bridge::msg::GimbalState;
    switch (gremsy_mode) {
      case PAYLOAD_CAMERA_GIMBAL_MODE_LOCK:   return GS::MODE_LOCK;
      case PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW: return GS::MODE_FOLLOW;
      // OFF/MAPPING/RESET have no GimbalState equivalent. UNSPECIFIED is honest
      // here: the consumer learns "not a mode you can reason about", which is
      // the truth, rather than a plausible wrong one.
      default: return GS::MODE_UNSPECIFIED;
    }
  }

  // Assert the control mode the commanded angles assume. Without this the
  // driver inherited whatever the payload booted in -- and an absolute angle
  // sent into OFF produces a brief move followed by a decay back to the
  // resting attitude, with nothing in the logs to say why.
  // MUST be called immediately before setGimbalSpeed, with no camera-parameter
  // write in between -- see setCamParam() for why. Sends when the SDK's notion
  // of the mode has been dirtied by another write, or when the re-assert
  // interval has elapsed (so the mode also recovers from a payload power-cycle
  // or another operator changing it underneath us).
  void ensureGimbalMode()
  {
    if (my_payload == nullptr) {
      return;
    }
    const auto now = this->get_clock()->now();
    const bool stale = last_mode_sent_.nanoseconds() == 0 ||
      (now - last_mode_sent_).seconds() >= gimbal_mode_reassert_sec_;
    if (!gimbal_mode_dirty_ && !stale) {
      return;
    }
    setCamParam(PAYLOAD_CAMERA_GIMBAL_MODE, gimbal_mode_);
    gimbal_mode_dirty_ = false;   // set by the setCamParam above; this IS the mode
    last_mode_sent_ = now;
    // Log only on a real transition, so a healthy stack stays quiet but a
    // gimbal being fought over by another operator is visible.
    if (have_mode_.load() && gb_mode_.load() != gimbal_mode_) {
      RCLCPP_WARN(this->get_logger(),
                  "gimbal mode was %d, asserting %d (GB_MODE: OFF=0 LOCK=1 FOLLOW=2)",
                  gb_mode_.load(), gimbal_mode_);
    }
  }

  // Absolute zoom goes through MAV_CMD_SET_CAMERA_ZOOM with ZOOM_TYPE_RANGE --
  // the path the working UI demo uses (tests/ui_demo/main.cpp:108). Two other
  // zoom APIs exist and neither does what it looks like:
  // setCameraZoomTarget() sends a raw zoom POSITION (payloadSdkInterface.cpp:
  // 1262-1271), and the C_V_ZM_CB_LV combine-factor parameter takes a discrete
  // ladder index. Both were tried on nx3 and neither moved the lens.
  //
  // MEASURED ON THIS PAYLOAD, 2026-08-19: the scale is INVERTED relative to the
  // MAVLink convention. The spec says 0 = wide and 100 = tele; here a LOWER
  // value is MORE zoomed in (50 is tighter than 75). Do not "fix" this to match
  // the spec without re-checking on the aircraft.
  void setZoomRange(float range)
  {
    my_payload->setCameraZoom(ZOOM_TYPE_RANGE, range);
  }

  // Wire contract for GimbalCommand.zoom: 0.0 = no zoom action (the value the
  // basestation's manual set_gimbal_angle path has always sent, so it must
  // stay a no-op), anything in (0, 100] = an absolute ZOOM RANGE POSITION.
  //
  // NOT a magnification. The field used to be documented as a zoom level and
  // carried 4.0 meaning "4x"; it now carries a payload range position where
  // LOWER IS MORE ZOOMED IN (measured, see setZoomRange). A magnification could
  // only be converted into this with the payload's zoom curve, which we do not
  // have, so the knob is the range itself and the value is chosen by looking at
  // the video. The only other publisher of this field is the basestation's
  // manual path, which sends 0.0 and is unaffected.
  //
  // Because 0.0 is the no-op sentinel, an exact 0.0 range (maximum tele) is not
  // reachable; use a small positive value.
  //
  // The gimbal pointer streams commands at 5 Hz, so re-sends are rate-limited.
  // A change is sent promptly (at most once a second); an UNCHANGED value is
  // re-asserted every zoom_reassert_sec. That re-assert is not redundancy for
  // its own sake: this used to latch on the first send, so the payload got
  // exactly ONE zoom command per inspection, and if that command was dropped --
  // payload still connecting, camera busy -- the camera stayed wide for the
  // whole inspection and never retried.
  void applyZoom(float zoom)
  {
    if (my_payload == nullptr || zoom <= 0.0f) {
      return;
    }
    if (zoom > 100.0f) {
      RCLCPP_WARN_ONCE(this->get_logger(),
                       "GimbalCommand.zoom %.1f is out of the 0-100 range scale; "
                       "this field is a range POSITION, not a magnification", zoom);
      return;
    }
    const auto now = this->get_clock()->now();
    const bool level_changed =
      last_zoom_commanded_ < 0.0f || std::fabs(zoom - last_zoom_commanded_) > 0.05f;
    const double since_sent =
      last_zoom_sent_.nanoseconds() > 0 ? (now - last_zoom_sent_).seconds() : 1e9;
    if (level_changed ? (since_sent < 1.0) : (since_sent < zoom_reassert_sec_)) {
      return;
    }
    setZoomRange(zoom);
    last_zoom_commanded_ = zoom;
    last_zoom_sent_ = now;
    if (level_changed) {
      // eo_zoom_ (PARAM_EO_ZOOM_LEVEL) is the OPTICAL magnification and is not
      // comparable to a range position, so it is logged as context, never as a
      // pass/fail check.
      RCLCPP_INFO(this->get_logger(),
                  "zoom range -> %.1f (lower = tighter); EO optical now %.1fx",
                  zoom, eo_zoom_.load());
    }
  }

  // ---- members ----
  T_ConnInfo conn_{};
  std::string gimbal_command_topic_;
  std::string move_gimbal_angle_topic_;
  float last_zoom_commanded_ = -1.0f;   // <0 = nothing commanded yet
  rclcpp::Time last_zoom_sent_{0, 0, RCL_ROS_TIME};
  double zoom_reassert_sec_ = 3.0;
  int gimbal_mode_ = PAYLOAD_CAMERA_GIMBAL_MODE_FOLLOW;
  double gimbal_mode_reassert_sec_ = 2.0;
  rclcpp::Time last_mode_sent_{0, 0, RCL_ROS_TIME};
  // True when a camera-parameter write has overwritten the SDK's
  // current_gimbal_mode and the next attitude command must re-assert it.
  bool gimbal_mode_dirty_ = true;

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gimbal_orientation_pub_;
  rclcpp::Publisher<lion_ros2_bridge::msg::Position>::SharedPtr position_pub_;
  rclcpp::Publisher<lion_ros2_bridge::msg::GimbalState>::SharedPtr gimbal_state_pub_;
  rclcpp::Publisher<lion_ros2_bridge::msg::LrfTrackingData>::SharedPtr lrf_pub_;
  rclcpp::TimerBase::SharedPtr position_timer_;
  std::string robot_name_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fc_gps_sub_;
  std::atomic<double> fc_lat_{0.0}, fc_lon_{0.0}, fc_alt_{0.0};
  std::atomic<int64_t> fc_rx_ns_{0};
  std::atomic<bool> fc_fix_{false};
  double fc_gps_max_age_sec_{1.0};
  double attitude_max_age_sec_{1.0};
  std::atomic<int64_t> ori_rx_ns_{0};
  double lrf_valid_min_m_{5.0}, lrf_valid_max_m_{300.0};
  std::atomic<double> pay_lat_{0.0}, pay_lon_{0.0}, pay_alt_{0.0};
  std::atomic<double> roll_deg_{0.0}, pitch_deg_{0.0}, yaw_deg_{0.0};
  std::atomic<double> eo_zoom_{1.0}, lrf_range_{0.0};
  std::atomic<int> gb_mode_{PAYLOAD_CAMERA_GIMBAL_MODE_OFF};
  std::atomic<bool> have_lat_{false}, have_lon_{false}, have_alt_{false}, have_orientation_{false}, have_zoom_{false}, have_lrf_{false}, have_mode_{false};
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
