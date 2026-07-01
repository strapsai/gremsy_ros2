// ui_demo_ros2: the payload control UI, but with a pure ROS2 backend.
//
// It reuses the exact same GTK widgets as ui_demo. The difference is that every
// control is *published* as a ROS2 command topic, and every readout is driven by
// a ROS2 telemetry subscription -- instead of talking to the Gremsy over UDP.
// So this app only needs a running spirit_driver on the drone; it never touches
// the Gremsy network. Run it anywhere ROS2 can reach the drone (incl. over X11).
//
// The node runs under the same namespace as spirit_driver (e.g. /spiritnx3/gremsy)
// so the relative topic names line up. Launch with:
//   ros2 run gremsy_ros2 ui_demo_ros2 --ros-args -r __ns:=/spiritnx3/gremsy

#include <atomic>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gtkmm.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "MainWindow.h"
#include "payloadSdkInterface.h"
#include "gremsy_ros_topics.h"

// GTK/gdkx pulls in X11's Xlib.h, which #defines Bool/None/Status/... as macros
// that collide with std_msgs::msg::Bool and friends. Undo them.
#ifdef Bool
#undef Bool
#endif
#ifdef None
#undef None
#endif
#ifdef Status
#undef Status
#endif
#ifdef Success
#undef Success
#endif

using namespace gremsy_topics;

static MainWindow* window = nullptr;

// Run a callable on the GTK main thread (thread-safe from any thread).
static void post_to_ui(std::function<void()> fn)
{
  auto* f = new std::function<void()>(std::move(fn));
  g_idle_add(
    [](gpointer data) -> gboolean {
      auto* fp = static_cast<std::function<void()>*>(data);
      (*fp)();
      delete fp;
      return G_SOURCE_REMOVE;
    },
    f);
}

class RosBridge
{
public:
  explicit RosBridge(rclcpp::Node::SharedPtr node) : node_(std::move(node))
  {
    // ---- Command publishers (ground -> drone) ----
    view_src_pub_ = node_->create_publisher<std_msgs::msg::Int32>(CMD_VIEW_SRC, 10);
    record_src_pub_ = node_->create_publisher<std_msgs::msg::Int32>(CMD_RECORD_SRC, 10);
    capture_pub_ = node_->create_publisher<std_msgs::msg::Empty>(CMD_CAPTURE, 10);
    record_pub_ = node_->create_publisher<std_msgs::msg::Bool>(CMD_RECORD, 10);
    eo_zoom_speed_pub_ = node_->create_publisher<std_msgs::msg::Int32>(CMD_EO_ZOOM_SPEED, 10);
    zoom_continuous_pub_ = node_->create_publisher<std_msgs::msg::Float64>(CMD_ZOOM_CONTINUOUS, 10);
    zoom_step_pub_ = node_->create_publisher<std_msgs::msg::Float64>(CMD_ZOOM_STEP, 10);
    zoom_range_pub_ = node_->create_publisher<std_msgs::msg::Float64>(CMD_ZOOM_RANGE, 10);
    eo_focus_speed_pub_ = node_->create_publisher<std_msgs::msg::Int32>(CMD_EO_FOCUS_SPEED, 10);
    focus_auto_pub_ = node_->create_publisher<std_msgs::msg::Empty>(CMD_FOCUS_AUTO, 10);
    focus_continuous_pub_ = node_->create_publisher<std_msgs::msg::Float64>(CMD_FOCUS_CONTINUOUS, 10);
    ae_mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>(CMD_AE_MODE, 10);
    shutter_pub_ = node_->create_publisher<std_msgs::msg::Int32>(CMD_SHUTTER, 10);
    iris_pub_ = node_->create_publisher<std_msgs::msg::Int32>(CMD_IRIS, 10);
    gain_pub_ = node_->create_publisher<std_msgs::msg::Int32>(CMD_GAIN, 10);
    white_balance_pub_ = node_->create_publisher<std_msgs::msg::Int32>(CMD_WHITE_BALANCE, 10);
    white_balance_trigger_pub_ = node_->create_publisher<std_msgs::msg::Empty>(CMD_WHITE_BALANCE_TRIGGER, 10);
    ir_palette_pub_ = node_->create_publisher<std_msgs::msg::Int32>(CMD_IR_PALETTE, 10);
    ir_ffc_mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>(CMD_IR_FFC_MODE, 10);
    ir_ffc_trigger_pub_ = node_->create_publisher<std_msgs::msg::Empty>(CMD_IR_FFC_TRIGGER, 10);
    lrf_mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>(CMD_LRF_MODE, 10);
    osd_mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>(CMD_OSD_MODE, 10);
    image_flip_pub_ = node_->create_publisher<std_msgs::msg::Int32>(CMD_IMAGE_FLIP, 10);
    track_mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>(CMD_TRACK_MODE, 10);
    track_touch_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>(CMD_TRACK_TOUCH, 10);
    track_pub_ = node_->create_publisher<std_msgs::msg::Int32>(CMD_TRACK, 10);
    gimbal_tilt_pub_ = node_->create_publisher<std_msgs::msg::Float64>(CMD_GIMBAL_TILT, 10);
    gimbal_pan_pub_ = node_->create_publisher<std_msgs::msg::Float64>(CMD_GIMBAL_PAN, 10);
    gimbal_angle_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>(CMD_GIMBAL_ANGLE, 10);
    gimbal_mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>(CMD_GIMBAL_MODE, 10);
    query_params_pub_ = node_->create_publisher<std_msgs::msg::Empty>(CMD_QUERY_PARAMS, 10);

    // ---- Telemetry subscriptions (drone -> ground) ----
    // gimbal_orientation is a pre-existing external topic (absolute by default,
    // matching spirit_driver's gimbal_orientation_topic); overridable per drone.
    const std::string gimbal_orientation_topic =
      node_->declare_parameter<std::string>("gimbal_orientation_topic", "/gimbal_orientation");
    gimbal_orientation_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
      gimbal_orientation_topic, 10, [](const geometry_msgs::msg::Vector3::SharedPtr m) {
        const double roll = m->x, pitch = m->y, yaw = m->z;
        post_to_ui([roll, pitch, yaw]() {
          if (window) window->update_gimbal_attitude(pitch, roll, yaw);
        });
      });

    // One subscription per payload parameter; reconstruct the (index,value) pair
    // the widgets expect from the SDK's param table.
    for (const auto & p : payloadParams) {
      std::string id(p.id);
      for (auto & c : id) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      const int index = p.index;
      param_subs_.push_back(node_->create_subscription<std_msgs::msg::Float64>(
        std::string(TLM_PARAMS_PREFIX) + id, 10,
        [index](const std_msgs::msg::Float64::SharedPtr m) {
          const double value = m->data;
          post_to_ui([index, value]() {
            if (window) {
              double params[2] = {static_cast<double>(index), value};
              window->update_payload_status(params);
            }
          });
        }));
    }

    cam_param_sub_ = node_->create_subscription<std_msgs::msg::String>(
      TLM_CAM_PARAM, 10, [](const std_msgs::msg::String::SharedPtr m) {
        std::istringstream iss(m->data);
        std::string id;
        double value = 0.0;
        if (!(iss >> id >> value)) return;
        post_to_ui([id, value]() {
          if (window) window->update_payload_param(const_cast<char*>(id.c_str()), value);
        });
      });

    stream_uri_sub_ = node_->create_subscription<std_msgs::msg::String>(
      TLM_STREAM_URI, rclcpp::QoS(1).transient_local(), [](const std_msgs::msg::String::SharedPtr m) {
        const std::string uri = m->data;
        post_to_ui([uri]() {
          if (window) window->update_url_streaming(const_cast<char*>(uri.c_str()));
        });
      });

    storage_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      TLM_STORAGE, 10, [](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
        if (m->data.size() < 4) return;
        const double total = m->data[0], used = m->data[1], avail = m->data[2];
        const int status = static_cast<int>(m->data[3]);
        post_to_ui([status, total, used, avail]() {
          if (window) window->update_storage_info(status, total, used, avail);
        });
      });

    capture_sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
      TLM_CAPTURE, 10, [](const std_msgs::msg::Int32MultiArray::SharedPtr m) {
        if (m->data.size() < 4) return;
        const int a = m->data[0], b = m->data[1], c = m->data[2], d = m->data[3];
        post_to_ui([a, b, c, d]() {
          if (window) window->update_capture_info(a, b, c, d);
        });
      });
  }

  // GTK-thread callback: translate a UI control event into a ROS2 command.
  void publishCommand(int event, double* param)
  {
    switch (event) {
      case CAM_VIEW_MODE:            pubInt(view_src_pub_, (int)param[0]); break;
      case CAM_SOURCE_RECORD:        pubInt(record_src_pub_, (int)param[0]); break;
      case CAM_CAPTURE:              capture_pub_->publish(std_msgs::msg::Empty()); break;
      case CAM_RECORD:               pubBool(record_pub_, param[0] == 0.0); break;  // 0=start,1=stop
      case CAM_EO_SPEED_ZOOM:        pubInt(eo_zoom_speed_pub_, (int)param[0]); break;
      case CAM_ZOOM_CONTINIOUS:      pubFloat(zoom_continuous_pub_, param[0]); break;
      case CAM_ZOOM_STEP:            pubFloat(zoom_step_pub_, param[0]); break;
      case CAM_ZOOM_RANGE:           pubFloat(zoom_range_pub_, param[0]); break;
      case CAM_EO_SPEED_FOCUS:       pubInt(eo_focus_speed_pub_, (int)param[0]); break;
      case CAM_FOCUS_AUTO:           focus_auto_pub_->publish(std_msgs::msg::Empty()); break;
      case CAM_FOCUS_CONTINIOUS:     pubFloat(focus_continuous_pub_, param[0]); break;
      case CAM_AE_MODE:              pubInt(ae_mode_pub_, (int)param[0]); break;
      case CAM_SHUTTER:              pubInt(shutter_pub_, (int)param[0]); break;
      case CAM_IRIS:                 pubInt(iris_pub_, (int)param[0]); break;
      case CAM_GAIN:                 pubInt(gain_pub_, (int)param[0]); break;
      case CAM_WHITE_BALANCE:        pubInt(white_balance_pub_, (int)param[0]); break;
      case CAM_WHITE_BALANCE_TRIGGER:white_balance_trigger_pub_->publish(std_msgs::msg::Empty()); break;
      case CAM_IR_PALETTE:           pubInt(ir_palette_pub_, (int)param[0]); break;
      case CAM_IR_FFC_MODE:          pubInt(ir_ffc_mode_pub_, (int)param[0]); break;
      case CAM_IR_FFC_TRIGGER:       ir_ffc_trigger_pub_->publish(std_msgs::msg::Empty()); break;
      case CAM_LRF_MODE:             pubInt(lrf_mode_pub_, (int)param[0]); break;
      case CAM_OSD_MODE:             pubInt(osd_mode_pub_, (int)param[0]); break;
      case CAM_IMAGE_FLIP:           pubInt(image_flip_pub_, (int)param[0]); break;
      case PAYLOAD_TRACK_MODE:       pubInt(track_mode_pub_, (int)param[0]); break;
      case PAYLOAD_TOUCH:            pubVec(track_touch_pub_, param[0], param[1], 0); break;
      case PAYLOAD_TRACK:            pubInt(track_pub_, (int)param[0]); break;
      case GIMBAL_CONTROL_TILT:      pubFloat(gimbal_tilt_pub_, param[0]); break;
      case GIMBAL_CONTROL_PAN:       pubFloat(gimbal_pan_pub_, param[0]); break;
      case GIMBAL_CONTROL_ANGLE:     pubVec(gimbal_angle_pub_, param[0], param[1], param[2]); break;
      case GIMBAL_MODE:              pubInt(gimbal_mode_pub_, (int)param[0]); break;
      case QUERY_PAYLOAD_PARAM:      query_params_pub_->publish(std_msgs::msg::Empty()); break;
      default: break;
    }
  }

private:
  void pubInt(const rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr& p, int v)
  { std_msgs::msg::Int32 m; m.data = v; p->publish(m); }
  void pubFloat(const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& p, double v)
  { std_msgs::msg::Float64 m; m.data = v; p->publish(m); }
  void pubBool(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr& p, bool v)
  { std_msgs::msg::Bool m; m.data = v; p->publish(m); }
  void pubVec(const rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr& p, double x, double y, double z)
  { geometry_msgs::msg::Vector3 m; m.x = x; m.y = y; m.z = z; p->publish(m); }

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr view_src_pub_, record_src_pub_, eo_zoom_speed_pub_,
    eo_focus_speed_pub_, ae_mode_pub_, shutter_pub_, iris_pub_, gain_pub_, white_balance_pub_, ir_palette_pub_,
    ir_ffc_mode_pub_, lrf_mode_pub_, osd_mode_pub_, image_flip_pub_, track_mode_pub_, track_pub_, gimbal_mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr zoom_continuous_pub_, zoom_step_pub_, zoom_range_pub_,
    focus_continuous_pub_, gimbal_tilt_pub_, gimbal_pan_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr record_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr capture_pub_, focus_auto_pub_, white_balance_trigger_pub_,
    ir_ffc_trigger_pub_, query_params_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr track_touch_pub_, gimbal_angle_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr gimbal_orientation_sub_;
  std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> param_subs_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cam_param_sub_, stream_uri_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr storage_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr capture_sub_;
};

static RosBridge* bridge = nullptr;

static void onUICommandChanged(int event, double* param)
{
  if (bridge) bridge->publishCommand(event, param);
}

static void onUIConnectCommandChanged(int event, const char* /*param*/)
{
  // No UDP to open in ROS2 mode -- "connect" just means start listening / prompt
  // the driver to re-emit camera info, and flip the UI to the connected state.
  if (event == CONNECT_PAYLOAD) {
    if (window) window->send_connected();
    // Ask spirit_driver to (re)publish camera info/settings so the UI populates.
    onUICommandChanged(QUERY_PAYLOAD_PARAM, nullptr);
  } else if (event == DISCONNECT_PAYLOAD) {
    if (window) window->send_disconnected();
  }
}

// Distinct look so it is obvious this is the ROS2 client (indigo/teal theme).
static void apply_ros2_theme()
{
  auto css = Gtk::CssProvider::create();
  css->load_from_data(R"CSS(
    window, .background { background-color: #12141c; color: #e8ecff; }
    label { color: #dfe4ff; }
    notebook header.top tabs tab {
      background: #1b2030; color: #b9c4ff; padding: 6px 14px;
      border-top: 2px solid transparent;
    }
    notebook header.top tabs tab:checked {
      background: #232a44; color: #ffffff; border-top: 2px solid #7c5cff;
    }
    button {
      background-image: none; background-color: #2a3350; color: #eef1ff;
      border: 1px solid #3d4a72; border-radius: 8px; padding: 6px 12px;
    }
    button:hover { background-color: #35406a; }
    button:active, button:checked { background-color: #7c5cff; color: #ffffff; }
    entry, combobox, spinbutton {
      background-color: #1b2030; color: #eef1ff;
      border: 1px solid #3d4a72; border-radius: 6px;
    }
    frame > border, frame { border-color: #3d4a72; }
    frame > label { color: #8ad9ff; font-weight: bold; }
  )CSS");
  Gtk::StyleContext::add_provider_for_screen(
    Gdk::Screen::get_default(), css, GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ui_demo_ros2");

  auto app = Gtk::Application::create("org.airlab.ui_demo_ros2");
  apply_ros2_theme();

  window = new MainWindow(1920, 1080);
  window->set_title("Spirit Payload Control — ROS2");
  window->regUICommandChanged(onUICommandChanged);
  window->regUIConnectCommandChanged(onUIConnectCommandChanged);

  RosBridge ros_bridge(node);
  bridge = &ros_bridge;

  // Spin ROS2 in the background; GTK owns the main thread.
  std::atomic<bool> running{true};
  std::thread spin_thread([&]() {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    while (running && rclcpp::ok()) {
      exec.spin_some(std::chrono::milliseconds(50));
    }
  });

  app->run(*window);

  running = false;
  rclcpp::shutdown();
  if (spin_thread.joinable()) spin_thread.join();
  bridge = nullptr;
  delete window;
  window = nullptr;
  return 0;
}
