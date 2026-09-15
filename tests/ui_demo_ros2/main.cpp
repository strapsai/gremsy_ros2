// ui_demo_ros2: the payload control UI, but with a pure ROS2 backend.
//
// It reuses the exact same GTK widgets as ui_demo. The difference is that every
// control is *published* as a ROS2 command topic, and every readout is driven by
// a ROS2 telemetry subscription -- instead of talking to the Gremsy over UDP.
// So this app only needs a running spirit_driver on the drone; it never touches
// the Gremsy network. Run it anywhere ROS2 can reach the drone (incl. over X11).
//
// Topics are addressed by DRONE NAME: every cmd/* and telemetry topic is built
// as an absolute name under /<drone>/gremsy/, the namespace spirit_driver runs
// in on that aircraft. The "Drone" field at the top of the window holds the
// name (prefilled from the `drone` parameter / $ROBOT_NAME); pressing Connect
// (re)binds every publisher/subscription to that drone, so one UI can hop
// between spiritnx1/2/3 without a restart. Launch with:
//   ros2 launch gremsy_ros2 ui_demo_ros2.launch.py drone:=spiritnx3
// or plainly `ros2 run gremsy_ros2 ui_demo_ros2` and type the name.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <mutex>
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
    // gimbal_orientation is a pre-existing external topic (absolute by default,
    // matching spirit_driver's gimbal_orientation_topic); overridable per drone.
    gimbal_orientation_topic_ =
      node_->declare_parameter<std::string>("gimbal_orientation_topic", "/gimbal_orientation");
  }

  // Ask for a rebind to <drone>. Thread-safe; the actual (re)creation of the
  // ROS entities happens on the spin thread (see service_reconnect) so we never
  // tear down subscriptions underneath the executor that is delivering them.
  void request_connect(const std::string& drone)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    pending_drone_ = drone;
    reconnect_ = true;
  }

  // Called from the spin thread between spin_some() calls.
  void service_reconnect()
  {
    std::string drone;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!reconnect_) return;
      reconnect_ = false;
      drone = pending_drone_;
    }
    bind(drone);
  }

  // Called between spins. The boot IR zoom CANNOT be sent from bind(): the
  // publisher is created there, and a publish issued before DDS has matched
  // the driver's subscription is dropped on the floor with no error -- which
  // is exactly what happened the first time (payload stayed at ~7x). So wait
  // for a matched subscriber, then send once. Bounded, because a driver that
  // never appears must not leave this retrying forever.
  void service_boot_ir_zoom()
  {
    if (!boot_ir_zoom_pending_ || !set_cam_param_pub_) return;
    if (set_cam_param_pub_->get_subscription_count() == 0) {
      if (std::chrono::steady_clock::now() - boot_ir_zoom_since_ > std::chrono::seconds(15)) {
        boot_ir_zoom_pending_ = false;
        fprintf(stderr, "ui_demo_ros2: no subscriber on cmd/set_camera_param after 15s; "
                        "IR zoom NOT reset to 1x (is spirit_driver running?)\n");
      }
      return;
    }
    boot_ir_zoom_pending_ = false;
    pubIrZoom(0);
    fprintf(stderr, "ui_demo_ros2: IR zoom reset to 1x on connect\n");
  }

  std::string drone() const { std::lock_guard<std::mutex> lk(mtx_); return drone_; }

  // GTK-thread callback: translate a UI control event into a ROS2 command.
  void publishCommand(int event, double* param)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!gimbal_tilt_pub_) return;  // not bound to a drone yet
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
      case CAM_IR_ZOOM:              pubIrZoom((int)param[0]); break;
      case QUERY_PAYLOAD_PARAM:      query_params_pub_->publish(std_msgs::msg::Empty()); break;
      default: break;
    }
  }

private:
  // Absolute topic name under the drone's gremsy namespace: /<drone>/gremsy/<rel>
  std::string t(const char* rel) const { return "/" + drone_ + "/gremsy/" + rel; }

  // (Re)create every publisher/subscription for <drone>. Dropping the old
  // shared_ptrs unsubscribes/unadvertises from the previous drone.
  void bind(const std::string& drone)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    drone_ = drone;
    RCLCPP_INFO(node_->get_logger(), "binding to drone '%s' (topics under /%s/gremsy)", drone_.c_str(), drone_.c_str());

    // ---- Command publishers (ground -> drone) ----
    view_src_pub_ = node_->create_publisher<std_msgs::msg::Int32>(t(CMD_VIEW_SRC), 10);
    record_src_pub_ = node_->create_publisher<std_msgs::msg::Int32>(t(CMD_RECORD_SRC), 10);
    capture_pub_ = node_->create_publisher<std_msgs::msg::Empty>(t(CMD_CAPTURE), 10);
    record_pub_ = node_->create_publisher<std_msgs::msg::Bool>(t(CMD_RECORD), 10);
    eo_zoom_speed_pub_ = node_->create_publisher<std_msgs::msg::Int32>(t(CMD_EO_ZOOM_SPEED), 10);
    zoom_continuous_pub_ = node_->create_publisher<std_msgs::msg::Float64>(t(CMD_ZOOM_CONTINUOUS), 10);
    zoom_step_pub_ = node_->create_publisher<std_msgs::msg::Float64>(t(CMD_ZOOM_STEP), 10);
    zoom_range_pub_ = node_->create_publisher<std_msgs::msg::Float64>(t(CMD_ZOOM_RANGE), 10);
    eo_focus_speed_pub_ = node_->create_publisher<std_msgs::msg::Int32>(t(CMD_EO_FOCUS_SPEED), 10);
    focus_auto_pub_ = node_->create_publisher<std_msgs::msg::Empty>(t(CMD_FOCUS_AUTO), 10);
    focus_continuous_pub_ = node_->create_publisher<std_msgs::msg::Float64>(t(CMD_FOCUS_CONTINUOUS), 10);
    ae_mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>(t(CMD_AE_MODE), 10);
    shutter_pub_ = node_->create_publisher<std_msgs::msg::Int32>(t(CMD_SHUTTER), 10);
    iris_pub_ = node_->create_publisher<std_msgs::msg::Int32>(t(CMD_IRIS), 10);
    gain_pub_ = node_->create_publisher<std_msgs::msg::Int32>(t(CMD_GAIN), 10);
    white_balance_pub_ = node_->create_publisher<std_msgs::msg::Int32>(t(CMD_WHITE_BALANCE), 10);
    white_balance_trigger_pub_ = node_->create_publisher<std_msgs::msg::Empty>(t(CMD_WHITE_BALANCE_TRIGGER), 10);
    ir_palette_pub_ = node_->create_publisher<std_msgs::msg::Int32>(t(CMD_IR_PALETTE), 10);
    ir_ffc_mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>(t(CMD_IR_FFC_MODE), 10);
    ir_ffc_trigger_pub_ = node_->create_publisher<std_msgs::msg::Empty>(t(CMD_IR_FFC_TRIGGER), 10);
    lrf_mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>(t(CMD_LRF_MODE), 10);
    osd_mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>(t(CMD_OSD_MODE), 10);
    image_flip_pub_ = node_->create_publisher<std_msgs::msg::Int32>(t(CMD_IMAGE_FLIP), 10);
    track_mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>(t(CMD_TRACK_MODE), 10);
    track_touch_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>(t(CMD_TRACK_TOUCH), 10);
    track_pub_ = node_->create_publisher<std_msgs::msg::Int32>(t(CMD_TRACK), 10);
    gimbal_tilt_pub_ = node_->create_publisher<std_msgs::msg::Float64>(t(CMD_GIMBAL_TILT), 10);
    gimbal_pan_pub_ = node_->create_publisher<std_msgs::msg::Float64>(t(CMD_GIMBAL_PAN), 10);
    gimbal_angle_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>(t(CMD_GIMBAL_ANGLE), 10);
    gimbal_mode_pub_ = node_->create_publisher<std_msgs::msg::Int32>(t(CMD_GIMBAL_MODE), 10);
    query_params_pub_ = node_->create_publisher<std_msgs::msg::Empty>(t(CMD_QUERY_PARAMS), 10);
    set_cam_param_pub_ = node_->create_publisher<std_msgs::msg::String>(t(CMD_SET_CAMERA_PARAM), 10);

    // ---- Telemetry subscriptions (drone -> ground) ----
    gimbal_orientation_sub_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
      gimbal_orientation_topic_, 10, [](const geometry_msgs::msg::Vector3::SharedPtr m) {
        const double roll = m->x, pitch = m->y, yaw = m->z;
        post_to_ui([roll, pitch, yaw]() {
          if (window) window->update_gimbal_attitude(pitch, roll, yaw);
        });
      });

    // One subscription per payload parameter; reconstruct the (index,value) pair
    // the widgets expect from the SDK's param table.
    param_subs_.clear();
    for (const auto & p : payloadParams) {
      std::string id(p.id);
      for (auto & c : id) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
      const int index = p.index;
      param_subs_.push_back(node_->create_subscription<std_msgs::msg::Float64>(
        t((std::string(TLM_PARAMS_PREFIX) + id).c_str()), 10,
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
      t(TLM_CAM_PARAM), 10, [](const std_msgs::msg::String::SharedPtr m) {
        std::istringstream iss(m->data);
        std::string id;
        double value = 0.0;
        if (!(iss >> id >> value)) return;
        post_to_ui([id, value]() {
          if (window) window->update_payload_param(const_cast<char*>(id.c_str()), value);
        });
      });

    stream_uri_sub_ = node_->create_subscription<std_msgs::msg::String>(
      t(TLM_STREAM_URI), rclcpp::QoS(1).transient_local(), [](const std_msgs::msg::String::SharedPtr m) {
        const std::string uri = m->data;
        post_to_ui([uri]() {
          if (window) window->update_url_streaming(const_cast<char*>(uri.c_str()));
        });
      });

    storage_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      t(TLM_STORAGE), 10, [](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
        if (m->data.size() < 4) return;
        const double total = m->data[0], used = m->data[1], avail = m->data[2];
        const int status = static_cast<int>(m->data[3]);
        post_to_ui([status, total, used, avail]() {
          if (window) window->update_storage_info(status, total, used, avail);
        });
      });

    capture_sub_ = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
      t(TLM_CAPTURE), 10, [](const std_msgs::msg::Int32MultiArray::SharedPtr m) {
        if (m->data.size() < 4) return;
        const int a = m->data[0], b = m->data[1], c = m->data[2], d = m->data[3];
        post_to_ui([a, b, c, d]() {
          if (window) window->update_capture_info(a, b, c, d);
        });
      });

    // Prompt the driver to re-emit camera info/settings so the UI populates.
    query_params_pub_->publish(std_msgs::msg::Empty());

    // Always start at IR 1x. The payload keeps whatever digital zoom the last
    // run left it in, and a thermal frame that is silently 4x cropped looks
    // like a working camera pointed somewhere else. Same argument as the
    // driver's boot zoom for EO, but this one lives here because the UI is the
    // only thing that knows an operator just took the camera.
    boot_ir_zoom_pending_ = true;
    boot_ir_zoom_since_ = std::chrono::steady_clock::now();
  }

  // IR zoom rides the generic passthrough rather than a dedicated topic, so it
  // works against the driver that is ALREADY deployed -- no aircraft change.
  // "C_T_ZOOM" is PAYLOAD_CAMERA_IR_ZOOM_FACTOR; the value is the step index
  // (ZOOM_IR_1X..ZOOM_IR_8X = 0..7), not a magnification.
  void pubIrZoom(int step)
  {
    if (!set_cam_param_pub_) return;
    if (step < 0) step = 0;
    if (step > 7) step = 7;
    std_msgs::msg::String m;
    m.data = "C_T_ZOOM " + std::to_string(step);
    set_cam_param_pub_->publish(m);
  }

  void pubInt(const rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr& p, int v)
  { std_msgs::msg::Int32 m; m.data = v; p->publish(m); }
  void pubFloat(const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr& p, double v)
  { std_msgs::msg::Float64 m; m.data = v; p->publish(m); }
  void pubBool(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr& p, bool v)
  { std_msgs::msg::Bool m; m.data = v; p->publish(m); }
  void pubVec(const rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr& p, double x, double y, double z)
  { geometry_msgs::msg::Vector3 m; m.x = x; m.y = y; m.z = z; p->publish(m); }

  rclcpp::Node::SharedPtr node_;
  mutable std::mutex mtx_;
  std::string drone_;
  std::string pending_drone_;
  bool reconnect_ = false;
  std::string gimbal_orientation_topic_;

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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr set_cam_param_pub_;
  bool boot_ir_zoom_pending_ = false;
  std::chrono::steady_clock::time_point boot_ir_zoom_since_{};
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cam_param_sub_, stream_uri_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr storage_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr capture_sub_;
};

static RosBridge* bridge = nullptr;

static void onUICommandChanged(int event, double* param)
{
  if (bridge) bridge->publishCommand(event, param);
}

static void onUIConnectCommandChanged(int event, const char* param)
{
  // No UDP to open in ROS2 mode -- "connect" means: bind every topic to the
  // drone named in the field (/<drone>/gremsy/...), which also asks
  // spirit_driver to re-emit camera info, and flip the UI to connected.
  if (event == CONNECT_PAYLOAD) {
    std::string drone = param ? param : "";
    while (!drone.empty() && drone.front() == '/') drone.erase(drone.begin());
    while (!drone.empty() && drone.back() == '/') drone.pop_back();
    if (drone.empty()) {
      fprintf(stderr, "ui_demo_ros2: no drone name given -- type e.g. spiritnx3 and press Connect\n");
      return;
    }
    if (bridge) bridge->request_connect(drone);
    if (window) {
      window->set_title("Spirit Payload Control — ROS2 (" + drone + ")");
      window->send_connected();
    }
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
  // GStreamer autoplugging (decodebin) ranks the Jetson hardware decoder
  // nvv4l2decoder above the software ones. On a ground machine, or inside a
  // container without the Jetson device nodes, instantiating it SEGFAULTS the
  // whole UI the moment the RTSP stream connects (reproduced on nx-03,
  // 2026-09-01). Demote it so decodebin picks avdec_h264/h265 instead. Only a
  // default -- an explicit GST_PLUGIN_FEATURE_RANK in the environment wins.
  setenv("GST_PLUGIN_FEATURE_RANK", "nvv4l2decoder:NONE", /*overwrite=*/0);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ui_demo_ros2");

  // Which aircraft to talk to. Prefills the "Drone" field; the operator can
  // still type another name and press Connect. Falls back to $ROBOT_NAME.
  const char* env_robot = std::getenv("ROBOT_NAME");
  const std::string default_drone =
    node->declare_parameter<std::string>("drone", env_robot ? env_robot : "spiritnx3");

  auto app = Gtk::Application::create("org.airlab.ui_demo_ros2");
  apply_ros2_theme();

  // Size to the monitor's work area instead of a fixed 1920x1080: on a laptop
  // panel the fixed size overflows and the right-hand Payload Info column gets
  // clipped off-screen.
  int win_w = 1920, win_h = 1080;
  if (auto display = Gdk::Display::get_default()) {
    if (auto monitor = display->get_primary_monitor()) {
      Gdk::Rectangle wa;
      monitor->get_workarea(wa);
      if (wa.get_width() > 800 && wa.get_height() > 600) {
        win_w = std::min(win_w, wa.get_width());
        win_h = std::min(win_h, wa.get_height() - 40);  // leave room for the title bar
      }
    }
  }
  window = new MainWindow(win_w, win_h);
  window->set_title("Spirit Payload Control — ROS2");
  window->set_connect_field("Drone", "e.g. spiritnx3", default_drone, /*is_ip=*/false);
  window->regUICommandChanged(onUICommandChanged);
  window->regUIConnectCommandChanged(onUIConnectCommandChanged);

  RosBridge ros_bridge(node);
  bridge = &ros_bridge;

  // Spin ROS2 in the background; GTK owns the main thread. Rebinding to a
  // (new) drone is serviced here, between spins, never under the executor.
  std::atomic<bool> running{true};
  std::thread spin_thread([&]() {
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    while (running && rclcpp::ok()) {
      ros_bridge.service_reconnect();
      ros_bridge.service_boot_ir_zoom();
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
