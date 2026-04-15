#include <chrono>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "payloadSdkInterface.h"
#include "geometry_msgs/msg/vector3.hpp"

#include <humanflow_msgs/msg/human_flow_array.hpp>

using namespace std::chrono_literals;


T_ConnInfo s_conn = {
    CONTROL_UDP,
    udp_ip_target,
    udp_port_target
};
PayloadSdkInterface* my_payload = nullptr;

class MyPublisher : public rclcpp::Node
{
public:
  MyPublisher() : Node("my_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10);
    gimbal_orientation_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("gimbal_orientation", 10);
    
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&MyPublisher::timer_callback, this));

    move_gimbal_angle_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "move_gimbal_angle", 10, std::bind(&MyPublisher::move_gimbal_angle_mode_callback, this, std::placeholders::_1));
    humanflow_sub_ = this->create_subscription<humanflow_msgs::msg::HumanFlowArray>(
      "/drone1/demo/humanflow", 10, std::bind(&MyPublisher::humanflow_callback, this, std::placeholders::_1));
    my_payload = new PayloadSdkInterface(s_conn);
    my_payload->sdkInitConnection();

    my_payload->regPayloadStatusChanged(std::bind(&MyPublisher::onPayloadStatusChanged, this, std::placeholders::_1, std::placeholders::_2));

    my_payload->checkPayloadConnection();
    my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_RC_MODE, 
      PAYLOAD_CAMERA_RC_MODE_STANDARD, PARAM_TYPE_UINT32);
  }

private:
  int current_LRF_ID = -1;
  float current_LRF_lon = 0.0;
  float current_LRF_lat = 0.0;
  float current_LRF_alt = 0.0;
  void onPayloadStatusChanged(int event, double* param){
    // return;
    switch(event){
    case PAYLOAD_GB_ATTITUDE:{
      // param[0]: pitch
      // param[1]: roll
      // param[2]: yaw

      // note order is changed to convention roll, pitch, yaw
      gimbal_orientation_.x = param[1];
      gimbal_orientation_.y = param[0];
      gimbal_orientation_.z = param[2];

      gimbal_orientation_pub_->publish(gimbal_orientation_);

      // SDK_LOG("Pich: %.2f - Roll: %.2f - Yaw: %.2f", param[0], param[1], param[2]);
      break;
    }
    case PAYLOAD_PARAMS:{
      // param[0]: param index
      // param[1]: value
      if(param[0] == PARAM_EO_ZOOM_LEVEL){
        // SDK_LOG("Payload EO_ZOOM_LEVEL: %.2f ", param[1]);
      }
      else if(param[0] == PARAM_IR_ZOOM_LEVEL){
        // SDK_LOG("Payload IR_ZOOM_LEVEL: %.2f ", param[1]);
      }
      // #if defined VIO
      else if(param[0] == PARAM_LRF_RANGE){
        // SDK_LOG("Payload LRF_RANGE: %.2f ", param[1]);
      }
      else if(param[0] == PARAM_LRF_OFSET_X){
        // SDK_LOG("Payload PARAM_LRF_OFSET_X: %.2f ", param[1]);
      }
      else if(param[0] == PARAM_LRF_OFSET_Y){
        // SDK_LOG("Payload PARAM_LRF_OFSET_Y: %.2f ", param[1]);
      }
      else if(param[0] == PARAM_TARGET_COOR_LON){
        // RCLCPP_INFO(this->get_logger(),"Payload PARAM_TARGET_COOR_LON: %.6f ", param[1]);

        this->current_LRF_lon = param[1];
      }
      else if(param[0] == PARAM_TARGET_COOR_LAT){
        // RCLCPP_INFO(this->get_logger(),"Payload PARAM_TARGET_COOR_LAT: %.6f ", param[1]);
        this->current_LRF_lat = param[1];
      }
      else if(param[0] == PARAM_TARGET_COOR_ALT){
        // RCLCPP_INFO(this->get_logger(),"Payload PARAM_TARGET_COOR_ALT: %.6f ", param[1]);
        // RCLCPP_ERROR(this->get_logger(), "this->current_LRF_ID: %d", this->current_LRF_ID);
        this->current_LRF_alt = param[1];
      }

      else if(param[0] == PARAM_PAYLOAD_GPS_LON){
        // SDK_LOG("Payload PARAM_PAYLOAD_GPS_LON: %.6f ", param[1]);
      }
      else if(param[0] == PARAM_PAYLOAD_GPS_LAT){
        // SDK_LOG("Payload PARAM_PAYLOAD_GPS_LAT: %.6f ", param[1]);
      }
      else if(param[0] == PARAM_PAYLOAD_GPS_ALT){
        // SDK_LOG("Payload PARAM_PAYLOAD_GPS_ALT: %.6f ", param[1]);
      }

      else if(param[0] == PARAM_CAM_VIEW_MODE){
        // SDK_LOG("Payload PARAM_CAM_VIEW_MODE: %.2f ", param[1]);
      }
      else if(param[0] == PARAM_CAM_REC_SOURCE){
        // SDK_LOG("Payload PARAM_CAM_REC_SOURCE: %.2f ", param[1]);
      }
      else if(param[0] == PARAM_CAM_IR_TYPE){
        // SDK_LOG("Payload PARAM_CAM_IR_TYPE: %.2f ", param[1]);
      }
      else if(param[0] == PARAM_CAM_IR_PALETTE_ID){
        // SDK_LOG("Payload PARAM_CAM_IR_PALETTE_ID: %.2f ", param[1]);
      }
      else if(param[0] == PARAM_CAM_IR_FFC_MODE){
        // SDK_LOG("Payload PARAM_CAM_IR_FFC_MODE: %.2f ", param[1]);
      }
      else if(param[0] == PARAM_GIMBAL_MODE){
        // SDK_LOG("Payload PARAM_GIMBAL_MODE: %.2f ", param[1]);
      }
      else if(param[0] == PARAM_IR_TEMP_MAX){
        // SDK_LOG("Payload PARAM_IR_TEMP_MAX: %.2f ", param[1]);
      }
      else if(param[0] == PARAM_IR_TEMP_MIN){
        // SDK_LOG("Payload PARAM_IR_TEMP_MIN: %.2f ", param[1]);
      }
      else if(param[0] == PARAM_IR_TEMP_MEAN){
        // SDK_LOG("Payload PARAM_IR_TEMP_MEAN: %.2f ", param[1]);
      }
      // #endif /* VIO */
      break;
    }
    default: break;
    }
  }

  void timer_callback()
  {
    if (this->current_LRF_ID == -1)
      return;
    std::ostringstream oss;
    oss << "Person " << this->current_LRF_ID << " lat " << this->current_LRF_lat << " lon " << this->current_LRF_lon << " alt " << this->current_LRF_alt;
    auto message = std_msgs::msg::String();
    message.data = oss.str();
    publisher_->publish(message);
  }
  
  // Subscriptions:

  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr move_gimbal_angle_sub_;
  rclcpp::Subscription<humanflow_msgs::msg::HumanFlowArray>::SharedPtr humanflow_sub_;

  void move_gimbal_angle_mode_callback(const geometry_msgs::msg::Vector3 & msg) const
  {
    RCLCPP_INFO(this->get_logger(),"x: %f, y: %f, z: %f", msg.x, msg.y, msg.z);
    my_payload->setGimbalSpeed(msg.x, msg.y, msg.z, INPUT_ANGLE);
  }

  void humanflow_callback(const humanflow_msgs::msg::HumanFlowArray & msg) 
  {
    if (msg.humanflows.size() > 0){

        // RCLCPP_INFO(this->get_logger(),
        //     "bbox center=(%.1f, %.1f, theta=%.2f) size=(%.1f, %.1f)",
        //     msg.humanflows[0].bbox.center.position.x,   // or bbox.center.x depending on your version
        //     msg.humanflows[0].bbox.center.position.y,
        //     msg.humanflows[0].bbox.center.theta,
        //     msg.humanflows[0].bbox.size_x,
        //     msg.humanflows[0].bbox.size_y
        // );

        float x = msg.humanflows[0].bbox.center.position.x;
        float y = msg.humanflows[0].bbox.center.position.y;
        float w = msg.humanflows[0].bbox.size_x;
        float h = msg.humanflows[0].bbox.size_y;

        // 1920:1080 is what is usually used in the ui_demo, and 640:480 is the resized image in humanflow, so we do the following
        float hf_w = 640; // humanflow width
        float hf_h = 480;
        float gr_w = 1920;
        float gr_h = 1080; // gremsy height
        x = x * gr_w / hf_w;
        y = y * gr_h / hf_h;
        w = w * gr_w / hf_w;
        h = h * gr_h / hf_h;

        // // 1. First, enable tracking mode
        // my_payload->setPayloadCameraParam(PAYLOAD_CAMERA_TRACKING_MODE, 1, PARAM_TYPE_UINT32);

        // // 2. Then set the tracking to active/enabled
        // my_payload->setPayloadObjectTrackingMode(1);  // or whatever value enables tracking

        // 3. Finally, set the tracking position (the click)
        my_payload->setPayloadObjectTrackingPosition(x, y, w, h);

        if (x < gr_w/2 + 20 &&
            x > gr_w/2 - 20 &&
            y < gr_h/2 + 20 &&
            y > gr_h/2 - 20) {
              this->current_LRF_ID = msg.humanflows[0].tracking_id;
            }
        else{
          this->current_LRF_ID = -1;
        }
        
    }
    else {
        // RCLCPP_ERROR(this->get_logger(), "zeruh:");
        this->current_LRF_ID = -1;
    }
    // RCLCPP_ERROR(this->get_logger(), "size: %zu", msg.humanflows.size());
  }

  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gimbal_orientation_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Vector3 gimbal_orientation_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyPublisher>());
  rclcpp::shutdown();
  return 0;
}



