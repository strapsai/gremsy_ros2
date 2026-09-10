#ifndef GREMSY_ROS_TOPICS_H_
#define GREMSY_ROS_TOPICS_H_

// Shared ROS2 topic contract between the onboard bridge (spirit_driver) and the
// ground client (ui_demo_ros2). All names are RELATIVE, so they inherit the
// node's namespace (set at launch, e.g. /spiritnx3/gremsy) and can be remapped
// at launch time without rebuilding.
//
// spirit_driver SUBSCRIBES commands + PUBLISHES telemetry.
// ui_demo_ros2  PUBLISHES commands + SUBSCRIBES telemetry.
namespace gremsy_topics {

// ---- Command topics (ground -> drone) ----------------------------------
// Camera source / capture / record
constexpr const char* CMD_VIEW_SRC             = "cmd/view_src";              // std_msgs/Int32  (raw C_SOURCE enum)
constexpr const char* CMD_TOGGLE_EO_IR         = "cmd/toggle_eo_ir";          // std_msgs/Bool   (false=EO, true=IR)
constexpr const char* CMD_DUAL_STREAM          = "cmd/dual_stream";           // std_msgs/Bool   (true=EO+IR, false=EO)
constexpr const char* CMD_RECORD_SRC           = "cmd/record_src";            // std_msgs/Int32
constexpr const char* CMD_CAPTURE              = "cmd/capture";               // std_msgs/Empty
constexpr const char* CMD_RECORD               = "cmd/record";                // std_msgs/Bool   (true=start, false=stop)
// Zoom / focus
constexpr const char* CMD_EO_ZOOM_SPEED        = "cmd/eo_zoom_speed";         // std_msgs/Int32
constexpr const char* CMD_ZOOM_CONTINUOUS      = "cmd/zoom_continuous";       // std_msgs/Float64
constexpr const char* CMD_ZOOM_STEP            = "cmd/zoom_step";             // std_msgs/Float64
constexpr const char* CMD_ZOOM_RANGE           = "cmd/zoom_range";            // std_msgs/Float64
constexpr const char* CMD_EO_FOCUS_SPEED       = "cmd/eo_focus_speed";        // std_msgs/Int32
constexpr const char* CMD_FOCUS_AUTO           = "cmd/focus_auto";            // std_msgs/Empty
constexpr const char* CMD_FOCUS_CONTINUOUS     = "cmd/focus_continuous";      // std_msgs/Float64
// Exposure / image
constexpr const char* CMD_AE_MODE              = "cmd/ae_mode";               // std_msgs/Int32
constexpr const char* CMD_SHUTTER              = "cmd/shutter";               // std_msgs/Int32
constexpr const char* CMD_IRIS                 = "cmd/iris";                  // std_msgs/Int32
constexpr const char* CMD_GAIN                 = "cmd/gain";                  // std_msgs/Int32
constexpr const char* CMD_WHITE_BALANCE        = "cmd/white_balance";         // std_msgs/Int32
constexpr const char* CMD_WHITE_BALANCE_TRIGGER= "cmd/white_balance_trigger"; // std_msgs/Empty
constexpr const char* CMD_IMAGE_FLIP           = "cmd/image_flip";            // std_msgs/Int32
constexpr const char* CMD_OSD_MODE             = "cmd/osd_mode";              // std_msgs/Int32
// IR
constexpr const char* CMD_IR_PALETTE           = "cmd/ir_palette";            // std_msgs/Int32
constexpr const char* CMD_IR_FFC_MODE          = "cmd/ir_ffc_mode";           // std_msgs/Int32
constexpr const char* CMD_IR_FFC_TRIGGER       = "cmd/ir_ffc_trigger";        // std_msgs/Empty
// LRF
constexpr const char* CMD_LRF_MODE             = "cmd/lrf_mode";              // std_msgs/Int32
// Tracking
constexpr const char* CMD_TRACK_MODE           = "cmd/track_mode";            // std_msgs/Int32
constexpr const char* CMD_TRACK_TOUCH          = "cmd/track_touch";           // geometry_msgs/Vector3 (x,y pixels)
constexpr const char* CMD_TRACK                = "cmd/track";                 // std_msgs/Int32
// Gimbal. TILT/PAN each send a full 3-axis setpoint, so they cancel if both
// are published -- use CMD_GIMBAL_RATE for two axes.
constexpr const char* CMD_GIMBAL_TILT          = "cmd/gimbal_tilt";           // std_msgs/Float64 (deg/s), pitch only; yaw forced to 0
constexpr const char* CMD_GIMBAL_PAN           = "cmd/gimbal_pan";            // std_msgs/Float64 (deg/s), yaw only; pitch forced to 0
constexpr const char* CMD_GIMBAL_RATE          = "cmd/gimbal_rate";           // geometry_msgs/Vector3 (pitch,roll,yaw deg/s) -- all axes at once
constexpr const char* CMD_GIMBAL_ANGLE         = "cmd/gimbal_angle";          // geometry_msgs/Vector3 (pitch,roll,yaw deg)
constexpr const char* CMD_GIMBAL_MODE          = "cmd/gimbal_mode";           // std_msgs/Int32
// Misc / generic full-API passthrough
constexpr const char* CMD_QUERY_PARAMS         = "cmd/query_params";          // std_msgs/Empty
constexpr const char* CMD_SET_CAMERA_PARAM     = "cmd/set_camera_param";      // std_msgs/String "PARAM_ID VALUE [int32|uint32]"

// ---- Telemetry topics (drone -> ground) --------------------------------
constexpr const char* TLM_GIMBAL_ORIENTATION   = "gimbal_orientation";        // geometry_msgs/Vector3 (x=roll, y=pitch, z=yaw)
constexpr const char* TLM_PARAMS_PREFIX        = "params/";                   // std_msgs/Float64 per payload param, e.g. params/lrf_range
constexpr const char* TLM_CAM_PARAM            = "camera/param";              // std_msgs/String "PARAM_ID VALUE" (camera setting feedback)
constexpr const char* TLM_STREAM_URI           = "camera/stream_uri";         // std_msgs/String (RTSP uri)
constexpr const char* TLM_HAS_VIDEO            = "camera/has_video_stream";   // std_msgs/Bool
constexpr const char* TLM_STORAGE              = "storage/info";              // std_msgs/Float64MultiArray [total,used,available,status]
constexpr const char* TLM_CAPTURE              = "capture/status";            // std_msgs/Int32MultiArray [img_status,video_status,img_count,rec_time_ms]

}  // namespace gremsy_topics

#endif  // GREMSY_ROS_TOPICS_H_
