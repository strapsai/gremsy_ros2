#!/usr/bin/env python3
"""Stand-in for spirit_driver so ui_demo_ros2 can be exercised with NO drone.

Publishes fake telemetry on the topics the UI subscribes to and logs every
command the UI publishes, all under /<drone>/gremsy (default spiritnx3), so the
loop "button press -> ROS2 -> driver" and "driver -> ROS2 -> widget" can be
verified end to end on a laptop. It never touches a Gremsy.

    ros2 run gremsy_ros2 mock_spirit_driver.py --ros-args -p drone:=spiritnx3 \
        -p cmd_log:=/tmp/mock_cmds.log -p stream_uri:=rtsp://127.0.0.1:8554/payload

Each received command is appended to cmd_log as "<ros time> <topic> <repr>".
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Empty, Float64, Float64MultiArray, Int32, Int32MultiArray, String
from geometry_msgs.msg import Vector3

# (relative topic, type) -- mirrors gremsy_ros_topics.h; keep in sync.
CMD_TOPICS = [
    ('cmd/view_src', Int32), ('cmd/toggle_eo_ir', Bool), ('cmd/dual_stream', Bool),
    ('cmd/record_src', Int32), ('cmd/capture', Empty), ('cmd/record', Bool),
    ('cmd/eo_zoom_speed', Int32), ('cmd/zoom_continuous', Float64), ('cmd/zoom_step', Float64),
    ('cmd/zoom_range', Float64), ('cmd/eo_focus_speed', Int32), ('cmd/focus_auto', Empty),
    ('cmd/focus_continuous', Float64), ('cmd/ae_mode', Int32), ('cmd/shutter', Int32),
    ('cmd/iris', Int32), ('cmd/gain', Int32), ('cmd/white_balance', Int32),
    ('cmd/white_balance_trigger', Empty), ('cmd/image_flip', Int32), ('cmd/osd_mode', Int32),
    ('cmd/ir_palette', Int32), ('cmd/ir_ffc_mode', Int32), ('cmd/ir_ffc_trigger', Empty),
    ('cmd/lrf_mode', Int32), ('cmd/track_mode', Int32), ('cmd/track_touch', Vector3),
    ('cmd/track', Int32), ('cmd/gimbal_tilt', Float64), ('cmd/gimbal_pan', Float64),
    ('cmd/gimbal_angle', Vector3), ('cmd/gimbal_mode', Int32), ('cmd/query_params', Empty),
    ('cmd/set_camera_param', String),
]


class MockSpiritDriver(Node):
    def __init__(self):
        super().__init__('mock_spirit_driver')
        drone = self.declare_parameter('drone', 'spiritnx3').value
        self.log_path = self.declare_parameter('cmd_log', '/tmp/mock_cmds.log').value
        stream_uri = self.declare_parameter('stream_uri', 'rtsp://127.0.0.1:8554/payload').value
        self.gimbal_orientation_topic = self.declare_parameter(
            'gimbal_orientation_topic', '/gimbal_orientation').value
        ns = f'/{drone}/gremsy/'
        self.t = lambda rel: ns + rel

        self.subs = [self.create_subscription(typ, self.t(rel), self._on_cmd(rel), 10)
                     for rel, typ in CMD_TOPICS]

        tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.stream_pub = self.create_publisher(String, self.t('camera/stream_uri'), tl)
        self.has_video_pub = self.create_publisher(Bool, self.t('camera/has_video_stream'), tl)
        self.orient_pub = self.create_publisher(Vector3, self.gimbal_orientation_topic, 10)
        self.param_pubs = {k: self.create_publisher(Float64, self.t('params/' + k), 10)
                           for k in ('eo_zoom', 'ir_zoom', 'gb_mode', 'lrf_range', 'view_mode',
                                     'pay_lat', 'pay_lon', 'pay_alt', 'ir_temp_max')}
        self.cam_param_pub = self.create_publisher(String, self.t('camera/param'), 10)
        self.storage_pub = self.create_publisher(Float64MultiArray, self.t('storage/info'), 10)
        self.capture_pub = self.create_publisher(Int32MultiArray, self.t('capture/status'), 10)

        self.stream_pub.publish(String(data=stream_uri))
        self.has_video_pub.publish(Bool(data=True))
        self.tick = 0
        self.eo_zoom = 1.2
        self.create_timer(0.5, self._telemetry)
        self.get_logger().info(f'mock driver up for {drone}; logging UI commands to {self.log_path}')
        open(self.log_path, 'a').close()

    def _on_cmd(self, rel):
        def cb(msg):
            line = f'{self.get_clock().now().nanoseconds} {self.t(rel)} {msg}'
            self.get_logger().info('CMD ' + line)
            with open(self.log_path, 'a') as f:
                f.write(line + '\n')
            # React like the real payload would to the zoom knobs so the readout moves.
            if rel == 'cmd/zoom_range':
                self.eo_zoom = 1.0 + msg.data / 100.0 * 239.0
            elif rel == 'cmd/query_params':
                for pid, val in (('C_V_ZM_MODE', 0), ('C_V_ZM_CB_LV', 2), ('C_S_VIEW', 1)):
                    self.cam_param_pub.publish(String(data=f'{pid} {val}'))
        return cb

    def _telemetry(self):
        self.tick += 1
        # Slow sweep so the attitude labels visibly change.
        self.orient_pub.publish(Vector3(x=0.5, y=-45.0 + (self.tick % 20), z=10.0 + (self.tick % 40)))
        vals = {'eo_zoom': self.eo_zoom, 'ir_zoom': 1.0, 'gb_mode': 2.0, 'lrf_range': 42.5,
                'view_mode': 1.0, 'pay_lat': 40.4433, 'pay_lon': -79.9436, 'pay_alt': 300.0,
                'ir_temp_max': 36.6}
        for k, v in vals.items():
            self.param_pubs[k].publish(Float64(data=float(v)))
        self.storage_pub.publish(Float64MultiArray(data=[61440.0, 12288.0, 49152.0, 1.0]))
        self.capture_pub.publish(Int32MultiArray(data=[0, 0, 7, 0]))


def main():
    rclpy.init(args=sys.argv)
    node = MockSpiritDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
