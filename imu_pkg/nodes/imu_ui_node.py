#!/usr/bin/env python3

import sys
import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from PyQt5.QtWidgets import QApplication

from imu_pkg.ui.imu_widget import ImuWidget
from imu_pkg.utils.rotation_utils import (
    normalize_angle_deg,
    quaternion_to_rpy,
    rpy_to_quaternion,
)


class ImuUiNode(Node):
    def __init__(self):
        super().__init__('imu_ui_node')

        self._declare_parameters()
        self._validate_parameters()

        qos = self._create_qos_profile()

        self.roll_raw = 0.0
        self.pitch_raw = 0.0
        self.yaw_raw = 0.0

        self.roll_corrected = 0.0
        self.pitch_corrected = 0.0
        self.yaw_corrected = 0.0

        self.roll_offset = self.initial_roll_offset_deg
        self.pitch_offset = self.initial_pitch_offset_deg
        self.yaw_offset = self.initial_yaw_offset_deg

        self.last_msg = None
        self.has_data = False

        self.sub = self.create_subscription(Imu, self.input_topic, self.imu_callback, qos)
        self.pub = self.create_publisher(Imu, self.output_topic, qos)

        self.publish_timer = self.create_timer(1.0 / float(self.publish_rate_hz), self.publish_zeroed_imu)

        self.get_logger().info(
            f"IMU UI node started\n"
            f"  Input topic    : {self.input_topic}\n"
            f"  Output topic   : {self.output_topic}\n"
            f"  Frame ID       : {self.frame_id}\n"
            f"  Publish enabled: {self.enable_publish}\n"
            f"  Publish rate   : {self.publish_rate_hz} Hz\n"
            f"  Initial offsets: "
            f"R={self.roll_offset:.2f}, "
            f"P={self.pitch_offset:.2f}, "
            f"Y={self.yaw_offset:.2f}\n"
            f"  QoS depth      : {self.qos_depth}\n"
            f"  QoS reliability: {self.qos_reliability}"
        )

    def _declare_parameters(self):
        self.input_topic = self.declare_parameter('input_topic', '/imu/data_raw').get_parameter_value().string_value
        self.output_topic = self.declare_parameter('output_topic', '/imu/data_zeroed').get_parameter_value().string_value
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').get_parameter_value().string_value

        self.enable_publish = self.declare_parameter('enable_publish', True).get_parameter_value().bool_value
        self.publish_rate_hz = self.declare_parameter('publish_rate_hz', 100).get_parameter_value().integer_value

        self.initial_roll_offset_deg = self.declare_parameter('initial_roll_offset_deg', 0.0).get_parameter_value().double_value
        self.initial_pitch_offset_deg = self.declare_parameter('initial_pitch_offset_deg', 0.0).get_parameter_value().double_value
        self.initial_yaw_offset_deg = self.declare_parameter('initial_yaw_offset_deg', 0.0).get_parameter_value().double_value

        self.roll_slider_min_deg = self.declare_parameter('roll_slider_min_deg', -90).get_parameter_value().integer_value
        self.roll_slider_max_deg = self.declare_parameter('roll_slider_max_deg', 90).get_parameter_value().integer_value
        self.pitch_slider_min_deg = self.declare_parameter('pitch_slider_min_deg', -90).get_parameter_value().integer_value
        self.pitch_slider_max_deg = self.declare_parameter('pitch_slider_max_deg', 90).get_parameter_value().integer_value

        self.qos_depth = self.declare_parameter('depth', 100).get_parameter_value().integer_value
        self.qos_reliability = self.declare_parameter('reliability', 'reliable').get_parameter_value().string_value

    def _validate_parameters(self):
        if self.publish_rate_hz <= 0:
            self.get_logger().warn(
                f"Invalid publish_rate_hz={self.publish_rate_hz}, fallback to 100Hz"
            )
            self.publish_rate_hz = 100

        if self.qos_depth <= 0:
            self.get_logger().warn(
                f"Invalid depth={self.qos_depth}, fallback to 100"
            )
            self.qos_depth = 100

        if self.qos_reliability not in ['reliable', 'best_effort']:
            self.get_logger().warn(
                f"Invalid reliability={self.qos_reliability}, fallback to reliable"
            )
            self.qos_reliability = 'reliable'

        if self.roll_slider_min_deg >= self.roll_slider_max_deg:
            self.roll_slider_min_deg = -90
            self.roll_slider_max_deg = 90

        if self.pitch_slider_min_deg >= self.pitch_slider_max_deg:
            self.pitch_slider_min_deg = -90
            self.pitch_slider_max_deg = 90

    def _create_qos_profile(self):
        qos = QoSProfile(depth=self.qos_depth)
        qos.history = HistoryPolicy.KEEP_LAST

        if self.qos_reliability == 'best_effort':
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
        else:
            qos.reliability = ReliabilityPolicy.RELIABLE

        return qos

    def imu_callback(self, msg):
        roll, pitch, yaw = quaternion_to_rpy(msg.orientation)

        self.roll_raw = normalize_angle_deg(roll)
        self.pitch_raw = normalize_angle_deg(pitch)
        self.yaw_raw = normalize_angle_deg(yaw)

        self.last_msg = msg
        self.has_data = True

    def update_corrected_rpy(self):
        self.roll_corrected = normalize_angle_deg(
            self.roll_raw - self.roll_offset
        )

        self.pitch_corrected = normalize_angle_deg(
            self.pitch_raw - self.pitch_offset
        )

        self.yaw_corrected = normalize_angle_deg(
            self.yaw_raw - self.yaw_offset
        )

    def publish_zeroed_imu(self):
        if not self.enable_publish:
            return

        if self.last_msg is None:
            return

        self.update_corrected_rpy()

        zeroed_msg = copy.deepcopy(self.last_msg)
        zeroed_msg.header.stamp = self.get_clock().now().to_msg()
        zeroed_msg.header.frame_id = self.frame_id

        qx, qy, qz, qw = rpy_to_quaternion(
            roll_deg=self.roll_corrected,
            pitch_deg=self.pitch_corrected,
            yaw_deg=self.yaw_corrected
        )

        zeroed_msg.orientation.x = qx
        zeroed_msg.orientation.y = qy
        zeroed_msg.orientation.z = qz
        zeroed_msg.orientation.w = qw

        self.pub.publish(zeroed_msg)

    def set_roll_zero(self):
        self.roll_offset = self.roll_raw

    def set_pitch_zero(self):
        self.pitch_offset = self.pitch_raw

    def set_yaw_zero(self):
        self.yaw_offset = self.yaw_raw

    def set_all_zero(self):
        self.roll_offset = self.roll_raw
        self.pitch_offset = self.pitch_raw
        self.yaw_offset = self.yaw_raw

    def clear_offsets(self):
        self.roll_offset = 0.0
        self.pitch_offset = 0.0
        self.yaw_offset = 0.0

    def set_yaw_preset(self, target_yaw_deg: float):
        self.yaw_offset = normalize_angle_deg(
            self.yaw_raw - target_yaw_deg
        )


def main(args=None):
    rclpy.init(args=args)

    ros_node = ImuUiNode()

    app = QApplication(sys.argv)
    ui = ImuWidget(ros_node)
    ui.show()

    try:
        app.exec_()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()