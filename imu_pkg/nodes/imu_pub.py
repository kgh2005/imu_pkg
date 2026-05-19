#!/usr/bin/env python3

import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

from imu_pkg.drivers.ebimu_driver import EbimuDriver
from imu_pkg.converters.imu_converter import ImuConverter


class EbimuPublisher(Node):
    def __init__(self):
        super().__init__('ebimu_publisher')

        # =========================
        # Parameters
        # =========================
        self._declare_parameters()
        self._validate_parameters()

        # =========================
        # QoS
        # =========================
        qos = self._create_qos_profile()

        # =========================
        # Publishers
        # =========================
        self.imu_pub = self.create_publisher(Imu, self.topic_name, qos)
        self.gravity_pub = self.create_publisher(Vector3Stamped, self.gravity_topic_name, qos)

        # =========================
        # Driver / Converter
        # =========================
        self.driver = EbimuDriver(
            port=self.port,
            baud=self.baud
        )

        self.converter = ImuConverter(
            frame_id=self.frame_id,
            accel_scale=self.accel_scale,
            gyro_in_deg=self.gyro_in_deg,
            invert_accel_sign=self.invert_accel_sign,
            zero_orientation_on_start=self.zero_orientation_on_start
        )

        # =========================
        # Connect EBIMU
        # =========================
        self._connect_ebimu()

        # =========================
        # Timer
        # =========================
        timer_period = 1.0 / float(self.publish_rate_hz)

        self.timer = self.create_timer(
            timer_period,
            self.timer_callback
        )

        self._print_startup_info()

    def _declare_parameters(self):
        # Serial
        self.port = self.declare_parameter('port', '/dev/ttyUSB0').get_parameter_value().string_value
        self.baud = self.declare_parameter('baud', 115200).get_parameter_value().integer_value

        # Topic / frame
        self.topic_name = self.declare_parameter('topic_name', '/imu/data_raw').get_parameter_value().string_value
        self.gravity_topic_name = self.declare_parameter('gravity_topic_name', '/imu/gravity').get_parameter_value().string_value
        self.frame_id = self.declare_parameter('frame_id','imu_link').get_parameter_value().string_value

        # Publish rate
        self.publish_rate_hz = self.declare_parameter('publish_rate_hz', 100).get_parameter_value().integer_value

        # Unit conversion
        self.accel_scale = self.declare_parameter('accel_scale', 9.80665).get_parameter_value().double_value
        self.gyro_in_deg = self.declare_parameter('gyro_in_deg', True).get_parameter_value().bool_value
        self.invert_accel_sign = self.declare_parameter('invert_accel_sign', True).get_parameter_value().bool_value

        # Orientation
        self.zero_orientation_on_start = self.declare_parameter('zero_orientation_on_start', True).get_parameter_value().bool_value

        # QoS
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

    def _create_qos_profile(self):
        qos = QoSProfile(depth=self.qos_depth)
        qos.history = HistoryPolicy.KEEP_LAST

        if self.qos_reliability == 'best_effort':
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
        else:
            qos.reliability = ReliabilityPolicy.RELIABLE

        return qos

    def _connect_ebimu(self):
        try:
            self.driver.connect()
            self.driver.setup()
        except serial.SerialException as e:
            self.get_logger().error(f"EBIMU connection failed: {e}")
            raise

    def timer_callback(self):
        try:
            data = self.driver.read()
        except serial.SerialException as e:
            self.get_logger().warn(f"Serial read error: {e}")
            return

        if data is None:
            return

        stamp = self.get_clock().now().to_msg()

        imu_msg = self.converter.to_msg(
            data=data,
            stamp=stamp
        )

        gravity_msg = self.converter.to_gravity_msg(
            data=data,
            stamp=stamp
        )

        self.imu_pub.publish(imu_msg)
        self.gravity_pub.publish(gravity_msg)

    def _print_startup_info(self):
        self.get_logger().info(
            f"EBIMU publisher started\n"
            f"  Port          : {self.port}\n"
            f"  Baud          : {self.baud}\n"
            f"  IMU topic     : {self.topic_name}\n"
            f"  Gravity topic : {self.gravity_topic_name}\n"
            f"  Frame ID      : {self.frame_id}\n"
            f"  Rate          : {self.publish_rate_hz} Hz\n"
            f"  Zero start    : {self.zero_orientation_on_start}\n"
            f"  QoS depth     : {self.qos_depth}\n"
            f"  QoS reliability: {self.qos_reliability}"
        )

    def destroy_node(self):
        if hasattr(self, 'driver') and self.driver is not None:
            self.driver.close()
            self.get_logger().info("EBIMU serial closed")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = EbimuPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()