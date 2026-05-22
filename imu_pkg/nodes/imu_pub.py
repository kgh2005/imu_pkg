#!/usr/bin/env python3

import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

from imu_pkg.drivers.ebimu_driver import EbimuDriver
from imu_pkg.converters.imu_converter import ImuConverter

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


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
        self.tf_broadcaster = TransformBroadcaster(self)
        self.gravity_marker_topic_name = self.declare_parameter(
            'gravity_marker_topic_name',
            '/imu/gravity_marker'
        ).get_parameter_value().string_value

        self.gravity_marker_pub = self.create_publisher(
            Marker,
            self.gravity_marker_topic_name,
            qos
        )
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

        self._print_startup_info()

    def _declare_parameters(self):
        # Serial
        self.port = self.declare_parameter('port', '/dev/ttyUSB-EBIMU').get_parameter_value().string_value
        self.baud = self.declare_parameter('baud', 115200).get_parameter_value().integer_value

        # Topic / frame
        self.topic_name = self.declare_parameter('topic_name', '/imu/data').get_parameter_value().string_value
        self.gravity_topic_name = self.declare_parameter('gravity_topic_name', '/imu/gravity').get_parameter_value().string_value
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').get_parameter_value().string_value

        # Unit conversion
        self.accel_scale = self.declare_parameter('accel_scale', 9.80665).get_parameter_value().double_value
        self.gyro_in_deg = self.declare_parameter('gyro_in_deg', True).get_parameter_value().bool_value
        self.invert_accel_sign = self.declare_parameter('invert_accel_sign', True).get_parameter_value().bool_value

        # Orientation
        self.zero_orientation_on_start = self.declare_parameter(
            'zero_orientation_on_start',
            True
        ).get_parameter_value().bool_value

        # QoS
        self.qos_depth = self.declare_parameter('depth', 100).get_parameter_value().integer_value
        self.qos_reliability = self.declare_parameter('reliability', 'reliable').get_parameter_value().string_value

        self.fixed_frame_id = self.declare_parameter(
            'fixed_frame_id',
            'world'
        ).get_parameter_value().string_value

    def _validate_parameters(self):
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
        qos = QoSProfile(
            depth=self.qos_depth,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )
        return qos

    def _connect_ebimu(self):
        try:
            self.driver.connect()
            self.driver.setup()
        except serial.SerialException as e:
            self.get_logger().error(f"EBIMU connection failed: {e}")
            raise

    def publish_once(self):
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
        gravity_marker = self.make_gravity_marker(gravity_msg)

        self.imu_pub.publish(imu_msg)
        self.gravity_pub.publish(gravity_msg)
        self.publish_orientation_tf(imu_msg)
        self.gravity_marker_pub.publish(gravity_marker)

    def make_gravity_marker(self, gravity_msg):
        marker = Marker()

        marker.header.stamp = gravity_msg.header.stamp
        marker.header.frame_id = gravity_msg.header.frame_id

        marker.ns = "gravity_vector"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # 시작점: IMU frame 원점
        start = Point()
        start.x = 0.0
        start.y = 0.0
        start.z = 0.0

        # 끝점: 중력 방향
        scale = 1.0

        end = Point()
        end.x = gravity_msg.vector.x * scale
        end.y = gravity_msg.vector.y * scale
        end.z = gravity_msg.vector.z * scale

        marker.points = [start, end]

        # 화살표 크기
        marker.scale.x = 0.03   # shaft diameter
        marker.scale.y = 0.08   # head diameter
        marker.scale.z = 0.12   # head length

        # 색상: 노란색 계열
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        return marker

    def publish_orientation_tf(self, imu_msg):
        tf_msg = TransformStamped()

        tf_msg.header.stamp = imu_msg.header.stamp
        tf_msg.header.frame_id = self.fixed_frame_id
        tf_msg.child_frame_id = self.frame_id

        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0

        tf_msg.transform.rotation = imu_msg.orientation

        self.tf_broadcaster.sendTransform(tf_msg)

    def _print_startup_info(self):
        self.get_logger().info(
            f"EBIMU publisher started\n"
            f"  Port           : {self.port}\n"
            f"  Baud           : {self.baud}\n"
            f"  IMU topic      : {self.topic_name}\n"
            f"  Gravity topic  : {self.gravity_topic_name}\n"
            f"  Frame ID       : {self.frame_id}\n"
            f"  Publish mode   : on serial read\n"
            f"  Zero start     : {self.zero_orientation_on_start}\n"
            f"  QoS depth      : {self.qos_depth}\n"
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
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            node.publish_once()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()