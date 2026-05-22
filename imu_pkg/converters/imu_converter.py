import math

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3Stamped


class ImuConverter:
    def __init__(
        self,
        frame_id: str,
        accel_scale: float = 9.80665,
        gyro_in_deg: bool = True,
        invert_accel_sign: bool = True,
        zero_orientation_on_start: bool = True
    ):
        self.frame_id = frame_id
        self.accel_scale = accel_scale
        self.gyro_in_deg = gyro_in_deg
        self.invert_accel_sign = invert_accel_sign
        self.zero_orientation_on_start = zero_orientation_on_start

        # =========================
        # Initial orientation
        # =========================
        self.initial_orientation = None
        self.orientation_initialized = False

        self.initial_yaw_offset_deg = -90.0
        self.initial_offset_quat = self.quaternion_from_yaw(
            math.radians(self.initial_yaw_offset_deg)
        )

    def quaternion_from_yaw(self, yaw):
        half = yaw * 0.5

        return [
            0.0,
            0.0,
            math.sin(half),
            math.cos(half)
        ]

    def to_msg(self, data, stamp):
        imu = Imu()

        imu.header.stamp = stamp
        imu.header.frame_id = self.frame_id

        self.fill_orientation(imu, data)
        self.fill_angular_velocity(imu, data)
        self.fill_linear_acceleration(imu, data)

        return imu

    def to_gravity_msg(self, data, stamp):
        gravity = Vector3Stamped()

        gravity.header.stamp = stamp
        gravity.header.frame_id = self.frame_id

        gx, gy, gz = self.compute_gravity_vector(data)

        gravity.vector.x = gx
        gravity.vector.y = gy
        gravity.vector.z = gz

        return gravity

    # =========================
    # Fill IMU fields
    # =========================

    def rotate_vector_by_quaternion(self, v, q):
        """
        벡터 v를 quaternion q로 회전시킨다.
        v' = q * v * q⁻¹
        """
        # v를 pure quaternion으로 표현 [x, y, z, w=0]
        q_v = [v[0], v[1], v[2], 0.0]
        q_inv = self.quaternion_inverse(q)

        rotated = self.quaternion_multiply(
            self.quaternion_multiply(q, q_v),
            q_inv
        )
        return rotated[0], rotated[1], rotated[2]
    def fill_orientation(self, imu, data):
        qx, qy, qz, qw = self.get_output_quaternion(data)

        imu.orientation = Quaternion(
            x=qx,
            y=qy,
            z=qz,
            w=qw
        )
    def fill_angular_velocity(self, imu, data):
        gx = data["gx"]
        gy = data["gy"]
        gz = data["gz"]

        if self.gyro_in_deg:
            gx = math.radians(gx)
            gy = math.radians(gy)
            gz = math.radians(gz)

        imu.angular_velocity.x = -gy
        imu.angular_velocity.y = gx
        imu.angular_velocity.z = gz
    # def fill_angular_velocity(self, imu, data):
    #     gx = data["gx"]
    #     gy = data["gy"]
    #     gz = data["gz"]

    #     if self.gyro_in_deg:
    #         gx = math.radians(gx)
    #         gy = math.radians(gy)
    #         gz = math.radians(gz)

    #     # # 기존 좌표계 변환 (Y, Z 반전)
    #     gx =  gx
    #     gy = -gy
    #     gz = gz

    #     # zero_orientation 보정
    #     if self.zero_orientation_on_start and self.orientation_initialized:
    #         q_initial_inv = self.quaternion_inverse(self.initial_orientation)

    #         q_gyro_correction = self.quaternion_multiply(
    #             q_initial_inv,
    #             self.initial_offset_quat
    #         )

    #         gx, gy, gz = self.rotate_vector_by_quaternion(
    #             [gx, gy, gz],
    #             q_gyro_correction
    #         )

    #     imu.angular_velocity.x = gx
    #     imu.angular_velocity.y = gy
    #     imu.angular_velocity.z = gz

    def fill_linear_acceleration(self, imu, data):
        sign = -1.0 if self.invert_accel_sign else 1.0

        imu.linear_acceleration.x = sign * data["ax"] * self.accel_scale
        imu.linear_acceleration.y = sign * data["ay"] * self.accel_scale
        imu.linear_acceleration.z = -sign * data["az"] * self.accel_scale

    # =========================
    # Orientation handling
    # =========================
    def get_raw_quaternion(self, data):
        """
        EBIMU quaternion을 ROS 좌표계에 맞게 변환한 quaternion.

        기존 코드에서 z축 부호를 반전하고 있었으므로 그대로 유지한다.
        """
        q = [
            data["qx"],
            -data["qy"],
            -data["qz"],
            data["qw"]
        ]

        return self.normalize_quaternion(q)

    def get_output_quaternion(self, data):
        """
        최종 publish할 orientation quaternion을 반환한다.

        zero_orientation_on_start=True:
            시작 시점 자세를 기준 0으로 만든 상대 자세 반환

        zero_orientation_on_start=False:
            EBIMU 원본 자세 반환
        """
        q_current = self.get_raw_quaternion(data)

        if not self.zero_orientation_on_start:
            return q_current

        if not self.orientation_initialized:
            self.initial_orientation = q_current
            self.orientation_initialized = True

            # 시작 순간에는 단위 quaternion을 내보낸다.
            return [0.0, 0.0, 0.0, 1.0]

        q_initial_inv = self.quaternion_inverse(self.initial_orientation)
        q_relative = self.quaternion_multiply(q_initial_inv, q_current)

        # WJ: roa humanoid mapping 
        q_output = self.quaternion_multiply(
            q_relative,
            self.initial_offset_quat
        )

        return self.normalize_quaternion(q_output)

    def reset_initial_orientation(self):
        """
        필요하면 외부에서 호출해서 초기 자세를 다시 잡을 수 있게 하는 함수.
        """
        self.initial_orientation = None
        self.orientation_initialized = False

    # =========================
    # Gravity
    # =========================
    def compute_gravity_vector(self, data):
        """
        Quaternion으로부터 IMU frame 기준 중력 방향 벡터를 계산한다.

        zero_orientation_on_start=True이면,
        시작 자세 기준으로 보정된 quaternion을 사용한다.

        반환값:
            단위 중력벡터(unit vector)
        """
        qx, qy, qz, qw = self.get_output_quaternion(data)

        gx = 2.0 * (qx * qz - qw * qy)
        gy = 2.0 * (qw * qx + qy * qz)
        gz = qw * qw - qx * qx - qy * qy + qz * qz

        return -gx, -gy, -gz # WJ 중력 가속도 방향으로 반전 처리 

    # =========================
    # Quaternion utilities
    # =========================
    def normalize_quaternion(self, q):
        x, y, z, w = q

        norm = math.sqrt(x*x + y*y + z*z + w*w)

        if norm == 0.0:
            return [0.0, 0.0, 0.0, 1.0]

        return [
            x / norm,
            y / norm,
            z / norm,
            w / norm
        ]

    def quaternion_inverse(self, q):
        x, y, z, w = q

        norm = x*x + y*y + z*z + w*w

        if norm == 0.0:
            return [0.0, 0.0, 0.0, 1.0]

        return [
            -x / norm,
            -y / norm,
            -z / norm,
             w / norm
        ]

    def quaternion_multiply(self, q1, q2):
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2

        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2
        w = w1*w2 - x1*x2 - y1*y2 - z1*z2

        return [x, y, z, w]