from PyQt5.QtWidgets import (
    QWidget,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QDial,
    QSlider,
    QGroupBox
)
from PyQt5.QtCore import Qt, QTimer

import rclpy

from imu_pkg.utils.rotation_utils import clamp


class ImuWidget(QWidget):
    def __init__(self, ros_node):
        super().__init__()

        self.node = ros_node

        self.setWindowTitle("IMU Roll / Pitch / Yaw UI")
        self.resize(600, 500)

        self._build_ui()

        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros_once)
        self.ros_timer.start(5)

        self.ui_timer = QTimer()
        self.ui_timer.timeout.connect(self.update_ui)
        self.ui_timer.start(30)

    def _build_ui(self):
        main_layout = QVBoxLayout()

        self.status_label = QLabel("Waiting for IMU data...")
        self.status_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.status_label)

        # =========================
        # Yaw
        # =========================
        yaw_group = QGroupBox("Yaw")
        yaw_layout = QVBoxLayout()

        self.yaw_dial = QDial()
        self.yaw_dial.setRange(-180, 180)
        self.yaw_dial.setNotchesVisible(True)
        self.yaw_dial.setEnabled(False)

        self.yaw_label = QLabel("Yaw: 0.0°")
        self.yaw_label.setAlignment(Qt.AlignCenter)

        yaw_layout.addWidget(self.yaw_dial)
        yaw_layout.addWidget(self.yaw_label)

        yaw_zero_layout = QHBoxLayout()

        self.yaw_zero_button = QPushButton("Yaw Set Zero")
        self.yaw_zero_button.clicked.connect(self.node.set_yaw_zero)

        yaw_zero_layout.addWidget(self.yaw_zero_button)
        yaw_layout.addLayout(yaw_zero_layout)

        yaw_preset_layout = QGridLayout()

        for idx, angle in enumerate([-90, 0, 90, 180]):
            btn = QPushButton(f"Yaw {angle}°")
            btn.clicked.connect(
                lambda checked, a=angle: self.node.set_yaw_preset(a)
            )
            yaw_preset_layout.addWidget(btn, 0, idx)

        yaw_layout.addLayout(yaw_preset_layout)

        yaw_group.setLayout(yaw_layout)
        main_layout.addWidget(yaw_group)

        # =========================
        # Roll / Pitch
        # =========================
        rp_group = QGroupBox("Roll / Pitch")
        rp_layout = QGridLayout()

        self.roll_slider = QSlider(Qt.Horizontal)
        self.roll_slider.setRange(
            self.node.roll_slider_min_deg,
            self.node.roll_slider_max_deg
        )
        self.roll_slider.setEnabled(False)

        self.pitch_slider = QSlider(Qt.Horizontal)
        self.pitch_slider.setRange(
            self.node.pitch_slider_min_deg,
            self.node.pitch_slider_max_deg
        )
        self.pitch_slider.setEnabled(False)

        self.roll_label = QLabel("Roll: 0.0°")
        self.pitch_label = QLabel("Pitch: 0.0°")

        self.roll_zero_button = QPushButton("Roll Set Zero")
        self.roll_zero_button.clicked.connect(self.node.set_roll_zero)

        self.pitch_zero_button = QPushButton("Pitch Set Zero")
        self.pitch_zero_button.clicked.connect(self.node.set_pitch_zero)

        rp_layout.addWidget(QLabel("Roll"), 0, 0)
        rp_layout.addWidget(self.roll_slider, 0, 1)
        rp_layout.addWidget(self.roll_label, 0, 2)
        rp_layout.addWidget(self.roll_zero_button, 0, 3)

        rp_layout.addWidget(QLabel("Pitch"), 1, 0)
        rp_layout.addWidget(self.pitch_slider, 1, 1)
        rp_layout.addWidget(self.pitch_label, 1, 2)
        rp_layout.addWidget(self.pitch_zero_button, 1, 3)

        rp_group.setLayout(rp_layout)
        main_layout.addWidget(rp_group)

        # =========================
        # Control
        # =========================
        control_layout = QHBoxLayout()

        self.all_zero_button = QPushButton("Set All Zero")
        self.all_zero_button.clicked.connect(self.node.set_all_zero)

        self.clear_zero_button = QPushButton("Clear Offset")
        self.clear_zero_button.clicked.connect(self.node.clear_offsets)

        control_layout.addWidget(self.all_zero_button)
        control_layout.addWidget(self.clear_zero_button)

        main_layout.addLayout(control_layout)

        self.offset_label = QLabel("Offset R:0.0 P:0.0 Y:0.0")
        self.offset_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.offset_label)

        self.publish_label = QLabel("Publishing /imu/data_zeroed")
        self.publish_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.publish_label)

        self.setLayout(main_layout)

    def spin_ros_once(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def update_ui(self):
        if not self.node.has_data:
            self.status_label.setText("Waiting for IMU data...")
            return

        self.node.update_corrected_rpy()

        roll = self.node.roll_corrected
        pitch = self.node.pitch_corrected
        yaw = self.node.yaw_corrected

        self.status_label.setText("IMU data received")

        self.yaw_dial.setValue(int(yaw))
        self.yaw_label.setText(
            f"Yaw: {yaw:.1f}°   raw: {self.node.yaw_raw:.1f}°"
        )

        roll_slider_value = int(
            clamp(
                roll,
                self.node.roll_slider_min_deg,
                self.node.roll_slider_max_deg
            )
        )

        pitch_slider_value = int(
            clamp(
                pitch,
                self.node.pitch_slider_min_deg,
                self.node.pitch_slider_max_deg
            )
        )

        self.roll_slider.setValue(roll_slider_value)
        self.pitch_slider.setValue(pitch_slider_value)

        self.roll_label.setText(
            f"Roll: {roll:.1f}°   raw: {self.node.roll_raw:.1f}°"
        )

        self.pitch_label.setText(
            f"Pitch: {pitch:.1f}°   raw: {self.node.pitch_raw:.1f}°"
        )

        self.offset_label.setText(
            f"Offset  "
            f"R:{self.node.roll_offset:.1f}  "
            f"P:{self.node.pitch_offset:.1f}  "
            f"Y:{self.node.yaw_offset:.1f}"
        )

        if self.node.enable_publish:
            self.publish_label.setText(
                f"Publishing {self.node.output_topic} @ "
                f"{self.node.publish_rate_hz}Hz  "
                f"R:{roll:.1f} P:{pitch:.1f} Y:{yaw:.1f}"
            )
        else:
            self.publish_label.setText("Publishing disabled")