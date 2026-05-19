# imu_pkg
**IMU** package

## Development Environment

| Component   | Version          |
|-------------|------------------|
| **OS**      | Ubuntu 22.04     |
| **ROS**     | Humble Hawksbill    |
| **IMU**     | EBIMU    |

## Build

```bash
colcon build --packages-select imu_pkg --symlink-install
```

## Run

```bash
ros2 launch imu_pkg imu_pkg.launch.py
```
