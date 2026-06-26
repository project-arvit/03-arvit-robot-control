"""Pure-logic SLAM/LIO helpers for ARVIT (no ROS, no robot, no network).

- :mod:`arvit_robot_control.slam.extrinsics` -- the SE(3) head-mount
  transform ``base_link <- unilidar_lidar`` (Architecture A "transform #2").
- :mod:`arvit_robot_control.slam.timesync` -- Point-LIO timestamp/clock
  helpers (IMU rate from integration step, IMU->LiDAR stamp shift, loop-back
  guard).
"""

from arvit_robot_control.slam.extrinsics import (
    HeadMount,
    invert_se3,
    make_se3,
    rotation_matrix,
    transform_point,
)
from arvit_robot_control.slam.timesync import (
    imu_rate_from_inte,
    is_monotonic_after_shift,
    shift_imu_to_lidar,
)

__all__ = [
    "HeadMount",
    "rotation_matrix",
    "make_se3",
    "transform_point",
    "invert_se3",
    "imu_rate_from_inte",
    "shift_imu_to_lidar",
    "is_monotonic_after_shift",
]
