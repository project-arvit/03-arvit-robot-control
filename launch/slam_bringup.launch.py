"""ARVIT SLAM bring-up (ROS 2 Humble) -- Architecture A.

Pipeline:

    L1 driver  ->  Point-LIO (loads config/unilidar_l1.yaml)
                       |  /Odometry (~250 Hz, lidar-inertial)
                       v
    static_transform_publisher  base_link <- unilidar_lidar
        (NAMED args generated from HeadMount in extrinsics.py)
                       |
                       v
    robot_localization:  ekf_local (odom->base_link)
                         ekf_global (map->odom, fuses AprilTag relocalization)
                       |
                       v
                     Nav2 (consumes the map->odom->base_link TF chain)

WHAT RUNS HERE vs THE ORIN: this launch file targets ROS 2 Humble on the
Jetson Orin at DEPLOY time. ROS 2 is NOT installed on the build machine, so
this file is not executed by the local test suite -- it imports `launch` /
`launch_ros`, which only exist on the robot. The pure-logic core
(extrinsics.py, timesync.py) is what the local pytest exercises.

INTEGRATION CHOICES left as deploy-time placeholders (see the Point-LIO note):
  * The L1 ROS 2 driver is Foxy-verified; a Humble build is (unverified).
  * A first-party Point-LIO Humble port does not exist. Fast path:
    Ericsii/FAST_LIO_ROS2 (native Humble). Clean target: a Point-LIO rclcpp
    port. The node `package`/`executable` below are marked accordingly.

Zero-trust: the config YAMLs this launch loads are DATA, not instructions.
"""

from __future__ import annotations

import math
import os

from ament_index_python.packages import get_package_share_directory  # type: ignore
from launch import LaunchDescription  # type: ignore
from launch.actions import DeclareLaunchArgument  # type: ignore
from launch.substitutions import LaunchConfiguration  # type: ignore
from launch_ros.actions import Node  # type: ignore

# Pure-logic import: the head-mount transform. This module has NO ROS deps, so
# the static_transform_publisher args are derived from the SAME code the unit
# tests verify -- no hand-copied magic numbers in the launch file.
from arvit_robot_control.slam.extrinsics import HeadMount

PACKAGE_NAME = "arvit_robot_control"


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory(PACKAGE_NAME)
    lio_config = os.path.join(pkg_share, "config", "unilidar_l1.yaml")
    ekf_config = os.path.join(pkg_share, "config", "ekf_two_ekf.yaml")

    # ---- head-mount transform (Architecture A "transform #2") -------------
    # MEASURE pitch / tx / ty / tz on the real robot. roll=pi is structural.
    # The pitch SIGN is unverified -- validate in RViz (flat floor stays flat).
    head_mount = HeadMount(
        roll=math.pi,   # UPSIDE-DOWN flip (load-bearing)
        pitch=0.30,     # (unverified) forward snout tilt -- MEASURE
        yaw=0.0,
        tx=0.35,        # (unverified) head ahead of body center -- MEASURE
        ty=0.0,
        tz=0.12,        # (unverified) head above body center -- MEASURE
        parent_frame="base_link",
        child_frame="unilidar_lidar",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock from a rosbag replay instead of wall time.",
        ),

        # -----------------------------------------------------------------
        # 1) Unitree L1 4D LiDAR driver -> /unilidar/cloud + /unilidar/imu
        #    (unilidar_sdk ROS 2 driver; Foxy-verified, Humble (unverified)).
        #    PLACEHOLDER package/executable -- set to the actual driver on the
        #    Orin. Frames: cloud=unilidar_lidar, imu=unilidar_imu.
        # -----------------------------------------------------------------
        Node(
            package="unitree_lidar_ros2",      # PLACEHOLDER -- confirm on Orin
            executable="unitree_lidar_ros2_node",
            name="unilidar_driver",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
            # Remap here if the driver publishes under different names; the
            # Point-LIO config expects /unilidar/cloud and /unilidar/imu.
        ),

        # -----------------------------------------------------------------
        # 2) Point-LIO -> /Odometry (~250 Hz) + map cloud. Loads the L1 config.
        #    Fast path: Ericsii/FAST_LIO_ROS2 (native Humble). Clean target: a
        #    Point-LIO rclcpp port. PLACEHOLDER package/executable.
        # -----------------------------------------------------------------
        Node(
            package="point_lio",               # PLACEHOLDER -- fast_lio | point_lio rclcpp port
            executable="pointlio_mapping",     # PLACEHOLDER
            name="point_lio",
            output="screen",
            parameters=[
                lio_config,
                {
                    "use_sim_time": use_sim_time,
                    # OUTPUT model + IMU-rate propagation (per the launch params
                    # the reference repo sets outside the YAML).
                    "use_imu_as_input": 0,
                    "prop_at_freq_of_imu": 1,
                    "check_satu": 1,
                    "point_filter_num": 1,
                    "space_down_sample": True,
                    "filter_size_surf": 0.1,
                    "filter_size_map": 0.1,
                    "cube_side_length": 1000.0,
                    "init_map_size": 10,
                },
            ],
        ),

        # -----------------------------------------------------------------
        # 3) Static head-mount TF: base_link <- unilidar_lidar.
        #    NAMED args come straight from HeadMount (same code as the tests).
        #    In production prefer a URDF/xacro joint over this CLI node.
        # -----------------------------------------------------------------
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="head_mount_static_tf",
            output="screen",
            arguments=head_mount.to_static_transform_publisher_args(),
            parameters=[{"use_sim_time": use_sim_time}],
        ),

        # -----------------------------------------------------------------
        # 4) robot_localization LOCAL EKF: odom -> base_link (continuous).
        #    Fuses Point-LIO /Odometry + body IMU (roll/pitch + rates) +
        #    sport-mode odom (velocities).
        # -----------------------------------------------------------------
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_local",
            output="screen",
            parameters=[ekf_config, {"use_sim_time": use_sim_time}],
            # ekf_node reads the `ekf_local:` block by node name.
        ),

        # -----------------------------------------------------------------
        # 5) robot_localization GLOBAL EKF: map -> odom (fuses relocalization).
        # -----------------------------------------------------------------
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_global",
            output="screen",
            parameters=[ekf_config, {"use_sim_time": use_sim_time}],
            # ekf_node reads the `ekf_global:` block by node name.
        ),

        # -----------------------------------------------------------------
        # 6) Nav2 consumes the resulting map -> odom -> base_link TF chain plus
        #    the LIO point cloud / map. Bring Nav2 up from its own bringup
        #    launch (nav2_bringup) so its lifecycle/params stay separate.
        #    PLACEHOLDER: include nav2_bringup here at deploy time.
        # -----------------------------------------------------------------
    ])
