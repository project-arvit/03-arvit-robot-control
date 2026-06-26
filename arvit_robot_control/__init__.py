"""ARVIT robot-control package.

The SLAM / LiDAR-inertial-odometry (LIO) integration layer for the
head-mounted Unitree L1 4D LiDAR on the Go2 EDU inspection dog.

The pure-logic core (``arvit_robot_control.slam``) has NO ROS / robot
dependencies and is unit-tested on any machine. The launch/config layer
targets ROS 2 Humble on the Jetson Orin at deploy time.
"""

__version__ = "0.1.0"
