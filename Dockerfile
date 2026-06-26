# ARVIT robot-control: the LiDAR-inertial-odometry (LIO) integration package.
#
# Targets the Jetson Orin / Ubuntu 22.04 (ROS 2 Humble, arm64) at DEPLOY time.
# The image carries robot_localization + Nav2 (the deploy deps) AND the
# pure-logic Python core (extrinsics, timesync). It DEFAULTS to running the
# pytest suite, which exercises ONLY the pure-logic core -- no ROS, no robot,
# no network -- so the same image both deploys and self-tests.
#
# The SAME Dockerfile drives BOTH paths:
#   * Docker:         docker compose run --rm robot-control   (or docker build/run)
#   * Apple container: scripts/container-up.sh                (single build + run)
#
# NOTE on architecture: ros:humble-ros-base ships arm64 (Orin) and amd64. On an
# Apple-silicon Mac it pulls arm64 natively; on x86 hosts add
# `--platform linux/arm64` for Orin parity (or build amd64 for local convenience
# -- the pure-logic tests are arch-independent).
FROM ros:humble-ros-base

# uv for fast, reproducible Python installs (parity with the local uv workflow).
COPY --from=ghcr.io/astral-sh/uv:0.11 /uv /usr/local/bin/uv

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    DEBIAN_FRONTEND=noninteractive \
    # The ROS Humble base ships the `launch_testing` / `launch_testing_ros`
    # pytest entrypoints, which are incompatible with the pytest uv installs and
    # crash collection with a PluginValidationError. This suite is pure unit
    # tests needing no third-party pytest plugin, so disable plugin autoload.
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1

# ---- deploy-target ROS 2 deps (the fusion + navigation stack) ----
# robot_localization runs the two EKFs; nav2 consumes the TF chain. These are
# the real Orin dependencies; they are not exercised by the local pytest run.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-robot-localization \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-tf2-ros \
        python3-numpy \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Make the system Python the uv target (Humble uses the system interpreter).
ENV UV_SYSTEM_PYTHON=1

# Test deps. numpy is already present from apt (python3-numpy) but we let uv
# ensure the floor; pytest is the test runner.
RUN uv pip install --system --break-system-packages "numpy>=1.24" "pytest>=8.0"

# Copy the package, configs, launch, and tests.
COPY arvit_robot_control/ ./arvit_robot_control/
COPY config/ ./config/
COPY launch/ ./launch/
COPY tests/ ./tests/
COPY pyproject.toml README.md ./

# Make the source tree importable (no editable install needed).
ENV PYTHONPATH=/app

# Default: run the pure-logic test suite. Override to launch on the robot, e.g.
#   docker run --rm arvit-robot-control \
#     ros2 launch arvit_robot_control slam_bringup.launch.py
# (launch needs the real robot + L1 + drivers; it will not run in this image
# without hardware.)
CMD ["pytest", "-q"]
