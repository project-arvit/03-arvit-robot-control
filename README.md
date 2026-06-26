# arvit-robot-control

The **LiDAR-inertial-odometry (LIO) integration** layer for ARVIT, the
autonomous inspection system on a Unitree Go2 EDU quadruped ("robodog") with a
Jetson Orin compute module. This package wires the **head-mounted Unitree L1 4D
LiDAR** into a clean robot-body pose estimate for Nav2.

The L1 sits **upside-down and tilted forward** on the dog's snout. This package
owns the spatial + temporal glue that makes its data usable.

## Decided architecture (Architecture A)

Run **Point-LIO on the L1's BUILT-IN IMU**. The laser and the IMU share one
rigid housing, one device clock, and the factory extrinsic, so **inside the LIO
there is no time-sync and no extrinsic problem** (identity rotation, ~1.3 cm
translation, `time_lag_imu_to_lidar = 0`).

The head mount is handled as **one static SE(3) transform**
`base_link <- unilidar_lidar` -- "the homogeneous transformation". The **body
IMU + sport-mode odometry** join LATER at a `robot_localization` two-EKF layer,
**not inside the LIO**.

```
L1 (cloud @ ~11 Hz + IMU @ 250 Hz, one clock)
        |
   Point-LIO  -- config/unilidar_l1.yaml -->  /Odometry (~250 Hz)
        |
   static TF  base_link <- unilidar_lidar   (HeadMount, roll=pi + pitch + t)
        |
   robot_localization:  ekf_local (odom->base_link)
                        ekf_global (map->odom, fuses AprilTag relocalization)
        |
      Nav2  (consumes map -> odom -> base_link)
```

## Layout

| Path | What it is |
|------|-----------|
| `arvit_robot_control/slam/extrinsics.py` | **CORE** pure-numpy SE(3) head-mount transform. `rotation_matrix`, `make_se3`, `transform_point`, `invert_se3`, and the `HeadMount` dataclass (`.matrix`, `.to_static_transform_publisher_args()`). |
| `arvit_robot_control/slam/timesync.py` | Point-LIO timestamp helpers: `imu_rate_from_inte`, `shift_imu_to_lidar`, `is_monotonic_after_shift` (loop-back guard). |
| `config/unilidar_l1.yaml` | Point-LIO L1 config (Architecture A), per-key comments. |
| `config/ekf_two_ekf.yaml` | `robot_localization` two-EKF (local `odom->base_link`, global `map->odom`). |
| `launch/slam_bringup.launch.py` | ROS 2 Humble bring-up: L1 driver -> Point-LIO -> static TF -> two EKFs -> Nav2. Deploy-time. |
| `tests/` | `test_extrinsics.py`, `test_timesync.py` -- run NOW, pure logic. |

## The head-mount transform (the load-bearing math)

REP-103 body frame (x forward, y left, z up), radians/metres, fixed-axis RPY
`R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`:

- **UPSIDE-DOWN => roll = pi.** `Rx(pi) = diag(1, -1, -1)` flips Y and Z. This
  is the load-bearing term.
- **TILTED FORWARD => pitch = theta** about Y.
- **yaw = 0** as typically mounted.

Closed form (yaw = 0):

```
               | cos t    0   -sin t |
Ry(t) Rx(pi) = |   0     -1      0   |
               | -sin t   0   -cos t |
```

> **Off-diagonal sign:** the ARVIT design note writes `+sin t` in the (0,2) /
> (2,0) entries. That is inconsistent with the standard right-handed
> `Ry(t) = [[c,0,s],[0,1,0],[-s,0,c]]` it also specifies -- composing it with
> `Rx(pi) = diag(1,-1,-1)` negates column 2 of `Ry`, so those entries become
> `-sin t`. This package uses the mathematically self-consistent form above
> (and the tests assert it against an independent `Ry @ Rx` product). The
> note's own `(unverified)` pitch-sign caveat covers this; the RViz check is
> the final arbiter.

> **Pitch sign caveat:** with this fixed-axis convention a *positive* pitch
> rotates the sensor's +x toward +z (up); tilting the look-direction *down* may
> need *-theta* on the real mount. **Validate empirically** -- in RViz a flat
> floor must stay flat and walls must stay vertical. If the floor tilts, the
> pitch sign/magnitude is wrong; if the scene is inverted, the `roll = pi` flip
> is missing or doubled. The `pitch`/`tx`/`ty`/`tz` defaults are placeholders;
> **measure them on the robot or pull them from CAD.** Only `roll = pi` is
> structural.

`HeadMount.to_static_transform_publisher_args()` emits the ROS 2 **named** arg
list (`--x --y --z --roll --pitch --yaw --frame-id base_link
--child-frame-id unilidar_lidar`) -- the named form avoids the legacy
positional `x y z yaw pitch roll` footgun.

## Run the tests (NOW, on this machine -- no robot, no ROS)

The pure-logic core depends only on numpy and runs anywhere:

```bash
cd arvit-robot-control
uv venv --python 3.11
uv pip install numpy pytest
uv run pytest -q
```

`launch/slam_bringup.launch.py` is **deploy-target code** -- it imports
`launch` / `launch_ros`, which exist only on the robot, so the local suite does
not import it. The runnable tests cover the SE(3) transform and the timestamp
helpers.

## Run via Docker

Built from the ROS 2 Humble base; the default command runs the same pure-logic
pytest suite:

```bash
cd arvit-robot-control
cp .env.example .env            # parity; no secrets needed for tests
docker compose run --rm robot-control
# or, without compose:
docker build -t arvit-robot-control:latest .
docker run --rm arvit-robot-control:latest pytest -q
```

> The Humble base ships arm64 (Orin) + amd64. On Apple silicon it pulls arm64
> natively. On x86 add `--platform linux/arm64` for Orin parity, or build amd64
> locally -- the pure-logic tests are arch-independent.

## Run via Apple `container`

Same Dockerfile, single-container (no compose):

```bash
cd arvit-robot-control
cp .env.example .env
scripts/container-up.sh                 # build + run the test suite
scripts/container-down.sh               # stop + remove container and image
```

Pass args to override the default command, e.g.:

```bash
scripts/container-up.sh python -c \
  "from arvit_robot_control.slam.extrinsics import HeadMount; \
   print(' '.join(HeadMount().to_static_transform_publisher_args()))"
```

## What deploys to the Jetson Orin

- `config/unilidar_l1.yaml`, `config/ekf_two_ekf.yaml` -- loaded by the nodes.
- `launch/slam_bringup.launch.py` -- the ROS 2 Humble bring-up.
- `arvit_robot_control.slam.extrinsics` -- imported by the launch file so the
  static-TF args come from the SAME code the tests verify (no copied magic
  numbers).
- The Docker image (Humble + `robot_localization` + Nav2 + this package).

## What needs hardware / keys (NOT exercised locally)

- The **Unitree L1** and its ROS 2 driver (`/unilidar/cloud`, `/unilidar/imu`).
  The driver is Foxy-verified; a clean Humble build is **(unverified)**.
- A **Point-LIO node**. No first-party Humble port exists. Fast path:
  `Ericsii/FAST_LIO_ROS2` (native Humble). Clean target: a Point-LIO rclcpp
  port. The launch file marks these as deploy-time `package`/`executable`
  placeholders.
- The **body IMU** (`/imu/body`) and **sport-mode odom** (`/sportmode/odom`,
  from `rt/sportmodestate`) for the EKF layer.
- An **AprilTag / relocalization** pose source (`/relocalization/pose`) for the
  global EKF.
- No cloud keys are needed anywhere in this package.

## Conventions

Poses follow REP-103 (x fwd / y left / z up), radians and metres. Secrets, if
ever added, go only via `--env-file` / `.env` (gitignored; commit
`.env.example` only). `.pcd` map outputs are gitignored.
