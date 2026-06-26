"""Head-mount SE(3) transform for ARVIT's upside-down, tilted-forward L1.

This module implements **transform #2** of the ARVIT LIO architecture: the
single static rigid-body transform ``base_link <- unilidar_lidar`` that
re-expresses the L1 point cloud and Point-LIO odometry in the robot body
frame for Nav2 / robot_localization.

It is *not* the internal IMU->LiDAR extrinsic (transform #1), which is the
factory near-identity pair already baked into ``config/unilidar_l1.yaml`` and
consumed inside Point-LIO. Under Architecture A the LIO runs on the L1's
built-in IMU (one rigid housing, one clock, factory extrinsic), so there is no
time-sync and no extrinsic to solve *inside* the LIO -- only this one mount
transform on the outside.

Conventions (all per REP-103, https://www.ros.org/reps/rep-0103.html):

- Right-handed frames; body frame is **x forward, y left, z up**.
- Units: **metres and radians**.
- Fixed-axis (extrinsic) roll/pitch/yaw about X/Y/Z, composed yaw-outermost::

      R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

The two physical facts that build the mount rotation:

- **UPSIDE-DOWN  => roll = pi.**  ``Rx(pi) = diag(1, -1, -1)`` flips Y and Z.
  This is the load-bearing term: drop it and the whole map comes out inverted.
- **TILTED FORWARD ("snout") => pitch = theta** about Y, so the laser looks
  down-and-forward.
- **yaw = 0** as typically mounted (set the real azimuth if the bracket is
  rotated).

Carrying out ``Ry(theta) @ Rx(pi)`` (with yaw = 0) with the standard
right-handed ``Ry``/``Rx`` above gives the closed form::

      | cos t    0   -sin t |
  R = |   0     -1      0   |
      | -sin t   0   -cos t |

.. note::
   The ARVIT design note (``lidar-imu-extrinsics.md``) writes this matrix with
   ``+sin t`` in the (0,2) and (2,0) off-diagonals. That is **inconsistent**
   with the standard right-handed ``Ry(theta) = [[c,0,s],[0,1,0],[-s,0,c]]``
   the same note (and REP-103) specifies: composing it with
   ``Rx(pi) = diag(1,-1,-1)`` necessarily flips the sign of those two entries
   (column 2 of ``Ry`` is negated). The mathematically self-consistent form
   shown above is what this module produces and what the tests assert. The note
   itself flags the pitch *sign* as ``(unverified)`` -- this off-diagonal sign
   is the same caveat, resolved here in favour of the standard right-handed
   convention. Either way the empirical RViz check (flat floor flat, walls
   vertical) is the final arbiter of the mount's true ``pitch``.

.. warning::
   **Sign of pitch is unverified until validated empirically.** With this
   fixed-axis convention a *positive* pitch rotates the sensor's +x toward +z
   (up); tilting the look-direction *down* by theta may instead require
   *-theta* on the real mount. Validate by transforming a frame's cloud into
   ``base_link`` and checking, in RViz, that a flat floor stays flat and walls
   stay vertical. If the floor tilts, the pitch sign/magnitude is wrong; if the
   scene is inverted, the ``roll = pi`` flip is missing or doubled.

Pure numpy; no ROS, no robot, no network. Safe to import and unit-test
anywhere.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# Default child frame the Unitree ROS 2 driver stamps on its PointCloud2 / Imu.
DEFAULT_PARENT_FRAME = "base_link"
DEFAULT_CHILD_FRAME = "unilidar_lidar"


def _rx(roll: float) -> np.ndarray:
    """Rotation about the X axis (roll), radians, right-handed."""
    c, s = math.cos(roll), math.sin(roll)
    return np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c],
        ],
        dtype=float,
    )


def _ry(pitch: float) -> np.ndarray:
    """Rotation about the Y axis (pitch), radians, right-handed."""
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array(
        [
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c],
        ],
        dtype=float,
    )


def _rz(yaw: float) -> np.ndarray:
    """Rotation about the Z axis (yaw), radians, right-handed."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Build a 3x3 rotation matrix from fixed-axis RPY (radians).

    Composition is yaw-outermost: ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``,
    the convention REP-103 / the ROS ``static_transform_publisher`` use.

    Returns a proper rotation in SO(3) (orthonormal, det = +1).
    """
    return _rz(yaw) @ _ry(pitch) @ _rx(roll)


def make_se3(
    roll: float,
    pitch: float,
    yaw: float,
    tx: float,
    ty: float,
    tz: float,
) -> np.ndarray:
    """Assemble the 4x4 homogeneous transform ``T_base_lidar``.

    The rotation is :func:`rotation_matrix`; the translation ``(tx, ty, tz)``
    is the L1 origin's offset from ``base_link``, expressed in ``base_link``
    axes (metres). A point ``p_lidar`` maps to the body frame as
    ``p_base = R @ p_lidar + t`` (see :func:`transform_point`).

    The bottom row is exactly ``[0, 0, 0, 1]``.
    """
    T = np.eye(4, dtype=float)
    T[:3, :3] = rotation_matrix(roll, pitch, yaw)
    T[:3, 3] = (tx, ty, tz)
    return T


def transform_point(T: np.ndarray, p: np.ndarray) -> np.ndarray:
    """Apply a 4x4 SE(3) transform to a 3-vector point: ``R @ p + t``.

    ``T`` is 4x4, ``p`` is length-3. Returns a length-3 numpy array. This is
    the affine action ``T @ [p; 1]`` without forcing the caller to build the
    homogeneous vector.
    """
    T = np.asarray(T, dtype=float)
    p = np.asarray(p, dtype=float).reshape(3)
    return T[:3, :3] @ p + T[:3, 3]


def invert_se3(T: np.ndarray) -> np.ndarray:
    """Closed-form inverse of a 4x4 SE(3) transform.

    For ``T = [[R, t], [0, 1]]`` the inverse is ``[[R^T, -R^T t], [0, 1]]``,
    which avoids a general 4x4 matrix inversion and stays numerically exact for
    a proper rotation. Inverting ``T_base_lidar`` yields ``T_lidar_base``.
    """
    T = np.asarray(T, dtype=float)
    R = T[:3, :3]
    t = T[:3, 3]
    Tinv = np.eye(4, dtype=float)
    Tinv[:3, :3] = R.T
    Tinv[:3, 3] = -R.T @ t
    return Tinv


@dataclass(frozen=True)
class HeadMount:
    """The ARVIT L1 head-mount transform ``base_link <- unilidar_lidar``.

    Defaults encode the two structural facts of the ARVIT mount:

    - ``roll = pi`` (upside-down) -- the load-bearing flip. Keep this unless
      the physical mounting changes.
    - ``pitch`` -- the forward snout-tilt angle (radians). **Placeholder
      default; measure / CAD it on the real robot.** Sign is unverified (see
      the module docstring warning).
    - ``yaw = 0`` -- unless the bracket is rotated in azimuth.
    - ``tx, ty, tz`` -- L1 origin offset from ``base_link`` in body axes,
      metres. **Placeholders; measure them.**

    The frame names default to the Unitree ROS 2 driver's published frame
    (``unilidar_lidar``) and the robot body frame (``base_link``).

    Example
    -------
    >>> hm = HeadMount(pitch=0.30, tx=0.35, tz=0.12)
    >>> T = hm.matrix
    >>> T.shape
    (4, 4)
    """

    roll: float = math.pi
    pitch: float = 0.30  # (unverified) ~17 deg snout tilt -- MEASURE on robot
    yaw: float = 0.0
    tx: float = 0.35  # (unverified) head ahead of body center, metres
    ty: float = 0.0  # (unverified) centred on the snout
    tz: float = 0.12  # (unverified) head above body center, metres
    parent_frame: str = DEFAULT_PARENT_FRAME
    child_frame: str = DEFAULT_CHILD_FRAME

    # Cached 4x4 (the dataclass is frozen, so this is computed once on access
    # via __post_init__ + object.__setattr__).
    _matrix: np.ndarray = field(default=None, repr=False, compare=False)  # type: ignore[assignment]

    def __post_init__(self) -> None:
        T = make_se3(self.roll, self.pitch, self.yaw, self.tx, self.ty, self.tz)
        # Bypass frozen to cache the immutable result; make it read-only.
        T.setflags(write=False)
        object.__setattr__(self, "_matrix", T)

    @property
    def matrix(self) -> np.ndarray:
        """The 4x4 ``T_base_lidar`` for this mount (read-only)."""
        return self._matrix

    @property
    def inverse(self) -> np.ndarray:
        """The 4x4 ``T_lidar_base`` (inverse of :attr:`matrix`)."""
        return invert_se3(self._matrix)

    def transform_point(self, p: np.ndarray) -> np.ndarray:
        """Map a point from the lidar frame into ``base_link``."""
        return transform_point(self._matrix, p)

    def to_static_transform_publisher_args(self) -> list[str]:
        """Return the ROS 2 ``static_transform_publisher`` NAMED-arg list.

        Use the **named** form -- the legacy positional Euler order is
        ``x y z yaw pitch roll`` (yaw first), a well-known footgun. ``--roll
        --pitch --yaw`` are radians, ``--x --y --z`` metres. ``--frame-id`` is
        the PARENT (``base_link``); ``--child-frame-id`` is the CHILD
        (``unilidar_lidar``).

        The returned list is ready to splat after
        ``ros2 run tf2_ros static_transform_publisher``::

            ros2 run tf2_ros static_transform_publisher \\
                $(python -c "...; print(' '.join(HeadMount().to_static_transform_publisher_args()))")
        """
        return [
            "--x", repr(float(self.tx)),
            "--y", repr(float(self.ty)),
            "--z", repr(float(self.tz)),
            "--roll", repr(float(self.roll)),
            "--pitch", repr(float(self.pitch)),
            "--yaw", repr(float(self.yaw)),
            "--frame-id", self.parent_frame,
            "--child-frame-id", self.child_frame,
        ]
