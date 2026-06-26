"""Golden numeric tests for the head-mount SE(3) transform.

Runs NOW with `uv run pytest -q` -- pure numpy, no ROS, no robot.
"""

import math

import numpy as np
import pytest

from arvit_robot_control.slam.extrinsics import (
    DEFAULT_CHILD_FRAME,
    DEFAULT_PARENT_FRAME,
    HeadMount,
    invert_se3,
    make_se3,
    rotation_matrix,
    transform_point,
)

TOL = 1e-12


def test_rx_pi_is_diag_1_neg1_neg1():
    """Rx(pi) = diag(1, -1, -1) -- the load-bearing upside-down flip."""
    R = rotation_matrix(roll=math.pi, pitch=0.0, yaw=0.0)
    expected = np.diag([1.0, -1.0, -1.0])
    assert np.allclose(R, expected, atol=1e-12)


@pytest.mark.parametrize("theta", [-0.7, -0.3, 0.0, 0.20, 0.30, 0.45, 1.0, 1.5707963])
def test_ry_theta_times_rx_pi_closed_form(theta):
    """Ry(theta) @ Rx(pi) equals the closed form for several theta.

        | cos t    0   -sin t |
    R = |   0     -1      0   |
        | -sin t   0   -cos t |

    This is the standard right-handed result of composing
    Ry(theta) = [[c,0,s],[0,1,0],[-s,0,c]] with Rx(pi) = diag(1,-1,-1):
    Rx(pi) negates column 2 of Ry, flipping the off-diagonal sin terms to
    -sin t. (The ARVIT note writes +sin t there, which is inconsistent with
    the right-handed Ry it also specifies -- see the extrinsics module note.)
    We assert the math directly against an INDEPENDENT Ry @ Rx product so the
    test does not merely re-derive rotation_matrix's own composition.
    """
    c, s = math.cos(theta), math.sin(theta)
    closed = np.array(
        [
            [c, 0.0, -s],
            [0.0, -1.0, 0.0],
            [-s, 0.0, -c],
        ]
    )
    R = rotation_matrix(roll=math.pi, pitch=theta, yaw=0.0)
    assert np.allclose(R, closed, atol=1e-12)

    # Cross-check: the closed form IS an independent Ry(theta) @ Rx(pi).
    ry = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    rx_pi = np.diag([1.0, -1.0, -1.0])
    assert np.allclose(closed, ry @ rx_pi, atol=1e-12)


@pytest.mark.parametrize(
    "roll,pitch,yaw",
    [
        (0.0, 0.0, 0.0),
        (math.pi, 0.30, 0.0),
        (math.pi, -0.45, 0.0),
        (0.1, 0.2, 0.3),
        (math.pi, 0.30, 0.7),
        (-1.2, 0.9, -2.0),
    ],
)
def test_rotation_is_orthonormal_with_det_plus_one(roll, pitch, yaw):
    """R is a proper rotation: R^T R = I and det(R) = +1."""
    R = rotation_matrix(roll, pitch, yaw)
    assert np.allclose(R.T @ R, np.eye(3), atol=1e-12)
    assert math.isclose(np.linalg.det(R), 1.0, abs_tol=1e-12)


def test_rotation_composition_order_is_yaw_outermost():
    """R == Rz(yaw) @ Ry(pitch) @ Rx(roll), the fixed-axis convention."""
    roll, pitch, yaw = 0.3, -0.4, 1.1

    def rx(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

    def ry(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    def rz(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    expected = rz(yaw) @ ry(pitch) @ rx(roll)
    assert np.allclose(rotation_matrix(roll, pitch, yaw), expected, atol=1e-12)


def test_make_se3_shape_and_bottom_row():
    """make_se3 is 4x4 with bottom row exactly [0, 0, 0, 1]."""
    T = make_se3(math.pi, 0.30, 0.0, 0.35, 0.0, 0.12)
    assert T.shape == (4, 4)
    assert np.allclose(T[3, :], np.array([0.0, 0.0, 0.0, 1.0]), atol=0.0)
    # Translation block is exactly the supplied (tx, ty, tz).
    assert np.allclose(T[:3, 3], np.array([0.35, 0.0, 0.12]), atol=0.0)


def test_transform_point_matches_homogeneous_application():
    """transform_point(T, p) == (T @ [p; 1])[:3]."""
    T = make_se3(math.pi, 0.30, 0.0, 0.35, 0.0, 0.12)
    p = np.array([1.0, 2.0, 3.0])
    homo = T @ np.array([1.0, 2.0, 3.0, 1.0])
    assert np.allclose(transform_point(T, p), homo[:3], atol=1e-12)


@pytest.mark.parametrize(
    "roll,pitch,yaw,tx,ty,tz",
    [
        (math.pi, 0.30, 0.0, 0.35, 0.0, 0.12),
        (0.1, 0.2, 0.3, -1.0, 2.5, 0.7),
        (math.pi, -0.45, 0.7, 0.30, 0.05, 0.15),
    ],
)
def test_invert_se3_round_trips(roll, pitch, yaw, tx, ty, tz):
    """invert_se3(T) @ T == I, and a point round-trips back to itself."""
    T = make_se3(roll, pitch, yaw, tx, ty, tz)
    Tinv = invert_se3(T)
    assert np.allclose(Tinv @ T, np.eye(4), atol=1e-12)
    assert np.allclose(T @ Tinv, np.eye(4), atol=1e-12)

    p = np.array([0.4, -1.3, 2.2])
    p_base = transform_point(T, p)
    p_back = transform_point(Tinv, p_base)
    assert np.allclose(p_back, p, atol=1e-12)


def test_invert_se3_is_rt_minus_rt_t():
    """Inverse uses the SE(3) closed form: [[R^T, -R^T t], [0, 1]]."""
    T = make_se3(math.pi, 0.30, 0.0, 0.35, 0.0, 0.12)
    R = T[:3, :3]
    t = T[:3, 3]
    Tinv = invert_se3(T)
    assert np.allclose(Tinv[:3, :3], R.T, atol=1e-12)
    assert np.allclose(Tinv[:3, 3], -R.T @ t, atol=1e-12)
    assert np.allclose(Tinv[3, :], np.array([0, 0, 0, 1]), atol=0.0)


def test_lidar_point_maps_sensibly_into_base_link():
    """A point in front of an upside-down, snout-tilted L1 lands forward of and
    below base_link.

    With roll=pi the lidar Y and Z are flipped; with pitch=theta the rotation's
    columns become x_lidar->(cos t, 0, -sin t), y_lidar->(0, -1, 0),
    z_lidar->(-sin t, 0, -cos t). A point straight ahead of the lidar
    (+x_lidar) maps to (2 cos t, 0, -2 sin t) + t: forward of the mount and
    DIPPING DOWN (negative z relative to the mount) -- consistent with a laser
    looking down-and-forward. We check the exact golden value plus directional
    sense.
    """
    pitch = 0.30
    hm = HeadMount(pitch=pitch, tx=0.35, ty=0.0, tz=0.12)

    # 2 m straight ahead of the lidar's own +x axis.
    p_lidar = np.array([2.0, 0.0, 0.0])
    p_base = hm.transform_point(p_lidar)

    # Closed form: R @ [2,0,0] = [2 cos t, 0, -2 sin t]; + t.
    expected = np.array(
        [2.0 * math.cos(pitch) + 0.35, 0.0, -2.0 * math.sin(pitch) + 0.12]
    )
    assert np.allclose(p_base, expected, atol=1e-12)

    # Directional sanity: forward of the mount, on the snout center, dips down.
    assert p_base[0] > hm.tx           # forward
    assert math.isclose(p_base[1], hm.ty, abs_tol=1e-12)
    assert (p_base[2] - hm.tz) < 0.0   # below the mount (down-and-forward look)

    # The lidar's own +y maps to body -y (right) because of the roll=pi flip.
    p_y = hm.transform_point(np.array([0.0, 1.0, 0.0]))
    assert p_y[1] < hm.ty  # flipped to the right

    # The lidar's own +z maps to body (-sin t, 0, -cos t) relative to t:
    # backward and down for a positive pitch.
    p_z = hm.transform_point(np.array([0.0, 0.0, 1.0]))
    assert (p_z[0] - hm.tx) < 0.0
    assert (p_z[2] - hm.tz) < 0.0


def test_headmount_defaults():
    """Default mount: roll=pi (upside-down), yaw=0, frames per Unitree driver."""
    hm = HeadMount()
    assert math.isclose(hm.roll, math.pi, abs_tol=1e-12)
    assert math.isclose(hm.yaw, 0.0, abs_tol=1e-12)
    assert hm.parent_frame == DEFAULT_PARENT_FRAME == "base_link"
    assert hm.child_frame == DEFAULT_CHILD_FRAME == "unilidar_lidar"
    # The cached matrix matches a fresh make_se3 with the same params.
    assert np.allclose(
        hm.matrix,
        make_se3(hm.roll, hm.pitch, hm.yaw, hm.tx, hm.ty, hm.tz),
        atol=0.0,
    )


def test_headmount_matrix_is_read_only():
    """The cached matrix is immutable so callers can't corrupt the transform."""
    hm = HeadMount()
    with pytest.raises(ValueError):
        hm.matrix[0, 0] = 99.0


def test_headmount_inverse_round_trips():
    """HeadMount.inverse @ matrix == I."""
    hm = HeadMount(pitch=0.30, tx=0.35, tz=0.12)
    assert np.allclose(hm.inverse @ hm.matrix, np.eye(4), atol=1e-12)


def test_static_transform_publisher_args_named_and_ordered():
    """to_static_transform_publisher_args yields the NAMED arg list in order."""
    hm = HeadMount(pitch=0.30, tx=0.35, ty=0.0, tz=0.12)
    args = hm.to_static_transform_publisher_args()

    # All the named flags are present.
    for flag in ("--x", "--y", "--z", "--roll", "--pitch", "--yaw",
                 "--frame-id", "--child-frame-id"):
        assert flag in args

    # Parent is base_link, child is unilidar_lidar (footgun: order matters).
    assert args[args.index("--frame-id") + 1] == "base_link"
    assert args[args.index("--child-frame-id") + 1] == "unilidar_lidar"

    # Numeric values round-trip to the mount params.
    assert float(args[args.index("--roll") + 1]) == pytest.approx(math.pi)
    assert float(args[args.index("--pitch") + 1]) == pytest.approx(0.30)
    assert float(args[args.index("--x") + 1]) == pytest.approx(0.35)
    assert float(args[args.index("--z") + 1]) == pytest.approx(0.12)

    # The list is flat strings, ready to splat onto the ros2 run command line.
    assert all(isinstance(a, str) for a in args)
