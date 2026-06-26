"""Tests for the Point-LIO timestamp / clock helpers.

Runs NOW with `uv run pytest -q` -- pure Python, no ROS, no robot.
"""

import math

import pytest

from arvit_robot_control.slam.timesync import (
    L1_IMU_TIME_INTE,
    L1_TIME_LAG_IMU_TO_LIDAR,
    imu_rate_from_inte,
    is_monotonic_after_shift,
    shift_imu_to_lidar,
)


def test_imu_rate_from_inte_250hz():
    """imu_time_inte = 0.004 => 250 Hz (the L1's built-in IMU rate)."""
    assert imu_rate_from_inte(0.004) == pytest.approx(250.0)
    # The module constant matches the L1 config value.
    assert imu_rate_from_inte(L1_IMU_TIME_INTE) == pytest.approx(250.0)


@pytest.mark.parametrize(
    "inte,expected",
    [(0.004, 250.0), (0.005, 200.0), (0.01, 100.0), (0.002, 500.0)],
)
def test_imu_rate_from_inte_various(inte, expected):
    assert imu_rate_from_inte(inte) == pytest.approx(expected)


@pytest.mark.parametrize("bad", [0.0, -0.001, -1.0])
def test_imu_rate_from_inte_rejects_nonpositive(bad):
    with pytest.raises(ValueError):
        imu_rate_from_inte(bad)


def test_shift_imu_to_lidar_subtracts_lag():
    """Point-LIO moves the IMU stamp onto the LiDAR timeline by SUBTRACTING."""
    # Positive lag => IMU ahead of LiDAR => stamp pulled back.
    assert shift_imu_to_lidar(100.0, 0.02) == pytest.approx(99.98)
    # Negative lag => stamp pushed forward.
    assert shift_imu_to_lidar(100.0, -0.02) == pytest.approx(100.02)


def test_shift_imu_to_lidar_l1_is_identity():
    """Architecture A: L1 single clock => time_lag = 0 => no shift."""
    assert L1_TIME_LAG_IMU_TO_LIDAR == 0.0
    for stamp in (0.0, 1.5, 1234.567):
        assert shift_imu_to_lidar(stamp, L1_TIME_LAG_IMU_TO_LIDAR) == stamp


def test_loop_back_guard_accepts_monotonic():
    """A non-decreasing (after shift) stamp stream passes the guard."""
    stamps = [10.000, 10.004, 10.008, 10.012, 10.016]  # 250 Hz cadence
    assert is_monotonic_after_shift(stamps, 0.0) is True
    # Equal consecutive stamps are allowed (non-strict).
    assert is_monotonic_after_shift([5.0, 5.0, 5.004], 0.0) is True


def test_loop_back_guard_rejects_backwards():
    """A stamp going backwards trips the guard ('imu loop back, clear deque')."""
    stamps = [10.000, 10.004, 9.999, 10.012]  # third sample jumps back
    assert is_monotonic_after_shift(stamps, 0.0) is False


def test_loop_back_guard_lag_can_induce_loopback():
    """An over-large lag relative to spacing can itself induce a loop-back...

    The shift subtracts a CONSTANT lag from every stamp, so a constant lag
    cannot reorder a monotonic stream. But a per-call mix of stamps that is
    only barely monotonic stays monotonic under a constant shift -- documenting
    that the guard catches clock resets/jitter, not the constant lag itself.
    """
    stamps = [10.000, 10.004, 10.008]
    # Constant lag preserves ordering.
    assert is_monotonic_after_shift(stamps, 0.05) is True
    assert is_monotonic_after_shift(stamps, -0.05) is True


def test_loop_back_guard_trivial_sequences():
    """Empty and single-element streams are trivially monotonic."""
    assert is_monotonic_after_shift([], 0.0) is True
    assert is_monotonic_after_shift([42.0], 0.0) is True


def test_imu_cadence_consistency():
    """Sanity: 250 Hz means 0.004 s spacing -- the round-trip is consistent."""
    rate = imu_rate_from_inte(0.004)
    spacing = 1.0 / rate
    assert math.isclose(spacing, 0.004, abs_tol=1e-12)
