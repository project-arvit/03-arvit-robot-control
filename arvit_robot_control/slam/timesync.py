"""Point-LIO timestamp / clock helpers (the TIME half of LIO fusion).

The single most important fact: **Point-LIO does not need the LiDAR and the
IMU to share a sampling rate -- it needs correct per-sample timestamps on a
common clock.** The L1's ~11 Hz LiDAR cadence vs its 250 Hz IMU cadence is
handled natively by Point-LIO's point-wise, continuous-time IKFoM propagation;
the thing that actually matters is clock alignment.

Under Architecture A (LIO on the L1's built-in IMU) the cloud and the IMU come
off the *same* SDK device clock, so ``time_lag_imu_to_lidar = 0.0`` and there
is nothing to align. These helpers document and verify the few numeric facts
that still hold:

- :func:`imu_rate_from_inte` -- the IMU rate the config implies
  (``1 / imu_time_inte``); ``imu_time_inte = 0.004 -> 250.0 Hz``.
- :func:`shift_imu_to_lidar` -- how Point-LIO moves an IMU stamp onto the
  LiDAR timeline: it **subtracts** ``time_lag_imu_to_lidar``.
- :func:`is_monotonic_after_shift` -- the loop-back guard: Point-LIO's IMU
  callback clears its deque if a (shifted) stamp goes backwards ("imu loop
  back, clear deque").

Pure Python; no ROS, no robot, no network.
"""

from __future__ import annotations

from collections.abc import Sequence

# Architecture A: the L1 SDK stamps cloud + IMU on one device timeline.
L1_TIME_LAG_IMU_TO_LIDAR = 0.0
# config/unilidar_l1.yaml -> mapping.imu_time_inte
L1_IMU_TIME_INTE = 0.004


def imu_rate_from_inte(imu_time_inte: float) -> float:
    """IMU rate (Hz) implied by Point-LIO's ``imu_time_inte`` (seconds).

    ``imu_time_inte`` is ``1 / IMU_freq`` and doubles as the
    covariance-propagation dt threshold. For the L1 config
    ``imu_time_inte = 0.004`` => ``250.0`` Hz.

    Raises ``ValueError`` for a non-positive step (an undefined rate).
    """
    if imu_time_inte <= 0.0:
        raise ValueError(f"imu_time_inte must be > 0, got {imu_time_inte!r}")
    return 1.0 / imu_time_inte


def shift_imu_to_lidar(imu_stamp: float, time_lag: float) -> float:
    """Move an IMU stamp onto the LiDAR timeline.

    Point-LIO's IMU callback computes
    ``imu_stamp_on_lidar_timeline = imu_stamp - time_lag``.

    A *positive* ``time_lag`` means the IMU runs *ahead* of the LiDAR clock, so
    its stamps are pulled back. For the L1 (Architecture A) ``time_lag = 0.0``
    and this is the identity.
    """
    return imu_stamp - time_lag


def is_monotonic_after_shift(
    imu_stamps: Sequence[float],
    time_lag: float,
) -> bool:
    """Loop-back guard: do the shifted IMU stamps stay non-decreasing?

    Mirrors Point-LIO's IMU-callback check that rejects a sample whose shifted
    stamp goes backwards (logged as "imu loop back, clear deque", caused by a
    clock reset, jitter, or an over-large lag). Returns ``True`` if every
    consecutive shifted stamp is ``>=`` the previous one.

    An empty or single-element sequence is trivially monotonic. Equal
    consecutive stamps are allowed (non-strict) -- only a strict *decrease* is
    a loop-back.
    """
    shifted = [shift_imu_to_lidar(s, time_lag) for s in imu_stamps]
    return all(b >= a for a, b in zip(shifted, shifted[1:]))
