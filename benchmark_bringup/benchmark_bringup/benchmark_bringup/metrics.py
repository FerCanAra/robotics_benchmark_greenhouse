# +-------------------------------------------------------------------------+
# |                       Benchmark control simulator                       |
# |                                                                         |
# | Copyright (C) 2025  Fernando Cañadas Aránega                            |
# | PhD Student University of Almería, Spain                                |
# | Contact: fernando.ca@ual.es                                             |
# | Distributed under 3-clause BSD License                                  |
# | See COPYING                                                             |
# | Performance indices of the benchmark (shared by plotters and the        |
# | offline analysis). Pure numpy: no ROS dependencies.                     |
# +-------------------------------------------------------------------------+
"""Performance indices of the benchmark.

All the indices are time averages of normalised, dimensionless signals over the
evaluation window [t0, t1], so they do not depend on the sampling frequency or
on the duration of the trial:

    SAE = 1/T * int_0^T e_n(t) dt ,   SCI = 1/T * int_0^T u_n(t) dt

Category 1 (low level), per-wheel signals:
    e_n = (|e_R| + |e_L|) / (2 v_w,max)        (wheel linear velocity error)
    u_n = (|tau_R| + |tau_L|) / (2 tau_max)    (motor torque), u_n in [0, 1]

Category 2 (mid level), signals resampled to a common time base:
    e_n = d(p(t), Gamma_ref) / d_ref           (distance to the common reference path:
                                                start pose + waypoints, the polyline sent
                                                to every controller through FollowPath)
    u_n = sqrt((v/v_max)^2 + (w/w_max)^2) / sqrt(2)

Category 3 (high level):
    e_n = d(p(t), plan(t)) / d_ref             (distance to the current planner path)
    plus path-quality indicators (length ratio, clearance, contacts, success, time)

Each layer index is J = (SAE + SCI) / 2 (J3 = SAE3), and the total indices are
J_T2 = (J1 + J2) / 2 and J_T3 = (J1 + J2 + J3) / 3.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

# numpy >= 2.0 renamed trapz -> trapezoid
_trapz = getattr(np, "trapezoid", None) or getattr(np, "trapz")


# ---------------------------------------------------------------------------
#                          Normalisation constants
# ---------------------------------------------------------------------------
@dataclass
class MetricsConfig:
    v_wheel_max: float = 1.0   # [m/s] maximum wheel linear velocity of the platform
    tau_max: float = 500.0     # [N m] torque saturation (overwritten by the logged value)
    v_max: float = 0.5         # [m/s] max linear velocity of the mid-level controller
    w_max: float = 1.0         # [rad/s] max angular velocity of the mid-level controller (launch arg w_max)
    d_ref: float = 2.0         # [m] fixed distance scale (half-width of the greenhouse corridor)
    fs: float = 10.0           # [Hz] common time base for the mid/high-level indices
    horizon: float = 1.0          # [s] prediction horizon of the mid-level error (SAE2)
    lidar_min_range: float = 0.5  # [m] LiDAR returns closer than this are discarded (the sensor
                                  # is inside the robot: as on the real robot / Nav2 obstacle_min_range)
    goal_tolerance: float = 0.3  # [m] goal tolerance (Nav2 goal checker)

    def as_dict(self) -> Dict[str, float]:
        return asdict(self)


# ---------------------------------------------------------------------------
#                                 Helpers
# ---------------------------------------------------------------------------
def _as_array(x) -> np.ndarray:
    return np.asarray(x, dtype=float)


def time_average(t: Sequence[float], y: Sequence[float]) -> float:
    """Time-weighted average (trapezoidal rule) of y(t). Works with non-uniform sampling."""
    t = _as_array(t)
    y = _as_array(y)
    if t.size < 2:
        return float(np.mean(y)) if y.size else float("nan")
    T = t[-1] - t[0]
    if T <= 0:
        return float(np.mean(y))
    return float(_trapz(y, t) / T)


def window_mask(t: Sequence[float], t0: Optional[float], t1: Optional[float]) -> np.ndarray:
    t = _as_array(t)
    m = np.ones_like(t, dtype=bool)
    if t0 is not None:
        m &= t >= t0
    if t1 is not None:
        m &= t <= t1
    return m


def time_grid(t0: float, t1: float, fs: float) -> np.ndarray:
    if t1 <= t0:
        return np.array([t0])
    n = int(math.floor((t1 - t0) * fs)) + 1
    return t0 + np.arange(n) / fs


def resample(t_src: Sequence[float], y_src: Sequence[float], t_grid: np.ndarray) -> np.ndarray:
    """Linear interpolation of y_src(t_src) onto t_grid (values held at the edges)."""
    t_src = _as_array(t_src)
    y_src = _as_array(y_src)
    if t_src.size == 0:
        return np.full_like(t_grid, np.nan, dtype=float)
    order = np.argsort(t_src)
    return np.interp(t_grid, t_src[order], y_src[order])


def resample_hold(t_src: Sequence[float], y_src: Sequence[float], t_grid: np.ndarray) -> np.ndarray:
    """Zero-order hold resampling (for commands, which are piecewise constant)."""
    t_src = _as_array(t_src)
    y_src = _as_array(y_src)
    if t_src.size == 0:
        return np.zeros_like(t_grid, dtype=float)
    order = np.argsort(t_src)
    t_src, y_src = t_src[order], y_src[order]
    idx = np.searchsorted(t_src, t_grid, side="right") - 1
    out = np.where(idx >= 0, y_src[np.clip(idx, 0, None)], 0.0)
    return out


def dist_points_polyline(px: Sequence[float], py: Sequence[float], poly: np.ndarray) -> np.ndarray:
    """Minimum Euclidean distance from each point (px, py) to a polyline (M x 2)."""
    P = np.column_stack([_as_array(px), _as_array(py)])
    poly = _as_array(poly).reshape(-1, 2)
    if poly.shape[0] == 0 or P.shape[0] == 0:
        return np.full(P.shape[0], np.nan)
    if poly.shape[0] == 1:
        return np.linalg.norm(P - poly[0], axis=1)
    A = poly[:-1]
    B = poly[1:]
    AB = B - A                                   # (S, 2)
    AB2 = np.sum(AB * AB, axis=1)                # (S,)
    AB2[AB2 == 0] = 1e-12
    AP = P[:, None, :] - A[None, :, :]           # (N, S, 2)
    s = np.clip(np.sum(AP * AB[None, :, :], axis=2) / AB2[None, :], 0.0, 1.0)
    proj = A[None, :, :] + s[:, :, None] * AB[None, :, :]
    d = np.linalg.norm(P[:, None, :] - proj, axis=2)
    return d.min(axis=1)


def path_length(x: Sequence[float], y: Sequence[float]) -> float:
    x = _as_array(x)
    y = _as_array(y)
    if x.size < 2:
        return 0.0
    return float(np.sum(np.hypot(np.diff(x), np.diff(y))))


# ---------------------------------------------------------------------------
#                       Category 1: low-level indices
# ---------------------------------------------------------------------------
def low_level_indices(
    t: Sequence[float],
    e_l: Sequence[float],
    e_r: Sequence[float],
    tau_l: Sequence[float],
    tau_r: Sequence[float],
    cfg: MetricsConfig,
    t0: Optional[float] = None,
    t1: Optional[float] = None,
) -> Dict[str, float]:
    t = _as_array(t)
    m = window_mask(t, t0, t1)
    if np.count_nonzero(m) < 2:
        return {"SAE1": float("nan"), "SCI1": float("nan"), "J1": float("nan")}
    e_n = (np.abs(_as_array(e_r)[m]) + np.abs(_as_array(e_l)[m])) / (2.0 * cfg.v_wheel_max)
    u_n = (np.abs(_as_array(tau_r)[m]) + np.abs(_as_array(tau_l)[m])) / (2.0 * cfg.tau_max)
    sae = time_average(t[m], e_n)
    sci = time_average(t[m], u_n)
    return {"SAE1": sae, "SCI1": sci, "J1": 0.5 * (sae + sci)}


# ---------------------------------------------------------------------------
#                       Category 2: mid-level indices
# ---------------------------------------------------------------------------
def _dist_to_plans(tg: np.ndarray, xg: np.ndarray, yg: np.ndarray,
                   plans: List[Tuple[float, np.ndarray]]) -> np.ndarray:
    """Distance from each (resampled) pose to the plan that was valid at that instant
    (plans are recomputed during the run, so the latest one published is used)."""
    e = np.full(tg.size, np.nan)
    plans = sorted([p for p in plans if len(p[1])], key=lambda p: p[0])
    if not plans:
        return e
    t_plans = np.array([p[0] for p in plans])
    idx = np.searchsorted(t_plans, tg, side="right") - 1
    idx[idx < 0] = 0  # before the first plan, use the first one
    for k in np.unique(idx):
        sel = idx == k
        e[sel] = dist_points_polyline(xg[sel], yg[sel], plans[k][1])
    return e


def prediction_error(t_pose, x, y, local_plans, cfg: MetricsConfig, t0: float, t1: float):
    """(t, e): distance from the position at t + cfg.horizon to the local plan valid at t,
    for t in [t0, t1 - horizon] on the cfg.fs grid."""
    tg = time_grid(t0, t1 - cfg.horizon, cfg.fs)
    if tg.size < 1 or len(t_pose) < 2:
        return tg, np.full(tg.size, np.nan)
    xr = resample(t_pose, x, tg + cfg.horizon)
    yr = resample(t_pose, y, tg + cfg.horizon)
    return tg, _dist_to_plans(tg, xr, yr, local_plans)


def mid_level_indices(
    t_pose: Sequence[float],
    x: Sequence[float],
    y: Sequence[float],
    t_cmd: Sequence[float],
    v_cmd: Sequence[float],
    w_cmd: Sequence[float],
    local_plans: List[Tuple[float, np.ndarray]],
    cfg: MetricsConfig,
    t0: float,
    t1: float,
    ref_path: Optional[np.ndarray] = None,
    ref_plans: Optional[List[Tuple[float, np.ndarray]]] = None,
) -> Dict[str, float]:
    """Mid level: tracking error and normalised command effort.

    SAE2: with ref_path (category 2), distance from the robot position to the
    common reference path, the same for every controller, normalised by d_ref and
    averaged on the common time grid. With ref_plans (category 3), distance from
    the robot position to the global plan valid at each instant, i.e. the reference
    that the mid-level controller is following. Without either, distance from the
    robot position at t + h to the local path of the controller at t.
    SAE2_pred (diagnostic, always computed when local plans exist): the latter
    prediction error. It is the distance to the trajectory the controller plans
    (TEB) or to the path it follows (RPP, MPPI), and it decreases when the robot
    slows down, so it is not used to compare controllers.
    SCI2: normalised (v, w)."""
    tg = time_grid(t0, t1, cfg.fs)
    if tg.size < 2 or len(t_pose) < 2:
        return {"SAE2": float("nan"), "SAE2_pred": float("nan"), "SCI2": float("nan"), "J2": float("nan")}
    e = prediction_error(t_pose, x, y, local_plans, cfg, t0, t1)[1]
    tge = time_grid(t0, t1 - cfg.horizon, cfg.fs)
    e_n = e / cfg.d_ref
    valid = ~np.isnan(e_n)
    sae_pred = time_average(tge[valid], e_n[valid]) if np.count_nonzero(valid) > 1 else float("nan")
    if ref_path is not None and len(ref_path) >= 2:
        xr, yr = resample(t_pose, x, tg), resample(t_pose, y, tg)
        sae = time_average(tg, dist_points_polyline(xr, yr, np.asarray(ref_path, float)) / cfg.d_ref)
    elif ref_plans:
        xr, yr = resample(t_pose, x, tg), resample(t_pose, y, tg)
        e_r = _dist_to_plans(tg, xr, yr, ref_plans) / cfg.d_ref
        ok = ~np.isnan(e_r)
        sae = time_average(tg[ok], e_r[ok]) if np.count_nonzero(ok) > 1 else float("nan")
    else:
        sae = sae_pred
    vg = resample_hold(t_cmd, v_cmd, tg)
    wg = resample_hold(t_cmd, w_cmd, tg)
    u_n = np.sqrt((vg / cfg.v_max) ** 2 + (wg / cfg.w_max) ** 2) / math.sqrt(2.0)
    sci = time_average(tg, u_n)
    return {"SAE2": sae, "SAE2_pred": sae_pred, "SCI2": sci, "J2": 0.5 * (sae + sci)}


# ---------------------------------------------------------------------------
#                       Category 3: high-level indices
# ---------------------------------------------------------------------------
def high_level_indices(
    t_pose: Sequence[float],
    x: Sequence[float],
    y: Sequence[float],
    plans: List[Tuple[float, np.ndarray]],
    cfg: MetricsConfig,
    t0: float,
    t1: float,
    first_plan: Optional[np.ndarray] = None,
) -> Dict[str, float]:
    """High level.

    SAE3: distance from the executed trajectory to the first plan of the task (the
    answer of the planner at the start, computed with the prior map), normalised by
    d_ref. Unlike the distance to the plan valid at each instant, which is recomputed
    from the current robot position every second and therefore shrinks when the
    robot moves slowly, it measures how far the execution departs from the plan.
    SAE3_cur (diagnostic): distance to the plan valid at each instant."""
    tg = time_grid(t0, t1, cfg.fs)
    nan = {"SAE3": float("nan"), "SAE3_cur": float("nan"), "J3": float("nan")}
    if tg.size < 2 or len(t_pose) < 2 or not plans:
        return nan
    xg = resample(t_pose, x, tg)
    yg = resample(t_pose, y, tg)
    e_c = _dist_to_plans(tg, xg, yg, plans) / cfg.d_ref
    ok = ~np.isnan(e_c)
    sae_cur = time_average(tg[ok], e_c[ok]) if np.count_nonzero(ok) > 1 else float("nan")
    if first_plan is not None and len(first_plan) >= 2:
        sae = time_average(tg, dist_points_polyline(xg, yg, np.asarray(first_plan, float)) / cfg.d_ref)
    else:
        sae = sae_cur
    return {"SAE3": sae, "SAE3_cur": sae_cur, "J3": sae}


def path_quality(
    t_pose: Sequence[float],
    x: Sequence[float],
    y: Sequence[float],
    goal_xy: Optional[Tuple[float, float]],
    t_scan: Sequence[float],
    scan_min_range: Sequence[float],
    cfg: MetricsConfig,
    t0: float,
    t1: float,
    reference_length: Optional[float] = None,
    final_status: str = "",
    t_collision: Optional[Sequence[float]] = None,
    collision: Optional[Sequence[float]] = None,
) -> Dict[str, float]:
    """Indicators of whether the executed path is short, safe and useful."""
    t_pose = _as_array(t_pose)
    m = window_mask(t_pose, t0, t1)
    xs, ys = _as_array(x)[m], _as_array(y)[m]
    out: Dict[str, float] = {}
    L_exec = path_length(xs, ys)
    out["path_length"] = L_exec
    if reference_length is None and goal_xy is not None and xs.size:
        reference_length = float(math.hypot(goal_xy[0] - xs[0], goal_xy[1] - ys[0]))
    out["reference_length"] = float(reference_length) if reference_length else float("nan")
    out["length_ratio"] = (L_exec / reference_length) if reference_length else float("nan")
    # Safety: distance from the LiDAR to the nearest obstacle (returns closer than
    # cfg.lidar_min_range are already discarded by the recorder)
    ts = _as_array(t_scan)
    ms = window_mask(ts, t0, t1)
    rmin = _as_array(scan_min_range)[ms]
    rmin = rmin[np.isfinite(rmin)]
    out["min_obstacle_dist"] = float(np.min(rmin)) if rmin.size else float("nan")
    out["mean_obstacle_dist"] = float(np.mean(rmin)) if rmin.size else float("nan")
    out["contacts"] = float("nan")
    # Contacts: collisions reported by the simulator (ground truth)
    if t_collision is not None and collision is not None and len(t_collision):
        tc = _as_array(t_collision)
        col = _as_array(collision)[window_mask(tc, t0, t1)] > 0.5
        out["contacts"] = int(np.count_nonzero(col[1:] & ~col[:-1]) + int(col[0])) if col.size else 0
    # Usefulness: success and time to goal
    final_err = float("nan")
    if goal_xy is not None and xs.size:
        final_err = float(math.hypot(goal_xy[0] - xs[-1], goal_xy[1] - ys[-1]))
    out["final_goal_error"] = final_err
    reached = (final_status.upper() == "SUCCEEDED") or (
        not math.isnan(final_err) and final_err <= cfg.goal_tolerance
    )
    out["success"] = 1 if reached else 0
    out["time_to_goal"] = float(t1 - t0) if reached else float("nan")
    return out


# ---------------------------------------------------------------------------
#                              Composite indices
# ---------------------------------------------------------------------------
def total_index(*layer_indices: float) -> float:
    vals = [v for v in layer_indices if v is not None and not math.isnan(v)]
    return float(np.mean(vals)) if vals else float("nan")


def sector_from_mu(mu: float, mu_zones: Sequence[float], tol: float = 1e-6) -> int:
    """Terrain sector (1-based index of the zone whose mu matches), 0 if unknown."""
    for i, mz in enumerate(mu_zones, start=1):
        if mz > 0 and abs(mu - mz) <= tol:
            return i
    return 0
