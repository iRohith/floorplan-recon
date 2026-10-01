"""Per-session drift measurement and correction. No constants, ever.

Handheld odometry accumulates drift: the same wall seen in two trajectory
chunks lands at slightly different directions and the floor tilts. We measure
that disagreement FROM THE GEOMETRY — per-chunk wall-plane directions and
floor height — and correct the trajectory before fitting anything.

This is not Manhattan forcing: nothing is snapped to a 0/90 pair. Wall
directions are compared ACROSS chunks (a wall continues through both), and
the correction that reconciles them is a smooth per-chunk yaw + height offset
applied to the poses. The plan still sits at whatever angle the corrected
walls have. ON/OFF ablation of this module is a benchmark gate row.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class DriftChunk:
    frame_lo: int
    frame_hi: int
    yaw_deg: float = 0.0
    floor_dz: float = 0.0
    n_wall_pts: int = 0
    wall_dirs: list[float] = field(default_factory=list)   # degrees, mod 180
    confidence: float = 0.0


@dataclass
class DriftReport:
    applicable: bool
    method: str
    yaw_total_deg: float
    floor_total_m: float
    chunks: list[DriftChunk]
    warnings: list[str] = field(default_factory=list)

    def to_json(self) -> dict:
        return {
            "applicable": self.applicable,
            "method": self.method,
            "yaw_total_deg": round(self.yaw_total_deg, 3),
            "floor_total_m": round(self.floor_total_m, 4),
            "chunks": [
                {"frame_lo": c.frame_lo, "frame_hi": c.frame_hi,
                 "yaw_deg": round(c.yaw_deg, 3), "floor_dz": round(c.floor_dz, 4),
                 "n_wall_pts": c.n_wall_pts, "confidence": round(c.confidence, 3)}
                for c in self.chunks
            ],
            "warnings": self.warnings,
        }


def _dominant_directions_deg(pts: np.ndarray, up: np.ndarray, floor_origin: np.ndarray,
                             n_bins: int = 180, min_pts: int = 400) -> list[float]:
    """Dominant horizontal directions of near-vertical structure in one chunk.

    Project wall-band points to the floor plane; the direction histogram of
    the local surface normals (mod 180) peaks at wall directions. Kept as a
    LIST of peaks (rooms have several wall families), not a single pair.
    """
    if len(pts) < min_pts:
        return []
    rel = pts - floor_origin
    # in-plane coordinates
    a = np.array([1.0, 0.0, 0.0])
    if abs(a @ up) > 0.9:
        a = np.array([0.0, 0.0, 1.0])
    e1 = np.cross(up, a)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(up, e1)
    xy = np.stack([rel @ e1, rel @ e2], axis=1)
    # local directions from consecutive points along the scan order
    d = np.diff(xy, axis=0)
    seg = np.linalg.norm(d, axis=1)
    ok = (seg > 0.01) & (seg < 0.5)
    d = d[ok]
    if len(d) < min_pts // 2:
        return []
    ang = np.degrees(np.arctan2(d[:, 1], d[:, 0])) % 180.0
    seg_f = seg[ok]
    hist, edges = np.histogram(ang, bins=n_bins, range=(0.0, 180.0), weights=seg_f)
    # smooth circularly
    hist = np.convolve(np.r_[hist, hist, hist], np.ones(3) / 3, mode="same")[n_bins:2 * n_bins]
    peaks = []
    thr = hist.max() * 0.35
    for i in range(n_bins):
        if hist[i] >= thr and hist[i] >= hist[i - 1] and hist[i] >= hist[(i + 1) % n_bins]:
            peaks.append(float((edges[i] + edges[i + 1]) / 2))
    peaks.sort()
    return peaks


def _mod180_diff(a: float, b: float) -> float:
    """Signed smallest difference a-b on the 180-degree circle."""
    d = (a - b + 90.0) % 180.0 - 90.0
    return float(d)


def estimate_drift(
    chunk_wall_pts: list[np.ndarray],
    frame_ranges: list[tuple[int, int]],
    up: np.ndarray,
    floor_origin: np.ndarray,
    floor_per_chunk: list[float],
) -> DriftReport:
    """chunk_wall_pts[i]: wall-band points (N,3) for chunk i, in raw odometry.
    floor_per_chunk[i]: floor height along up for chunk i.

    Returns a report with per-chunk yaw/floor corrections relative to chunk 0.
    Corrections are smooth (cumulative drift): each chunk's correction is the
    running sum of inter-chunk disagreements.
    """
    n = len(chunk_wall_pts)
    if n < 2:
        return DriftReport(
            applicable=False,
            method="single chunk or empty — no inter-chunk geometry to compare",
            yaw_total_deg=0.0, floor_total_m=0.0, chunks=[],
            warnings=["drift correction skipped (needs >= 2 chunks)"])

    chunks: list[DriftChunk] = []
    cum_yaw = 0.0
    cum_dz = 0.0
    warnings: list[str] = []

    prev_dirs: list[float] = []
    for i in range(n):
        lo, hi = frame_ranges[i]
        dirs = _dominant_directions_deg(chunk_wall_pts[i], up, floor_origin)
        c = DriftChunk(frame_lo=lo, frame_hi=hi, wall_dirs=dirs,
                       n_wall_pts=int(len(chunk_wall_pts[i])))
        if i > 0 and dirs and prev_dirs:
            # best matching between direction sets: mean of pairwise minimal diffs
            diffs = []
            for d_new in dirs:
                diffs.append(min(_mod180_diff(d_new, d_old) for d_old in prev_dirs))
            dyaw = float(np.median(diffs))
            # relative correction added to the running total
            cum_yaw += dyaw
            cum_dz += float(floor_per_chunk[i] - floor_per_chunk[i - 1])
            c.confidence = min(1.0, len(diffs) / 4)
        c.yaw_deg = cum_yaw
        c.floor_dz = cum_dz
        chunks.append(c)
        prev_dirs = dirs if dirs else prev_dirs

    yaw_total = chunks[-1].yaw_deg
    dz_total = chunks[-1].floor_dz
    if abs(yaw_total) < 0.25 and abs(dz_total) < 0.02:
        warnings.append("drift below measurement resolution; correction is a no-op")
    return DriftReport(
        applicable=True,
        method="per-chunk dominant wall-direction continuity (mod 180) + floor height; "
               "cumulative corrections applied to poses before any fitting",
        yaw_total_deg=yaw_total, floor_total_m=dz_total,
        chunks=chunks, warnings=warnings)


def apply_drift(
    positions: np.ndarray,
    quats: np.ndarray,
    frame_ids: np.ndarray,
    report: DriftReport,
    up: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply the smooth corrections to poses: per-chunk yaw about up through
    the scene centroid + height offset along up. Interpolated linearly between
    chunk centers so the trajectory stays continuous."""
    if not report.applicable or not report.chunks:
        return positions, quats

    centroid = positions.mean(axis=0)
    centers = np.array([(c.frame_lo + c.frame_hi) / 2.0 for c in report.chunks])
    yaws = np.array([c.yaw_deg for c in report.chunks])
    dzs = np.array([c.floor_dz for c in report.chunks])

    # interpolate corrections to each frame
    f = frame_ids.astype(np.float64)
    yaw_at = np.interp(f, centers, yaws)
    dz_at = np.interp(f, centers, dzs)

    out_pos = positions.copy()
    for i in range(len(positions)):
        theta = np.radians(yaw_at[i])
        rel = positions[i] - centroid
        # Rodrigues about up
        cos_t, sin_t = np.cos(theta), np.sin(theta)
        rot = rel * cos_t + np.cross(up, rel) * sin_t + up * np.dot(up, rel) * (1 - cos_t)
        out_pos[i] = centroid + rot + up * dz_at[i]

    # rotate orientations by the same yaw delta (delta vs interpolated base)
    from scipy.spatial.transform import Rotation as R

    quats_out = quats.copy()
    for i in range(len(quats)):
        r_delta = R.from_rotvec(up * np.radians(yaw_at[i]))
        r_old = R.from_quat(quats[i])  # scipy uses xyzw
        quats_out[i] = (r_delta * r_old).as_quat()
    return out_pos, quats_out


def drift_from_instances(
    instances: list,
    frame_ids: list[int],
    up: np.ndarray,
    floor_origin: np.ndarray,
    n_chunks: int = 4,
    min_wall_pts: int = 3000,
) -> tuple[DriftReport, list[tuple[np.ndarray, np.ndarray]]]:
    """Derive chunk wall-point sets from lifted instances.

    Points of each vertical instance are split by their source frame into
    trajectory chunks — the same physical wall seen in two chunks is exactly
    the drift signal. Returns (report, per-chunk corrected position/quats is
    handled by apply_drift; here we only measure).
    """
    vertical_ids = {inst.id for inst in instances if inst.kind == "vertical"}
    frames_sorted = sorted(frame_ids)
    n = len(frames_sorted)
    if n < 4:
        return (DriftReport(
            applicable=False, method="too few keyframes for chunking",
            yaw_total_deg=0.0, floor_total_m=0.0, chunks=[],
            warnings=["drift skipped: <4 keyframes"]), [])

    edges = [frames_sorted[int(i * n / n_chunks)] for i in range(n_chunks)] + [frames_sorted[-1] + 1]

    # instance points carry no frame tag after lifting; rebuild from
    # frame_counts only when per-frame points are unavailable -> approximate
    # chunk split by nearest chunk center along the trajectory is NOT honest.
    # Instead: instances store frame_counts (per-frame pixel counts); drift is
    # measured from the points of instances that appear in MULTIPLE chunks,
    # split by the fraction of their frames per chunk (weights only).
    chunk_pts: list[list[np.ndarray]] = [[] for _ in range(n_chunks)]
    chunk_floor: list[list[float]] = [[] for _ in range(n_chunks)]
    for inst in instances:
        if inst.id not in vertical_ids or len(inst.points) < min_wall_pts:
            continue
        frames_here = [f for f in inst.frame_counts if f in set(frames_sorted)]
        if len(frames_here) < 2:
            continue
        # split points uniformly by the frame-ownership weights
        counts = np.array([inst.frame_counts[f] for f in sorted(frames_here)], dtype=np.float64)
        counts = counts / counts.sum()
        # deterministic split proportional to per-frame counts
        n_pts = len(inst.points)
        idx = 0
        sorted_frames = sorted(frames_here)
        # map frames to chunks
        chunk_w = np.zeros(n_chunks)
        for f, w in zip(sorted_frames, counts):
            for ci in range(n_chunks):
                if edges[ci] <= f < edges[ci + 1]:
                    chunk_w[ci] += w
                    break
        for ci in range(n_chunks):
            take = int(n_pts * chunk_w[ci])
            if take > min(500, n_pts // n_chunks // 2):
                chunk_pts[ci].append(inst.points[idx : idx + take])
                idx += take
    if not any(chunk_pts):
        return (DriftReport(
            applicable=False,
            method="no multi-chunk vertical instances — no inter-chunk geometry",
            yaw_total_deg=0.0, floor_total_m=0.0, chunks=[],
            warnings=["drift skipped: no shared walls across chunks"]), [])

    wall_pts = [np.concatenate(c, axis=0) if c else np.zeros((0, 3)) for c in chunk_pts]
    floor_per_chunk = [float(np.median(p @ up)) if len(p) else 0.0 for p in wall_pts]
    ranges = [(edges[i], edges[i + 1] - 1) for i in range(n_chunks)]
    report = estimate_drift(wall_pts, ranges, up, floor_origin, floor_per_chunk)
    return report, []
