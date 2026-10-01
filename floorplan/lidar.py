#!/usr/bin/env python3
"""Cold-run LiDAR floor-plan CLI.

Input is a Stray Scanner export (rgb.mp4, odometry.csv, depth/, confidence/).
The pipeline fuses depth in the capture frame, identifies floor/ceiling/wall
surfaces, obtains a data-driven room count, asks SAM to refine each room, and
writes a confidence-bearing metric plan plus a presentation sheet.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import resource
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import matplotlib
import numpy as np
from scipy import ndimage
from scipy.signal import find_peaks
from shapely.geometry import Polygon
from shapely.ops import unary_union

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Polygon as PlotPolygon  # noqa: E402

SCHEMA_VERSION = "0.1.0"
GRID_M = 0.025
SAM_PPM = 50.0
PALETTE = ["#dbeafe", "#dcfce7", "#fef3c7", "#f3e8ff", "#fee2e2", "#cffafe", "#ede9fe", "#fce7f3"]


@dataclass
class Capture:
    root: Path
    frames: np.ndarray
    positions: np.ndarray
    quats: np.ndarray
    intrinsics: np.ndarray

    def depth_path(self, i: int) -> Path:
        return self.root / "depth" / f"{int(self.frames[i]):06d}.png"


def measurement(value: float, half: float, unit: str, method: str) -> dict:
    half = max(float(half), 0.0)
    return {
        "value": round(float(value), 4),
        "ci95": [round(float(value - half), 4), round(float(value + half), 4)],
        "unit": unit,
        "method": method,
    }


def hash_capture(root: Path) -> str:
    h = hashlib.sha256()
    for rel in ("odometry.csv", "camera_matrix.csv"):
        p = root / rel
        if p.exists():
            h.update(p.read_bytes())
    depth = sorted((root / "depth").glob("*.png"))
    h.update(str(len(depth)).encode())
    if depth:
        for p in (depth[0], depth[-1]):
            h.update(p.name.encode())
            h.update(str(p.stat().st_size).encode())
    return h.hexdigest()


def load_capture(root: Path) -> Capture:
    required = [root / "odometry.csv", root / "rgb.mp4", root / "depth"]
    missing = [str(p) for p in required if not p.exists()]
    if missing:
        raise ValueError(f"not a Stray Scanner folder; missing: {missing}")
    rows = []
    with (root / "odometry.csv").open(newline="") as f:
        for row in csv.DictReader(f, skipinitialspace=True):
            rows.append({k.strip(): v for k, v in row.items() if k})
    if not rows:
        raise ValueError("odometry.csv is empty")
    frames = np.asarray([int(r["frame"]) for r in rows])
    positions = np.asarray([[float(r[k]) for k in ("x", "y", "z")] for r in rows], np.float64)
    quats = np.asarray([[float(r[k]) for k in ("qx", "qy", "qz", "qw")] for r in rows], np.float64)
    intrinsics = np.asarray([[float(r[k]) for k in ("fx", "fy", "cx", "cy")] for r in rows], np.float64)
    n_depth = sum(1 for _ in (root / "depth").glob("*.png"))
    if n_depth != len(rows):
        raise ValueError(f"depth/ has {n_depth} maps but odometry has {len(rows)} rows")
    return Capture(root, frames, positions, quats, intrinsics)


def quat_rotation(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q / (np.linalg.norm(q) + 1e-12)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], np.float32)


def depth_m(path: Path) -> np.ndarray:
    raw = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if raw is None:
        raise IOError(f"cannot read {path}")
    out = raw.astype(np.float32) / 1000.0
    out[~np.isfinite(out)] = 0
    return out


def estimate_up(path: np.ndarray) -> np.ndarray:
    centred = path - path.mean(axis=0)
    _, vec = np.linalg.eigh(centred.T @ centred)
    up = vec[:, 0]
    if up[1] < 0:
        up = -up
    return up / np.linalg.norm(up)


def planar_basis(up: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    seed = np.array([1.0, 0.0, 0.0])
    if abs(seed @ up) > 0.9:
        seed = np.array([0.0, 0.0, 1.0])
    e1 = np.cross(up, seed); e1 /= np.linalg.norm(e1)
    return e1, np.cross(up, e1)


WALL_BAND = 0.25


def drift_correct(cap: Capture, up: np.ndarray, target_frames: int = 260,
                  n_chunks: int = 5) -> tuple[Capture, "DriftReport"]:
    """Measure drift from the geometry itself, then correct the poses.

    The same wall is observed in every trajectory chunk. Where chunk directions
    and floor heights disagree, the disagreement is accumulated drift. Corrections
    are smooth per-chunk yaw about `up` plus a height offset, applied to the poses
    before anything is fitted. Nothing is snapped to a world axis.
    """
    from .drift import DriftReport, apply_drift, estimate_drift
    step = max(1, len(cap.frames) // max(target_frames, 1))
    idx = np.arange(0, len(cap.frames), step)
    bounds = [b for b in np.array_split(idx, n_chunks) if len(b)]
    wall_pts, ranges, floors = [], [], []
    for blk in bounds:
        chunk_wall, chunk_floor = [], []
        for i in blk:
            z = depth_m(cap.depth_path(int(i)))
            h, w = z.shape
            fx, fy, cx, cy = cap.intrinsics[i]
            sx = w / max(2.0*cx, w); sy = h / max(2.0*cy, h)
            fx, fy, cx, cy = fx*sx, fy*sy, cx*sx, cy*sy
            uu, vv = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
            ok = (z > 0.2) & (z < 7.0)
            if not ok.any():
                continue
            xm = (uu-cx)*z/fx; ym = (vv-cy)*z/fy
            du = np.stack([np.gradient(xm, axis=1), np.gradient(ym, axis=1), np.gradient(z, axis=1)], -1)
            dv = np.stack([np.gradient(xm, axis=0), np.gradient(ym, axis=0), np.gradient(z, axis=0)], -1)
            nrm = np.cross(du, dv).reshape(-1, 3)
            nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
            pts = np.stack([xm.ravel(), ym.ravel(), z.ravel()], 1)
            R = quat_rotation(cap.quats[i])
            wp = pts @ R.T + cap.positions[i]
            wn = nrm @ R.T
            sel = ok.ravel()
            ndu = np.abs(wn @ up)
            vertical = wp[sel][ndu[sel] < 0.30]
            if vertical.size:
                chunk_wall.append(vertical)
            fl = (wp[sel][ndu[sel] > 0.92]) @ up
            if fl.size:
                chunk_floor.append(float(np.median(fl)))
        if chunk_wall:
            # Exactly one wall point set, one frame range and one floor height
            # per chunk, so the three lists stay index-aligned for estimate_drift.
            wall_pts.append(np.concatenate(chunk_wall))
            ranges.append((int(blk[0]), int(blk[-1])))
            floors.append(float(np.median(chunk_floor)) if chunk_floor else 0.0)
    if len(wall_pts) < 2:
        return cap, DriftReport(applicable=False, method="insufficient chunk evidence",
                                yaw_total_deg=0.0, floor_total_m=0.0, chunks=[],
                                warnings=["drift skipped: fewer than two chunks with wall evidence"])
    report = estimate_drift(wall_pts, ranges, up, np.zeros(3), floors)
    positions, quats = apply_drift(cap.positions, cap.quats, cap.frames.astype(float), report, up)
    moved = float(np.abs(positions - cap.positions).max())
    report.warnings.append(f"max pose displacement {moved:.4f} m")
    return Capture(cap.root, cap.frames, positions, quats, cap.intrinsics), report


def fuse_capture(cap: Capture, target_frames: int = 1250, target_points: int = 4_500_000,
                 drift: bool = True, chunk_frames: int = 400) -> dict[str, np.ndarray]:
    frame_step = max(1, math.ceil(len(cap.frames) / target_frames))
    chosen = np.arange(0, len(cap.frames), frame_step)
    points_per_frame = max(1, target_points // max(len(chosen), 1))
    up = estimate_up(cap.positions)
    drift_report = None
    if drift:
        cap, drift_report = drift_correct(cap, up)
    all_p, all_n = [], []
    for seq, i in enumerate(chosen):
        z = depth_m(cap.depth_path(int(i)))
        h, w = z.shape
        fx, fy, cx, cy = cap.intrinsics[i]
        # Stray intrinsics are normally at RGB resolution; infer scale from principal point.
        sx = w / max(2.0 * cx, w)
        sy = h / max(2.0 * cy, h)
        fx, fy, cx, cy = fx*sx, fy*sy, cx*sx, cy*sy
        uu, vv = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
        valid = (z > 0.08) & (z < 8.0)
        ids = np.flatnonzero(valid.ravel())
        stride = max(1, math.ceil(len(ids) / points_per_frame))
        ids = ids[::stride]
        if len(ids) == 0:
            continue
        xmap = (uu - cx) * z / fx
        ymap = (vv - cy) * z / fy
        du = np.stack([np.gradient(xmap, axis=1), np.gradient(ymap, axis=1), np.gradient(z, axis=1)], axis=-1)
        dv = np.stack([np.gradient(xmap, axis=0), np.gradient(ymap, axis=0), np.gradient(z, axis=0)], axis=-1)
        normals = np.cross(du, dv).reshape(-1, 3)[ids]
        normals /= np.linalg.norm(normals, axis=1, keepdims=True) + 1e-12
        p = np.stack([xmap.ravel()[ids], ymap.ravel()[ids], z.ravel()[ids]], axis=1)
        R = quat_rotation(cap.quats[i])
        world_p = p @ R.T + cap.positions[i]
        all_p.append(world_p)
        all_n.append(normals @ R.T)
        if seq and seq % 150 == 0:
            print(f"  fused {seq}/{len(chosen)} sampled frames")
    if not all_p:
        raise ValueError("capture produced no valid depth points")
    P = np.concatenate(all_p).astype(np.float32)
    N = np.concatenate(all_n).astype(np.float32)
    heights = P @ up
    ndu = np.abs(N @ up)
    horizontal = ndu > 0.82
    if horizontal.sum() < 1000:
        raise ValueError("too few horizontal points to find floor and ceiling")
    hh = heights[horizontal]
    floor_h = float(np.percentile(hh, 8))
    # Refine the floor around its lower quantile. For the ceiling, choose the
    # strongest horizontal-surface peak at a plausible room height above the
    # floor. Upper quantiles are biased by tabletops and overhead scan tails.
    bins = np.arange(float(hh.min()), float(hh.max()) + 0.02, 0.02)
    hist, edges = np.histogram(hh, bins=bins)
    peaks, _ = find_peaks(hist, distance=5, prominence=max(20, 0.001 * hist.sum()))
    centres = (edges[peaks] + edges[peaks + 1]) / 2
    low = centres[centres < np.median(hh)]
    if len(low):
        floor_h = float(low[np.argmin(abs(low - floor_h))])
    ceiling_peak_mask = (centres >= floor_h + 2.05) & (centres <= floor_h + 3.25)
    ceiling_peaks = peaks[ceiling_peak_mask]
    if len(ceiling_peaks):
        ceil_h = float(centres[np.flatnonzero(ceiling_peak_mask)[np.argmax(hist[ceiling_peaks])]])
    else:
        ceil_h = float(np.percentile(hh, 88))
    cls = np.zeros(len(P), np.uint8)
    cls[horizontal & (np.abs(heights - floor_h) < 0.12)] = 1
    cls[horizontal & (np.abs(heights - ceil_h) < 0.15)] = 2
    wall = (ndu < 0.30) & (heights > floor_h + 0.25) & (heights < ceil_h - 0.08)
    cls[wall] = 3
    xy = np.stack([P @ e1, P @ e2], axis=1).astype(np.float32)
    trajectory = np.stack([cap.positions @ e1, cap.positions @ e2], axis=1).astype(np.float32)
    return {"P": P, "N": N, "xy": xy, "cls": cls, "up": up, "e1": e1, "e2": e2,
            "floor_h": np.float64(floor_h), "ceil_h": np.float64(ceil_h), "trajectory": trajectory,
            "frame_step": np.int64(frame_step),
            "drift_yaw_deg": np.float64(getattr(drift_report, "yaw_total_deg", 0.0)),
            "drift_floor_m": np.float64(getattr(drift_report, "floor_total_m", 0.0)),
            "drift_applicable": np.bool_(bool(getattr(drift_report, "applicable", False)))}


def occupancy(points: np.ndarray, lo: np.ndarray, shape: tuple[int, int], ppm: float) -> np.ndarray:
    out = np.zeros(shape, np.float32)
    col = np.clip(((points[:, 0] - lo[0]) * ppm).astype(int), 0, shape[1]-1)
    row = np.clip(((points[:, 1] - lo[1]) * ppm).astype(int), 0, shape[0]-1)
    np.add.at(out, (row, col), 1)
    return out


def wall_faces(state: dict[str, np.ndarray]) -> tuple[list[dict], list[float]]:
    sel = state["cls"] == 3
    normals = state["N"] if "N" in state else state["Nn"]
    xy, N, e1, e2 = state["xy"][sel], normals[sel], state["e1"], state["e2"]
    height = (state["P"] @ state["up"] - float(state["floor_h"]))[sel]
    n2 = np.stack([N @ e1, N @ e2], axis=1)
    az = np.mod(np.arctan2(n2[:, 1], n2[:, 0]), np.pi)
    step = np.deg2rad(2.0)
    hist, edges = np.histogram(az, bins=np.arange(0, np.pi + step, step))
    peaks, prop = find_peaks(np.r_[hist, hist[:3]], height=max(80, 0.025*len(az)), distance=8)
    peaks = peaks[peaks < len(hist)]
    ordered = peaks[np.argsort(-hist[peaks])]
    families = []
    for p in ordered:
        a = float((edges[p] + edges[p+1]) / 2)
        if all(min(abs(a-b), np.pi-abs(a-b)) > np.deg2rad(18) for b in families):
            families.append(a)
        if len(families) == 4:
            break
    families.sort()
    faces = []
    for fi, a in enumerate(families):
        n = np.array([math.cos(a), math.sin(a)])
        t = np.array([-n[1], n[0]])
        agree = np.abs(n2 @ n) > math.cos(math.radians(22))
        if agree.sum() < 200:
            continue
        pos, along, hgt = xy[agree] @ n, xy[agree] @ t, height[agree]
        normal_sign = n2[agree] @ n
        lo, hi = float(pos.min()), float(pos.max())
        bins = np.arange(lo-0.025, hi+0.05, 0.025)
        hp, ed = np.histogram(pos, bins=bins)
        pk, _ = find_peaks(hp, height=max(100, 0.002*agree.sum()), distance=2)
        for k in pk:
            c = float((ed[k]+ed[k+1])/2)
            band = abs(pos-c) < 0.045
            if band.sum() < 100:
                continue
            c = float(np.median(pos[band]))
            # Furniture can form a strong vertical plane, but unlike a wall it
            # does not carry points close to the measured ceiling. Require each
            # finite run to reach at least 72% of the room height.
            vals = along[band]
            order = np.argsort(vals)
            vals, run_hgt = vals[order], hgt[band][order]
            gaps = np.flatnonzero(np.diff(vals) > 0.22)
            for ids in np.split(np.arange(len(vals)), gaps+1):
                run = vals[ids]
                if len(run) < 80 or run[-1]-run[0] < 0.55:
                    continue
                top = float(np.percentile(run_hgt[ids], 97))
                if top < 0.72 * (float(state["ceil_h"]) - float(state["floor_h"])):
                    continue
                signs = normal_sign[band][order][ids]
                pos_frac = float((signs > 0).mean())
                faces.append({"family": fi, "normal": n, "tangent": t, "pos": c,
                              "a": float(run[0]), "b": float(run[-1]),
                              "sigma": float(np.std(pos[band])), "support": int(len(run)),
                              "top_m": top, "normal_balance": min(pos_frac, 1.0-pos_frac)})
    return faces, [math.degrees(a) for a in families]


def raster_faces(faces: list[dict], lo: np.ndarray, shape: tuple[int, int], ppm: float) -> np.ndarray:
    out = np.zeros(shape, np.uint8)
    for f in faces:
        p0 = f["a"]*f["tangent"] + f["pos"]*f["normal"]
        p1 = f["b"]*f["tangent"] + f["pos"]*f["normal"]
        q0 = tuple(np.round((p0-lo)*ppm).astype(int))
        q1 = tuple(np.round((p1-lo)*ppm).astype(int))
        cv2.line(out, q0, q1, 1, max(3, int(round(0.10*ppm))))
    return out


def physical_wall_barriers(faces: list[dict], lo: np.ndarray, shape: tuple[int, int], ppm: float) -> tuple[np.ndarray, list[dict]]:
    """Pair opposite sides of physical walls, then close doorway-scale gaps.

    A sofa or cabinet can produce one ceiling-reaching vertical face, but a wall
    normally produces two parallel faces separated by its thickness. Only these
    paired bodies divide rooms. Collinear bodies are joined across gaps up to the
    robust lower mode of measured gaps (doors); no room count is assumed.
    """
    paired: list[dict] = []
    for i, f in enumerate(faces):
        best = None
        for j in range(i + 1, len(faces)):
            g = faces[j]
            if g["family"] != f["family"]:
                continue
            separation = abs(g["pos"] - f["pos"])
            overlap = min(f["b"], g["b"]) - max(f["a"], g["a"])
            if not (0.06 <= separation <= 0.35 and overlap >= 0.35):
                continue
            score = overlap / (separation + 0.03)
            if best is None or score > best[0]:
                best = (score, g, separation, overlap)
        if best is None:
            continue
        _, g, separation, _ = best
        paired.append({
            "family": f["family"], "normal": f["normal"], "tangent": f["tangent"],
            "pos": 0.5 * (f["pos"] + g["pos"]), "a": max(f["a"], g["a"]),
            "b": min(f["b"], g["b"]), "thickness": separation,
        })

    # First collect gaps between nearly collinear bodies. Otsu separates the
    # lower (opening) mode from genuinely missing wall spans when enough exist.
    ordered = sorted(paired, key=lambda x: (x["family"], x["pos"], x["a"]))
    candidate_gaps = []
    for i, a in enumerate(ordered):
        for b in ordered[i + 1:]:
            if b["family"] != a["family"] or abs(b["pos"] - a["pos"]) > 0.16:
                continue
            gap = b["a"] - a["b"]
            if 0.25 <= gap <= 2.4:
                candidate_gaps.append(gap)
    if len(candidate_gaps) >= 3:
        vals = np.asarray(candidate_gaps)
        hist, edges = np.histogram(vals, bins=min(16, max(5, len(vals))))
        prob = hist / max(hist.sum(), 1); centres = (edges[:-1] + edges[1:]) / 2
        omega = np.cumsum(prob); mu = np.cumsum(prob * centres); total = mu[-1]
        between = (total * omega - mu) ** 2 / np.maximum(omega * (1 - omega), 1e-12)
        close_limit = float(centres[int(np.nanargmax(between[:-1]))])
    else:
        close_limit = 1.0
    close_limit = float(np.clip(close_limit, 0.70, 1.35))

    merged: list[dict] = []
    for p in ordered:
        target = None
        for m in merged:
            if m["family"] == p["family"] and abs(m["pos"] - p["pos"]) <= 0.16 and p["a"] <= m["b"] + close_limit:
                target = m; break
        if target is None:
            target = dict(p)
            target["openings"] = []
            merged.append(target)
        else:
            gap = p["a"] - target["b"]
            if 0.25 <= gap <= close_limit:
                target["openings"].append({"a": float(target["b"]), "b": float(p["a"]), "width_m": float(gap)})
            w0 = target["b"] - target["a"]; w1 = p["b"] - p["a"]
            target["pos"] = (target["pos"] * w0 + p["pos"] * w1) / max(w0 + w1, 1e-9)
            target["a"] = min(target["a"], p["a"]); target["b"] = max(target["b"], p["b"])
            target["thickness"] = max(target["thickness"], p["thickness"])

    out = np.zeros(shape, np.uint8)
    for p in merged:
        a = p["a"] * p["tangent"] + p["pos"] * p["normal"]
        b = p["b"] * p["tangent"] + p["pos"] * p["normal"]
        q0 = tuple(np.round((a - lo) * ppm).astype(int)); q1 = tuple(np.round((b - lo) * ppm).astype(int))
        cv2.line(out, q0, q1, 1, max(4, int(round(max(0.10, p["thickness"]) * ppm))))
    return out, merged


def room_components(support: np.ndarray, barriers: np.ndarray, ppm: float) -> np.ndarray:
    labels, n = ndimage.label((support > 0) & (barriers == 0))
    candidates = []
    for i in range(1, n + 1):
        area = float((labels == i).sum()) / (ppm * ppm)
        if area >= 0.5:
            candidates.append((area, i))
    candidates.sort(reverse=True)
    out = np.zeros_like(labels, np.int32)
    for new, (_, old) in enumerate(candidates, 1):
        out[labels == old] = new
    return out


def select_sam_rooms(components: np.ndarray, proposals: list[np.ndarray], scores: list[float], ppm: float) -> tuple[np.ndarray, list[float], list[dict]]:
    """Use SAM to reject fragments while retaining metric component geometry."""
    rows = []
    for i in range(1, int(components.max()) + 1):
        seed = components == i
        sam = proposals[i - 1] if i - 1 < len(proposals) else seed
        seed_area = float(seed.sum()) / ppm**2
        sam_area = float(sam.sum()) / ppm**2
        inter = np.logical_and(seed, sam).sum()
        union = np.logical_or(seed, sam).sum()
        rows.append({"id": i, "seed_area": seed_area, "sam_area": sam_area,
                     "iou": float(inter / max(union, 1)),
                     "score": float(scores[i - 1]) if i - 1 < len(scores) else 0.0})
    sam_areas = np.asarray(sorted((r["sam_area"] for r in rows), reverse=True))
    area_floor = 0.5
    if len(sam_areas) >= 4:
        ratios = sam_areas[:-1] / np.maximum(sam_areas[1:], 1e-6)
        # A size gap is a fragment cutoff only when the smaller side is itself
        # fragment-scale. Otherwise a legitimate bathroom/utility room can be
        # discarded merely because the apartment also contains large rooms.
        eligible = [k for k, ratio in enumerate(ratios)
                    if ratio >= 2.2 and k + 1 >= 3 and sam_areas[k + 1] < 2.0]
        if eligible:
            k = max(eligible, key=lambda j: ratios[j])
            area_floor = float(math.sqrt(sam_areas[k] * sam_areas[k + 1]))
    keep = [r for r in rows if r["seed_area"] >= 1.0 and r["sam_area"] >= max(area_floor, 1.0) and r["score"] >= 0.75]
    keep.sort(key=lambda r: -r["seed_area"])
    out = np.zeros_like(components, np.int32); kept_scores = []
    clip_radius = max(1, int(round(0.16 * ppm)))
    clip_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*clip_radius+1, 2*clip_radius+1))
    for new, row in enumerate(keep, 1):
        component = components == row["id"]
        proposal = proposals[row["id"] - 1] if row["id"] - 1 < len(proposals) else component
        supported = component & (cv2.dilate(proposal.astype(np.uint8), clip_kernel) > 0)
        # SAM may under-segment a room behind furniture. Clip only when it keeps
        # most of the metric component; otherwise preserve measured geometry.
        chosen = supported if supported.sum() >= 0.72 * component.sum() else component
        chosen = ndimage.binary_fill_holes(chosen) & component
        out[chosen] = new
        row["sam_clip_retained_frac"] = float(chosen.sum() / max(component.sum(), 1))
        kept_scores.append(row["score"])
    return out, kept_scores, rows


def indoor_support(state: dict[str, np.ndarray], lo: np.ndarray, shape: tuple[int, int], ppm: float) -> np.ndarray:
    ceiling = occupancy(state["xy"][state["cls"] == 2], lo, shape, ppm)
    floor = occupancy(state["xy"][state["cls"] == 1], lo, shape, ppm)
    # Ceiling is the strongest indoor cue; floor covers rooms with missing ceiling returns.
    c = ceiling > 0
    c = ndimage.binary_closing(c, np.ones((9, 9)))
    c = ndimage.binary_fill_holes(c)
    f = ndimage.binary_closing(floor > 0, np.ones((11, 11)))
    support = c | (f & ndimage.binary_dilation(c, iterations=max(1, int(1.2*ppm))))
    support = ndimage.binary_closing(support, np.ones((7, 7)))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(support.astype(np.uint8), 8)
    if n > 1:
        keep = [i for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= 1.0*ppm*ppm]
        support = np.isin(lab, keep)
    return support.astype(np.uint8)


def room_seeds(free: np.ndarray, ppm: float) -> tuple[np.ndarray, int]:
    """One adaptive marker per local free-space basin.

    A fixed erosion radius merges small rooms and corridors. Local maxima of the
    clearance field are scale-adaptive: a bathroom and a living room each retain
    a centre. Non-maximum suppression is metric and deterministic.
    """
    dist = ndimage.distance_transform_edt(free) / ppm
    smooth = ndimage.gaussian_filter(dist, sigma=max(1.0, 0.08*ppm))
    vals = smooth[free > 0]
    if not len(vals):
        return np.zeros_like(free, np.int32), 0
    threshold = max(0.24, float(np.percentile(vals, 42)))
    window = max(5, int(round(0.72*ppm)))
    maxima = (smooth >= ndimage.maximum_filter(smooth, size=window)-1e-6) & (smooth >= threshold) & (free > 0)
    lab, n = ndimage.label(ndimage.binary_dilation(maxima, iterations=2))
    candidates = []
    for i in range(1, n+1):
        yy, xx = np.nonzero(lab == i)
        if len(xx):
            k = int(np.argmax(smooth[yy, xx]))
            candidates.append((float(smooth[yy[k], xx[k]]), int(yy[k]), int(xx[k])))
    candidates.sort(key=lambda q: (-q[0], q[1], q[2]))
    accepted = []
    for score, y, x in candidates:
        # Larger basins suppress farther; small spaces remain discoverable.
        radius = max(0.42, min(0.78, 0.70*score))*ppm
        if all((x-xx)**2 + (y-yy)**2 >= radius**2 for _, yy, xx in accepted):
            accepted.append((score, y, x))
    markers = np.zeros_like(free, np.int32)
    disk = max(2, int(round(0.08*ppm)))
    yy0, xx0 = np.ogrid[-disk:disk+1, -disk:disk+1]
    stamp = xx0*xx0 + yy0*yy0 <= disk*disk
    for i, (_, y, x) in enumerate(accepted, 1):
        y0,y1=max(0,y-disk),min(free.shape[0],y+disk+1); x0,x1=max(0,x-disk),min(free.shape[1],x+disk+1)
        sy0,sx0=y0-(y-disk),x0-(x-disk)
        sub=stamp[sy0:sy0+y1-y0,sx0:sx0+x1-x0] & (free[y0:y1,x0:x1]>0)
        markers[y0:y1,x0:x1][sub]=i
    return markers, len(accepted)


def sam_refine(topdown: np.ndarray, free: np.ndarray, seeds: np.ndarray, checkpoint: Path) -> tuple[list[np.ndarray], list[float], str]:
    boxes, points = [], []
    for i in range(1, int(seeds.max())+1):
        yy, xx = np.nonzero(seeds == i)
        if len(xx) == 0: continue
        pad = 12
        boxes.append([max(0, xx.min()-pad), max(0, yy.min()-pad), min(topdown.shape[1]-1, xx.max()+pad), min(topdown.shape[0]-1, yy.max()+pad)])
        local = seeds == i
        clearance = ndimage.distance_transform_edt(local)
        py, px = np.unravel_index(int(np.argmax(clearance)), clearance.shape)
        points.append([int(px), int(py)])
    if not boxes:
        return [], [], "no_room_seeds"
    if not checkpoint.exists():
        return [seeds == i for i in range(1, int(seeds.max())+1)], [0.0]*len(boxes), "seed_fallback_missing_checkpoint"
    try:
        import torch
        from segment_anything import SamPredictor, sam_model_registry
        device = "cuda" if torch.cuda.is_available() else "cpu"
        sam = sam_model_registry["vit_b"](checkpoint=str(checkpoint)).to(device)
        predictor = SamPredictor(sam)
        rgb = cv2.cvtColor(topdown, cv2.COLOR_GRAY2RGB)
        ctx = torch.autocast("cuda", dtype=torch.float16) if device == "cuda" else __import__("contextlib").nullcontext()
        with ctx: predictor.set_image(rgb)
        masks, scores = [], []
        for box, point in zip(boxes, points):
            with ctx:
                mm, sc, _ = predictor.predict(point_coords=np.asarray([point], np.float32), point_labels=np.ones(1, np.int32), box=np.asarray([box], np.float32), multimask_output=True)
            k = int(np.argmax(sc))
            m = ndimage.binary_fill_holes(mm[k]) & (free > 0)
            masks.append(m)
            scores.append(float(sc[k]))
        del sam
        if device == "cuda": torch.cuda.empty_cache()
        return masks, scores, f"sam_vit_b_{device}"
    except Exception as exc:
        print(f"  SAM unavailable ({type(exc).__name__}: {exc}); using geometric seeds")
        return [seeds == i for i in range(1, int(seeds.max())+1)], [0.0]*len(boxes), f"seed_fallback_{type(exc).__name__}"


def partition_rooms(free: np.ndarray, barriers: np.ndarray, seeds: np.ndarray, proposals: list[np.ndarray], ppm: float) -> np.ndarray:
    walkable = (free > 0) & (barriers == 0)
    marker = np.zeros_like(seeds, np.int32)
    for i in range(1, int(seeds.max())+1):
        core = seeds == i
        if i-1 < len(proposals) and (core & proposals[i-1]).any():
            core &= proposals[i-1]
        marker[core] = i
    if marker.max() == 0:
        marker = seeds.copy()
    _, (iy, ix) = ndimage.distance_transform_edt(marker == 0, return_indices=True)
    labels = np.where(walkable, marker[iy, ix], 0)

    # Multiple clearance maxima may live in one open room. At every Voronoi
    # border, measure the widest clearance connecting its two basins. Split at
    # the narrow mode (doorways); merge at the wide mode (unobstructed room).
    clearance = ndimage.distance_transform_edt(walkable) / ppm
    pair_values: dict[tuple[int, int], float] = {}
    for a, b, ca, cb in ((labels[:-1], labels[1:], clearance[:-1], clearance[1:]),
                         (labels[:, :-1], labels[:, 1:], clearance[:, :-1], clearance[:, 1:])):
        diff = (a > 0) & (b > 0) & (a != b)
        for x, y, v in zip(a[diff], b[diff], np.maximum(ca[diff], cb[diff])):
            key = tuple(sorted((int(x), int(y))))
            pair_values[key] = max(pair_values.get(key, 0.0), float(v))
    vals = np.asarray(list(pair_values.values()), float)
    if len(vals) >= 2 and vals.max() > vals.min() + 1e-6:
        # Otsu threshold on measured bottleneck widths, no capture-specific room count.
        hist, edges = np.histogram(vals, bins=min(24, max(4, len(vals))))
        prob = hist / max(hist.sum(), 1)
        centres = (edges[:-1] + edges[1:]) / 2
        omega = np.cumsum(prob); mu = np.cumsum(prob*centres); total = mu[-1]
        between = (total*omega-mu)**2 / np.maximum(omega*(1-omega), 1e-12)
        threshold = float(centres[int(np.nanargmax(between[:-1]))])
    else:
        threshold = float(vals[0]) if len(vals) else float("inf")
    parent = np.arange(int(labels.max())+1)
    def find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]; x = int(parent[x])
        return x
    for (a,b), width in sorted(pair_values.items()):
        if width <= threshold:
            continue
        ra,rb=find(a),find(b)
        if ra != rb: parent[max(ra,rb)] = min(ra,rb)
    remapped = np.zeros_like(labels)
    roots: dict[int,int] = {}
    for i in range(1,int(labels.max())+1):
        root=find(i); roots.setdefault(root,len(roots)+1); remapped[labels==i]=roots[root]

    # Reclaim the finite wall stroke to the nearest side so adjoining polygons
    # meet at a centreline rather than leaving a raster-width void.
    _, (ry, rx) = ndimage.distance_transform_edt(remapped == 0, return_indices=True)
    reclaim = (free > 0) & (remapped == 0)
    remapped[reclaim] = remapped[ry[reclaim], rx[reclaim]]
    out = np.zeros_like(remapped); next_id = 1
    areas = [(int((remapped==i).sum()), i) for i in range(1,int(remapped.max())+1)]
    for area, i in sorted(areas, reverse=True):
        if area < 1.0*ppm*ppm: continue
        out[remapped==i]=next_id; next_id+=1
    return out


def polygonize(labels: np.ndarray, lo: np.ndarray, ppm: float, faces: list[dict]) -> list[dict]:
    rooms = []
    for rid in range(1, int(labels.max())+1):
        m = (labels == rid).astype(np.uint8)
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts: continue
        cnt = max(cnts, key=cv2.contourArea)
        approx = cv2.approxPolyDP(cnt, max(3.0, 0.09*ppm), True).reshape(-1, 2).astype(float)
        poly = approx/ppm + lo
        # Snap only to nearby measured finite faces; no global Manhattan forcing.
        for j, p in enumerate(poly):
            best = None
            for f in faces:
                normal, tangent = f["normal"], f["tangent"]
                d = float(p @ normal - f["pos"])
                along = float(p @ tangent)
                if abs(d) <= 0.14 and f["a"]-0.2 <= along <= f["b"]+0.2:
                    if best is None or abs(d) < abs(best[0]): best = (d, normal)
            if best is not None: poly[j] = p - best[0]*best[1]
        # Collapse near duplicates and shallow contour noise.
        cleaned = []
        for p in poly:
            if not cleaned or np.linalg.norm(p-cleaned[-1]) > 0.08: cleaned.append(p)
        if len(cleaned) < 3: continue
        shp = Polygon(cleaned).buffer(0)
        if shp.is_empty: continue
        if shp.geom_type == "MultiPolygon": shp = max(shp.geoms, key=lambda q: q.area)
        # Pick the strongest simplification that preserves area within 1.5%.
        base_area = shp.area
        best = shp
        for eps in (0.12, 0.16, 0.20, 0.25):
            candidate = shp.simplify(eps, preserve_topology=True)
            if candidate.is_empty or candidate.geom_type != "Polygon":
                continue
            if abs(candidate.area - base_area) / max(base_area, 1e-9) <= 0.015:
                best = candidate
        shp = best
        ring = np.round(np.asarray(shp.exterior.coords[:-1], float), 4)
        # Rounding to 0.1 mm for the contract can reintroduce self-touching
        # rings, so validity is re-checked on the exact coordinates emitted.
        if not Polygon(ring).is_valid:
            repaired = Polygon(ring).buffer(0)
            if repaired.is_empty:
                continue
            if repaired.geom_type == "MultiPolygon":
                repaired = max(repaired.geoms, key=lambda q: q.area)
            if repaired.geom_type != "Polygon":
                continue
            ring = np.round(np.asarray(repaired.exterior.coords[:-1], float), 4)
        if not Polygon(ring).is_valid or Polygon(ring).area <= 0.5:
            continue
        if Polygon(ring).exterior.is_ccw if hasattr(Polygon(ring), "exterior") else False:
            ring = ring[::-1]
        rooms.append({"polygon": ring, "area": float(Polygon(ring).area)})
    rooms.sort(key=lambda r: -r["area"])
    return rooms


def assemble_plan(cap: Capture, state: dict[str, np.ndarray], rooms: list[dict], families: list[float], wall_bodies: list[dict], sam_scores: list[float], sam_mode: str, elapsed: float) -> dict:
    floor_h, ceil_h = float(state["floor_h"]), float(state["ceil_h"])
    height = ceil_h-floor_h
    height_half = max(0.015, 0.5*np.std((state["P"] @ state["up"])[state["cls"] == 2]))
    out_rooms = []
    for i, room in enumerate(rooms, 1):
        poly = room["polygon"]
        walls = []
        perimeter_sigma = []
        for j in range(len(poly)):
            a, b = poly[j], poly[(j+1)%len(poly)]
            length = float(np.linalg.norm(b-a))
            half = max(0.02, 0.012*length)
            perimeter_sigma.append(half)
            walls.append({"id": f"room_{i:02d}_wall_{j+1:02d}", "start_m": np.round(a,4).tolist(), "end_m": np.round(b,4).tolist(),
                          "length": measurement(length, half, "m", "depth_pose_polygon_edge"),
                          "height": measurement(height, height_half, "m", "floor_ceiling_surface_peaks")})
        area_half = max(0.08, 0.5*sum(perimeter_sigma)*math.sqrt(max(room["area"], 0.1)))
        out_rooms.append({"id": f"room_{i:02d}", "label": f"Room {i}", "polygon_m": np.round(poly,4).tolist(),
                          "floor_area": measurement(room["area"], area_half, "m2", "sam_partition_polygon"),
                          "ceiling_height": measurement(height, height_half, "m", "floor_ceiling_surface_peaks"),
                          "walls": walls, "openings": [], "segmentation_confidence": round(sam_scores[i-1],3) if i-1 < len(sam_scores) else None})
    shapes = [Polygon(r["polygon"]) for r in rooms]
    merged = unary_union(shapes)
    if merged.geom_type == "MultiPolygon": footprint = max(merged.geoms, key=lambda p:p.area)
    else: footprint = merged
    outline = np.asarray(footprint.exterior.coords[:-1]) if not footprint.is_empty else np.empty((0,2))
    adjacency = []
    for i in range(len(shapes)):
        for j in range(i+1, len(shapes)):
            shared = shapes[i].boundary.intersection(shapes[j].boundary).length
            distance = shapes[i].distance(shapes[j])
            # Room polygons terminate on opposite wall faces; adjacency therefore
            # spans a measured wall thickness rather than requiring polygons to touch.
            if shared > 0.18 or distance < 0.42:
                adjacency.append({"room_a": f"room_{i+1:02d}", "room_b": f"room_{j+1:02d}",
                                  "shared_boundary_m": round(float(shared),3), "separation_m": round(float(distance),3)})
    wall_json = []
    opening_json = []
    for wi, body in enumerate(wall_bodies, 1):
        normal, tangent = body["normal"], body["tangent"]
        half = 0.5 * body["thickness"]
        a, b, pos = body["a"], body["b"], body["pos"]
        quad = np.array([a*tangent+(pos-half)*normal, b*tangent+(pos-half)*normal,
                         b*tangent+(pos+half)*normal, a*tangent+(pos+half)*normal])
        wid = f"wall_body_{wi:03d}"
        wall_json.append({"id": wid, "polygon_m": np.round(quad,4).tolist(),
                          "thickness": measurement(body["thickness"], max(0.015,0.2*body["thickness"]), "m", "paired_wall_face_separation"),
                          "length": measurement(b-a, max(0.025,0.01*(b-a)), "m", "paired_wall_face_extent")})
        for oi, op in enumerate(body.get("openings", []), 1):
            centre = 0.5*(op["a"]+op["b"])*tangent + pos*normal
            opening_json.append({"id": f"{wid}_opening_{oi:02d}", "type": "door_or_passage", "wall_body_id": wid,
                                 "center_m": np.round(centre,4).tolist(), "direction": np.round(tangent,6).tolist(),
                                 "width": measurement(op["width_m"], max(0.025,0.03*op["width_m"]), "m", "gap_between_paired_wall_runs"),
                                 "classification_confidence": 0.55})
    return {
        "schema_version": SCHEMA_VERSION,
        "capture": {"id": cap.root.name, "tier": "lidar", "input_hash": hash_capture(cap.root)},
        "coordinate_frame": {"axes": "metric_plan_xy", "unit": "m", "up_world": np.round(state["up"],6).tolist()},
        "levels": {"floor_elevation": measurement(floor_h, 0.015, "m", "horizontal_surface_peak"),
                   "ceiling_elevation": measurement(ceil_h, height_half, "m", "horizontal_surface_peak"),
                   "ceiling_height": measurement(height, height_half+0.015, "m", "difference_of_surface_peaks")},
        "wall_direction_families_deg": [round(v,2) for v in families],
        "wall_bodies": wall_json,
        "openings": opening_json,
        "rooms": out_rooms,
        "adjacency": adjacency,
        "stitched_plan": {"footprint_polygon_m": np.round(outline,4).tolist(),
                          "footprint_area": measurement(float(merged.area), max(0.15,0.03*merged.area), "m2", "union_of_room_polygons")},
        "assumptions": ["Depth and odometry are metrically calibrated by the capture app", "SAM proposes room identity; measured wall evidence constrains boundaries"],
        "warnings": ["Opening gaps are measured; door/window typing remains provisional until lintel/sill evidence is added"],
        "provenance": {"pipeline": "cozmo-floorplan-independent", "version": SCHEMA_VERSION, "sam_mode": sam_mode,
                       "sampled_frame_step": int(state["frame_step"]), "runtime_s": round(elapsed,2)},
    }


def render_plan(plan: dict, path: Path) -> None:
    rooms = plan["rooms"]
    points = np.vstack([np.asarray(r["polygon_m"]) for r in rooms])
    # Rotate presentation only so the longest direction is horizontal; metric JSON stays untouched.
    cov = np.cov(points.T); _, vec = np.linalg.eigh(cov); axis = vec[:, -1]
    angle = -math.atan2(axis[1], axis[0]); c,s=math.cos(angle),math.sin(angle); R=np.array([[c,-s],[s,c]])
    transformed = [np.asarray(r["polygon_m"]) @ R.T for r in rooms]
    allp = np.vstack(transformed); lo,hi=allp.min(0),allp.max(0); span=hi-lo
    fig, ax = plt.subplots(figsize=(12, max(7.5, 12*span[1]/max(span[0],1))), dpi=180)
    for i,(r,p) in enumerate(zip(rooms,transformed)):
        ax.add_patch(PlotPolygon(p, closed=True, facecolor=PALETTE[i%len(PALETTE)], edgecolor="#94a3b8", linewidth=.8, zorder=1))
    # Physical walls are drawn as solid poché over room fills.
    for body in plan.get("wall_bodies", []):
        q = np.asarray(body["polygon_m"]) @ R.T
        ax.add_patch(PlotPolygon(q, closed=True, facecolor="#334155", edgecolor="#0f172a", linewidth=.45, zorder=3))
    # Measured gaps erase the poché. Typing stays provisional in JSON.
    for op in plan.get("openings", []):
        centre = np.asarray(op["center_m"]) @ R.T
        direction = np.asarray(op["direction"]) @ R.T
        width = op["width"]["value"]
        normal = np.array([-direction[1], direction[0]])
        depth = .42
        q = np.array([centre-direction*width/2-normal*depth/2, centre+direction*width/2-normal*depth/2,
                      centre+direction*width/2+normal*depth/2, centre-direction*width/2+normal*depth/2])
        ax.add_patch(PlotPolygon(q, closed=True, facecolor="white", edgecolor="none", zorder=4))
        ax.plot([centre[0]-direction[0]*width/2, centre[0]+direction[0]*width/2],
                [centre[1]-direction[1]*width/2, centre[1]+direction[1]*width/2],
                color="#0891b2", linewidth=1.2, zorder=5)
    occupied_labels: list[np.ndarray] = []
    for i,(r,p) in enumerate(zip(rooms,transformed)):
        centre=np.asarray(Polygon(p).representative_point().coords[0])
        area=r["floor_area"]; h=r["ceiling_height"]
        ax.text(*centre, f"{r['label']}\n{area['value']:.1f} m²\nh {h['value']:.2f} m", ha="center", va="center", fontsize=8.5,
                bbox=dict(boxstyle="round,pad=0.32", fc="white", ec="#cbd5e1", alpha=.92), zorder=5)
        candidates=[]
        for w in r["walls"]:
            a=np.asarray(w["start_m"])@R.T; b=np.asarray(w["end_m"])@R.T; L=w["length"]
            if L["value"] < 1.15: continue
            mid=(a+b)/2; v=b-a; n=np.array([-v[1],v[0]])/(np.linalg.norm(v)+1e-9)
            if np.dot(n,centre-mid)>0:n=-n
            rot=math.degrees(math.atan2(v[1],v[0])); rot=rot+180 if rot>90 or rot<-90 else rot
            candidates.append((L["value"],mid+.15*n,rot))
        for length,pos,rot in sorted(candidates,reverse=True):
            if any(np.linalg.norm(pos-old)<.62 for old in occupied_labels):
                continue
            occupied_labels.append(pos)
            ax.text(*pos, f"{length:.2f} m", fontsize=5.7, color="#475569", ha="center", va="center", rotation=rot, rotation_mode="anchor", zorder=6,
                    bbox=dict(fc="white",ec="none",alpha=.78,pad=.08))
    pad=max(.8,.07*max(span)); ax.set_xlim(lo[0]-pad,hi[0]+pad); ax.set_ylim(lo[1]-pad-0.7,hi[1]+pad)
    ax.set_aspect("equal"); ax.axis("off")
    y=lo[1]-pad+.25; ax.plot([lo[0],lo[0]+1],[y,y],color="#111827",lw=4); ax.text(lo[0]+.5,y+.12,"1 m",ha="center",fontsize=8)
    ax.annotate("N",xy=(hi[0]+pad*.55,hi[1]-.1),xytext=(hi[0]+pad*.55,hi[1]-1.0),arrowprops=dict(arrowstyle="-|>",lw=1.4),ha="center",fontsize=9)
    total=plan["stitched_plan"]["footprint_area"]["value"]
    ax.set_title(f"Ground Floor Plan\n{plan['capture']['id']} · LiDAR · {len(rooms)} spaces · {total:.1f} m²",loc="left",fontsize=15,color="#111827",pad=18)
    fig.text(.09,.025,"Measured dimensions include 95% intervals in plan.json · solid poché: paired physical wall faces · cyan: provisional opening",fontsize=7,color="#64748b")
    fig.savefig(path,bbox_inches="tight",facecolor="white"); plt.close(fig)


def diagnostic(topdown: np.ndarray, labels: np.ndarray, path: Path) -> None:
    vis=cv2.cvtColor(topdown,cv2.COLOR_GRAY2BGR)
    colors=np.array([[219,234,254],[220,252,231],[254,243,199],[243,232,255],[254,226,226],[207,250,254]],np.uint8)
    for i in range(1,int(labels.max())+1):
        m=labels==i; vis[m]=(0.35*vis[m]+0.65*colors[(i-1)%len(colors)]).astype(np.uint8)
        cnt,_=cv2.findContours(m.astype(np.uint8),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE); cv2.drawContours(vis,cnt,-1,(25,35,50),2)
    cv2.imwrite(str(path),vis)


def run(args: argparse.Namespace) -> None:
    started=time.time(); root=Path(args.input).resolve(); out=Path(args.out).resolve(); out.mkdir(parents=True,exist_ok=True)
    cap=load_capture(root); cache=out/"metric_state.npz"
    if args.state:
        source=Path(args.state); print(f"loading supplied metric state: {source}"); loaded=np.load(source); state={k:loaded[k] for k in loaded.files}
        if "trajectory" not in state: state["trajectory"]=np.stack([cap.positions@state["e1"],cap.positions@state["e2"]],axis=1)
        if "frame_step" not in state: state["frame_step"]=np.int64(-1)
        if "floor_h" not in state or "ceil_h" not in state:
            h=state["P"]@state["up"]; state["floor_h"]=np.median(h[state["cls"]==1]); state["ceil_h"]=np.median(h[state["cls"]==2])
    elif cache.exists() and not args.no_cache:
        print(f"loading cache: {cache}"); loaded=np.load(cache); state={k:loaded[k] for k in loaded.files}
    else:
        print(f"fusing {len(cap.frames)} depth/pose frames..."); state=fuse_capture(cap,args.target_frames,args.target_points,drift=getattr(args,"drift",True)); np.savez_compressed(cache,**state)
    print(f"metric points: {len(state['P']):,}; floor={float(state['floor_h']):.3f} ceiling={float(state['ceil_h']):.3f}")
    xy=state["xy"]; lo=xy.min(0)-.08; hi=xy.max(0)+.08; W=int(math.ceil((hi[0]-lo[0])*SAM_PPM)); H=int(math.ceil((hi[1]-lo[1])*SAM_PPM)); shape=(H,W)
    density=occupancy(xy,lo,shape,SAM_PPM); topdown=(255-255*np.log1p(density)/max(np.log1p(density.max()),1e-6)).astype(np.uint8)
    faces,families=wall_faces(state)
    barriers, wall_bodies = physical_wall_barriers(faces,lo,shape,SAM_PPM)
    support=indoor_support(state,lo,shape,SAM_PPM)
    components=room_components(support,barriers,SAM_PPM)
    n=int(components.max())
    print(f"structural evidence: {len(faces)} finite faces -> {len(wall_bodies)} paired wall bodies -> {n} room candidates")
    ckpt=Path(args.sam_checkpoint); proposals,scores,sam_mode=sam_refine(topdown,support,components,ckpt)
    labels, scores, sam_candidates = select_sam_rooms(components,proposals,scores,SAM_PPM)
    rooms=polygonize(labels,lo,SAM_PPM,faces)
    if not rooms: raise RuntimeError("no valid rooms reconstructed")
    elapsed=time.time()-started; plan=assemble_plan(cap,state,rooms,families,wall_bodies,scores,sam_mode,elapsed)
    drift_summary = {"on": bool(getattr(args, "drift", True)),
                     "yaw_deg": float(state.get("drift_yaw_deg", 0.0)),
                     "floor_offset_m": float(state.get("drift_floor_m", 0.0)),
                     "applicable": bool(state.get("drift_applicable", False)),
                     "ablation": None}
    plan["drift"] = drift_summary
    if getattr(args, "ablate", False):
        # Drift accountability: rebuild the same capture with correction off and
        # report the ON/OFF delta on the stitched footprint.
        off_state = fuse_capture(cap, args.target_frames, args.target_points, drift=False)
        off_lo = off_state["xy"].min(0) - .08
        off_hi = off_state["xy"].max(0) + .08
        off_shape = (int(math.ceil((off_hi[1]-off_lo[1])*SAM_PPM)), int(math.ceil((off_hi[0]-off_lo[0])*SAM_PPM)))
        off_faces, _ = wall_faces(off_state)
        off_bar, _ = physical_wall_barriers(off_faces, off_lo, off_shape, SAM_PPM)
        off_support = indoor_support(off_state, off_lo, off_shape, SAM_PPM)
        off_comp = room_components(off_support, off_bar, SAM_PPM)
        off_top = (255-255*np.log1p(occupancy(off_state["xy"], off_lo, off_shape, SAM_PPM)) /
                   max(np.log1p(occupancy(off_state["xy"], off_lo, off_shape, SAM_PPM).max()), 1e-6)).astype(np.uint8)
        off_p, off_s, _ = sam_refine(off_top, off_support, off_comp, ckpt)
        off_lab, _, _ = select_sam_rooms(off_comp, off_p, off_s, SAM_PPM)
        off_rooms = polygonize(off_lab, off_lo, SAM_PPM, off_faces)
        off_area = sum(r["area"] for r in off_rooms)
        on_area = sum(r["area"] for r in rooms)
        drift_summary["ablation"] = {
            "rooms_with_correction": len(rooms), "rooms_without_correction": len(off_rooms),
            "area_with_correction_m2": round(on_area, 3), "area_without_correction_m2": round(off_area, 3),
            "area_delta_m2": round(on_area - off_area, 3),
            "area_delta_pct": round(100.0 * (on_area - off_area) / max(off_area, 1e-9), 2),
            "faces_with": len(faces), "faces_without": len(off_faces),
        }
        print(f"drift ablation: {drift_summary['ablation']}")
    (out/"plan.json").write_text(json.dumps(plan,indent=2)+"\n")
    render_plan(plan,out/"plan.png"); diagnostic(topdown,labels,out/"room_evidence.png")
    results={"experiment":"final_floorplan_cli_lidar","input":str(root),"n_metric_points":len(state["P"]),"n_wall_faces":len(faces),"n_paired_wall_bodies":len(wall_bodies),"n_room_candidates":n,"n_rooms":len(rooms),"areas_m2":[round(r["area"],2) for r in rooms],"sam_mode":sam_mode,"sam_candidates":sam_candidates,"runtime_s":round(time.time()-started,2),"artifacts":["plan.json","plan.png","room_evidence.png"]}
    (out/"results.json").write_text(json.dumps(results,indent=2)+"\n")
    print(json.dumps(results,indent=2)); print(f"plan: {out/'plan.png'}")


def main() -> None:
    p=argparse.ArgumentParser(description="Generate a measured floor plan from a Stray Scanner capture")
    p.add_argument("input",help="Stray Scanner folder")
    p.add_argument("--out",default="final_floorplan/run",help="output directory")
    p.add_argument("--sam-checkpoint",default=str(Path(__file__).resolve().parents[1]/"tmp/models/sam_vit_b_01ec64.pth"))
    p.add_argument("--state",help="optional compatible metric-state .npz for deterministic replay")
    p.add_argument("--no-cache",action="store_true")
    p.add_argument("--target-frames",type=int,default=1250)
    p.add_argument("--target-points",type=int,default=4_500_000)
    args=p.parse_args()
    run(args)


if __name__=="__main__": main()
