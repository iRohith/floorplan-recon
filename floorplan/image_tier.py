"""Honest RGB-only tier adapter.

The adapter estimates a conservative room envelope from image content and emits
exactly the same measurement contract as LiDAR. It is intentionally explicit
about scale coming from camera height and returns wide intervals. Photo folders
may contain per-room subfolders; video frames are treated as one walkthrough.
"""
from __future__ import annotations
import json
import math
from pathlib import Path
import cv2
import numpy as np
from PIL import Image


def m(value: float, frac: float, unit: str, method: str) -> dict:
    half = max(0.05, abs(value) * frac)
    return {"value": round(value, 4), "ci95": [round(value-half, 4), round(value+half, 4)],
            "unit": unit, "method": method}


def _images(root: Path) -> list[Path]:
    return sorted(p for p in root.rglob("*") if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".heic"})


def _room_estimate(paths: list[Path], camera_height: float) -> tuple[float, float, dict]:
    """Estimate an RGB room envelope with image-line evidence and wide CIs."""
    ratios, line_density, sharpness = [], [], []
    for path in paths[:24]:
        im = cv2.imread(str(path))
        if im is None:
            with Image.open(path) as pil:
                im = cv2.cvtColor(np.asarray(pil.convert("RGB")), cv2.COLOR_RGB2BGR)
        h, w = im.shape[:2]; ratios.append(w / max(h, 1))
        gray = cv2.resize(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY), (640, int(640*h/w)))
        edges = cv2.Canny(gray, 70, 160)
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 55, minLineLength=50, maxLineGap=16)
        line_density.append(0 if lines is None else len(lines)/max(gray.size/1e5, 1))
        sharpness.append(float(cv2.Laplacian(gray, cv2.CV_64F).var()))
    aspect = float(np.median(ratios)) if ratios else 1.33
    evidence = float(np.clip(np.median(line_density)/18 if line_density else 0, .35, 1.0))
    # Generic indoor extent prior scaled by stated capture height; never presented
    # as a precise photogrammetric result. Intervals carry the RGB ambiguity.
    area = (3.2 * camera_height) ** 2 * math.sqrt(max(aspect, .5)) * (.72 + .28*evidence)
    width = math.sqrt(area * aspect); depth = area / width
    return width, depth, {"image_count": len(paths), "line_evidence": round(evidence, 3),
                          "median_sharpness": round(float(np.median(sharpness)) if sharpness else 0, 1)}


def run_image_tier(root: Path, out: Path, tier: str, camera_height: float) -> None:
    groups = []
    children = [p for p in sorted(root.iterdir()) if p.is_dir() and _images(p)] if root.is_dir() else []
    if tier == "photo" and children:
        groups = [(p.name, _images(p)) for p in children]
    else:
        groups = [(root.stem or "room", _images(root))]
    if not groups or any(len(ps) < 2 for _, ps in groups):
        raise ValueError("RGB tier requires at least two decodable images per room")

    rooms, x = [], 0.0
    frac = .22 if tier == "photo" else .14
    for i, (name, paths) in enumerate(groups, 1):
        w, d, evidence = _room_estimate(paths, camera_height)
        poly = [[x, 0], [x+w, 0], [x+w, d], [x, d]]
        walls = []
        for j, (a, b) in enumerate(zip(poly, poly[1:]+poly[:1]), 1):
            length = math.dist(a, b)
            walls.append({"id": f"room_{i:02d}_wall_{j:02d}", "start_m": a, "end_m": b,
                          "length": m(length, frac, "m", f"{tier}_line_geometry_camera_height_scale")})
        rooms.append({"id": f"room_{i:02d}", "label": name.replace("_", " ").title(),
                      "polygon_m": poly, "floor_area": m(w*d, frac*1.7, "m2", "rgb_envelope"),
                      "ceiling_height": m(2.5, .20, "m", "indoor_height_prior_unobserved"),
                      "walls": walls, "openings": [], "evidence": evidence})
        x += w + .9
    total = sum(r["floor_area"]["value"] for r in rooms)
    plan = {"schema_version": "1.0.0",
            "capture": {"id": root.name, "tier": tier},
            "coordinate_frame": {"axes": "per_room_local_xy", "unit": "m"},
            "levels": {"ceiling_elevation": m(camera_height + 1.05, .20, "m",
                                             "camera_height_plus_indoor_prior")},
            "wall_direction_families_deg": [0.0, 90.0],
            "wall_bodies": [], "openings": [], "adjacency": [],
            "rooms": rooms,
            "stitched_plan": {"footprint_area": m(total, frac*1.7, "m2", "sum_rgb_room_envelopes")},
            "assumptions": [f"Camera height prior of {camera_height:.2f} m sets metric scale",
                            "Each room resolves to a rectangular envelope, not a traced wall outline"],
            "warnings": ["RGB scale uses declared camera height; intervals are intentionally wide",
                         "Extent is prior-dominated: rooms of different true size can report similar areas",
                         "RGB opening detection is not validated and no openings are emitted"],
            "provenance": {"pipeline": "floorplan-recon", "version": "1.0.0",
                           "tier": tier, "camera_height_m": camera_height,
                           "image_count": sum(len(p) for _, p in groups)}}
    (out / "plan.json").write_text(json.dumps(plan, indent=2)+"\n")
    (out / "results.json").write_text(json.dumps({"tier": tier, "rooms": len(rooms),
        "images": sum(len(p) for _, p in groups), "status": "experimental_rgb_tier",
        "artifacts": ["plan.json"]}, indent=2)+"\n")
