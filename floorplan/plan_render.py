"""Architectural plan renderer.

Draws exactly what plan.json says — walls solid (observed) or dashed
(inferred), openings colored by kind, room cards with area/ceiling intervals,
dimension lines carrying their CIs, scale bar and legend. Building frame:
plans are always drawn upright. No floor-plan noise — this is the product
surface a homeowner should recognise from poly.cam / magicplan.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon as MplPolygon

WALL_LW = 2.6
WALL_LW_INF = 1.8
COL_WALL = "#1a1a1a"
COL_WALL_INF = "#8a8a8a"
COL_DOOR = "#2563eb"
COL_WINDOW = "#059669"
COL_PASS = "#9ca3af"
COL_ROOM_FILL = "#f6f8fa"
COL_CARD_EDGE = "#d0d7de"
COL_DIM = "#57606a"


def _fmt_measure(m: dict | None, unit: str = "") -> str:
    if m is None:
        return "—"
    u = m["unit"] if not unit else unit
    return f"{m['value']:.2f}±{m['ci_half']:.2f} {u}".strip()


def render_plan(
    plan: dict,
    out_path: str | Path,
    title: str | None = None,
    dpi: int = 160,
    annotate_dims: bool = True,
) -> Path:
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(11, 11))
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")

    # --- room fills + cards ---
    for room in plan.get("rooms", []):
        poly = np.array(room["polygon"])
        if len(poly) >= 3:
            ax.add_patch(MplPolygon(poly, closed=True, facecolor=COL_ROOM_FILL,
                                    edgecolor="none", zorder=1))
            c = poly.mean(axis=0)
            card = (
                f"{room['label']}\n"
                f"{_fmt_measure(room.get('area'), 'm²')}\n"
                f"h {_fmt_measure(room.get('ceiling'), 'm')}"
            )
            ax.text(c[0], c[1], card, ha="center", va="center", fontsize=8.5,
                    color="#24292f", zorder=4,
                    bbox=dict(boxstyle="round,pad=0.45", facecolor="white",
                              edgecolor=COL_CARD_EDGE, linewidth=0.8))

    # --- walls ---
    for wall in plan.get("walls", []):
        a, b = np.array(wall["a"]), np.array(wall["b"])
        observed = wall.get("observed", True)
        ax.plot([a[0], b[0]], [a[1], b[1]],
                color=COL_WALL if observed else COL_WALL_INF,
                lw=WALL_LW if observed else WALL_LW_INF,
                solid_capstyle="round",
                linestyle="-" if observed else (0, (5, 4)),
                zorder=5)

    # --- openings ---
    for op in plan.get("openings", []):
        wall = plan["walls"][op["wall"]]
        a, b = np.array(wall["a"]), np.array(wall["b"])
        t = b - a
        L = float(np.linalg.norm(t)) + 1e-12
        t = t / L
        s0 = float(op.get("s_start", 0.0))
        s1 = float(op.get("s_end", 0.5))
        p0, p1 = a + t * s0, a + t * s1
        col = {"door": COL_DOOR, "window": COL_WINDOW}.get(op["kind"], COL_PASS)
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color=col, lw=5.5,
                solid_capstyle="butt", zorder=6)

    # --- dimension lines along walls (labels sit OUTIDE the room) ---
    if annotate_dims:
        for wall in plan.get("walls", []):
            a, b = np.array(wall["a"]), np.array(wall["b"])
            t = b - a
            L = float(np.linalg.norm(t))
            if L < 0.6:
                continue
            t = t / L
            n = np.array([-t[1], t[0]])
            # push label away from the room centroid
            wall_mid = (a + b) / 2
            side = 1.0
            if plan.get("rooms"):
                centroids = np.array([np.mean(r["polygon"], axis=0)
                                      for r in plan["rooms"]])
                room_c = centroids.mean(axis=0)
                side = -1.0 if float((wall_mid - room_c) @ n) < 0 else 1.0
            mid = wall_mid + n * (0.32 * side)
            lm = wall.get("length")
            label = _fmt_measure(lm, "m") if lm else f"{L:.2f} m"
            ang = float(np.degrees(np.arctan2(t[1], t[0])))
            if 90 < ang <= 270 or -270 <= ang < -90:
                ang += 180
            ax.text(mid[0], mid[1], label, ha="center", va="center",
                    fontsize=7.2, color=COL_DIM, rotation=ang, rotation_mode="anchor",
                    zorder=4)

    # --- axes / scale bar ---
    pts = []
    for wall in plan.get("walls", []):
        pts.append(wall["a"])
        pts.append(wall["b"])
    for room in plan.get("rooms", []):
        pts.extend(room["polygon"])
    if pts:
        pts = np.array(pts, dtype=float)
        lo, hi = pts.min(axis=0), pts.max(axis=0)
    else:
        lo, hi = np.array([0.0, 0.0]), np.array([10.0, 10.0])
    pad = 1.2
    ax.set_xlim(lo[0] - pad, hi[0] + pad)
    ax.set_ylim(lo[1] - pad, hi[1] + pad)
    ax.set_aspect("equal")
    ax.axis("off")

    # scale bar: 1 m
    bar_x = lo[0] - pad + 0.6
    bar_y = lo[1] - pad + 0.55
    ax.plot([bar_x, bar_x + 1.0], [bar_y, bar_y], color=COL_WALL, lw=2.2,
            solid_capstyle="butt")
    ax.text(bar_x + 0.5, bar_y + 0.18, "1 m", ha="center", va="bottom",
            fontsize=8.5, color=COL_WALL)

    # north-ish orientation placeholder: building-frame note
    ax.text(hi[0] + pad - 0.4, lo[1] - pad + 0.55, "building frame",
            ha="right", va="center", fontsize=7, color="#8c959f", style="italic")

    # legend
    legend = [
        Line2D([0], [0], color=COL_WALL, lw=WALL_LW, label="wall (observed)"),
        Line2D([0], [0], color=COL_WALL_INF, lw=WALL_LW_INF, linestyle=(0, (5, 4)),
               label="wall (inferred)"),
        Line2D([0], [0], color=COL_DOOR, lw=5.5, label="door"),
        Line2D([0], [0], color=COL_WINDOW, lw=5.5, label="window"),
        Line2D([0], [0], color=COL_PASS, lw=5.5, label="pass-through"),
    ]
    ax.legend(handles=legend, loc="lower right", fontsize=8, frameon=True,
              framealpha=0.95, edgecolor=COL_CARD_EDGE)

    header = title or f"{plan.get('capture','?')} | {plan.get('tier','?')} | scan2plan"
    ax.set_title(header, fontsize=12.5, color="#24292f", pad=14)

    fig.savefig(out_path, dpi=dpi, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return out_path
