# Reproduction bundle

Everything needed to rebuild every reported number from raw captures.

| Component | Source | Notes |
|---|---|---|
| LiDAR capture | `data/single_scan_with_ceiling/` | Stray Scanner export: depth, confidence, odometry, intrinsics, RGB |
| LiDAR second capture | `data/single_scan_floor_only/` | reserved for the repeatability pair |
| Photo input | `runs/photo_input/<room>/` | 2–8 images per room folder |
| Video input | `data/single_scan_with_ceiling/rgb.mp4` | single walkthrough clip |
| SAM ViT-B | `tmp/models/sam_vit_b_01ec64.pth` | optional; geometry fallback is declared in provenance |
| Python env | `pyproject.toml` | `uv sync` on a clean machine |

## Rebuild

```bash
bash scripts/regenerate.sh
```

Each tier is one command. The LiDAR path fuses raw depth and poses on first run and
writes `metric_state.npz`, which replays deterministically on later runs. RGB tiers
decode deterministically and do not depend on cached model output for their scale.

## Environment variables

- `PY` — interpreter to use (default `python`).
- `DATA` — capture root (default `../data`).
