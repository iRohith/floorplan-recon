# floorplan-recon

One command turns handheld iPhone captures into the same confidence-bearing floor-plan contract across three tiers.

```bash
uv sync
uv run floorplan CAPTURE --tier lidar --out runs/new
uv run floorplan PHOTO_ROOT --tier photo --out runs/photos
uv run floorplan walkthrough.mp4 --tier video --out runs/video
```

## Inputs

- **LiDAR:** stock Stray Scanner export containing `depth/`, `confidence/`, `odometry.csv`, `camera_matrix.csv`, and `rgb.mp4`.
- **Photos:** one subfolder per room, 2–8 JPEG/PNG images per folder.
- **Video:** one iPhone walkthrough video. Frames are decoded deterministically before entering the RGB adapter.

## What is implemented

LiDAR is the validated vertical slice: metric depth fusion, gravity and horizontal-surface estimation, true-angle wall-face families, physical wall pairing, SAM room validation, confidence intervals, JSON, PNG, and evidence render. A cold run from the 9,745-frame sample completes locally and the checked-in result is under `runs/lidar/`.

Photo and video use the same output shape but are **experimental**. They provide a runnable thin-input path with wide intervals and explicit warnings; their metric accuracy has not been benchmarked against tape truth. They do not fabricate openings or damage.

## Outputs

- `plan.json`: rooms, walls, areas, ceiling height, wall bodies, openings, adjacency, intervals, warnings, provenance.
- `plan.png`: dimensioned LiDAR plan.
- `room_evidence.png`: segmentation evidence.
- `results.json`: run summary.
- `metric_state.npz`: reusable LiDAR fusion cache (not committed).

## Models

SAM ViT-B is optional. Place `sam_vit_b_01ec64.pth` at `tmp/models/` or pass `--sam-checkpoint`. Without it, the LiDAR path declares a geometric fallback in provenance.

## Known limits

RGB geometry is not walk-in ready. LiDAR opening typing is provisional. Damage, concealed-damage flags, scope generation, repeatability scoring, drift ablation, and tape-ground-truth benchmark data remain incomplete; see `docs/compliance.md` for the exact status.
