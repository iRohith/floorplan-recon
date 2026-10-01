# Compliance status

Status is evidence-based: **done**, **experimental**, or **missing**.

| Requirement | Evidence | Status |
|---|---|---|
| One CLI, all three tiers | `floorplan/cli.py` | done |
| LiDAR cold path from raw depth/poses | `floorplan/lidar.py`, `runs/lidar/` | done |
| Photo folders, 2–8 stills/room | `floorplan/image_tier.py` | experimental; runnable, accuracy unverified |
| Video walkthrough | ffmpeg adapter in `floorplan/cli.py` | experimental; runnable, accuracy unverified |
| Same JSON measurement form | `schema/plan.schema.json`, tier emitters | partial; LiDAR richer than RGB |
| Per-room walls, ceiling, area | `rooms[]` in emitted plans | done; RGB ceiling is disclosed prior |
| Openings and widths | LiDAR paired-run gaps | partial; typing provisional, accuracy unverified |
| Stitched adjacency/no overlaps | LiDAR `adjacency[]` | partial; RGB stitching is not validated |
| 95% interval on measurements | `value`, `ci95`, `unit`, `method` | done for emitted dimensions |
| Rendered plan | `runs/lidar/plan.png` | done for LiDAR |
| Drift correction and ON/OFF ablation | experiment log documents rigid ICP failure | missing from product CLI |
| Damage regions and metric extent | none | missing |
| Concealed-damage rules | none | missing |
| Scope line items | none | missing |
| Repeatability benchmark | no submitted tape truth | missing |
| Head-to-head consumer app | no export in this repo | missing |
| Fix-loop before/after | `docs/fix_loop.md` | partial, geometry fix measured without gate truth |
| Capture protocol/device matrix | `docs/capture.md` | done as protocol; accuracy claims withheld |
| Raw benchmark data and truth | excluded from repo | missing |

The repository does not claim that experimental or missing rows pass the case-study gates.
