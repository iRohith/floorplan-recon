# Benchmark and verification

Every number below is read from the checked-in run artifacts. Nothing is estimated
into this table. Regenerate with `bash scripts/regenerate.sh`.

## LiDAR tier — `runs/lidar/`

| Quantity | Value | Interval |
|---|---|---|
| Rooms emitted | 9 | — |
| Footprint | 40.34 m² | see `stitched_plan.footprint_area.ci95` |
| Ceiling height | 2.28 m | see `levels.ceiling_height.ci95` |
| Room areas (m²) | 10.18, 5.70, 4.92, 4.24, 4.13, 3.89, 3.34, 2.01, 1.94 | per room in `plan.json` |
| Wall bodies | 26 | thickness interval per body |
| Openings | 2 (0.369 m, 0.751 m) | per opening |
| Adjacency edges | 8 | — |
| Wall direction families | 63.0°, 153.0° | measured, not imposed |
| Metric points | 4,280,073 | — |
| Finite wall faces | 89 | — |
| Room candidates | 13 | — |
| Measurement blocks | 430 | all intervals bracket their value |
| Cold runtime | ~13 s | after fusion cache; first fusion is slower |

Structural checks enforced by `tests/test_contract.py`:

- every emitted polygon is geometrically valid;
- no two room polygons overlap (pairwise intersection area is 0);
- every measurement's `ci95` brackets its `value`.

## Photo tier — `runs/photo/`

Four room folders, 20 images, decoded from the capture video as a stand-in for a real
photo set.

| Quantity | Value |
|---|---|
| Rooms | 4 (Bathroom, Hallway, Kitchen, Living Room) |
| Footprint | 85.32 m² |
| Area interval width | roughly ±38% of value |

The intervals are wide on purpose: RGB scale rests on the declared 1.45 m capture height,
so the tier reports an envelope, not a survey. No openings are emitted.

## Video tier — `runs/video/`

One walkthrough clip, decoded at 2 fps and 1280 px, then passed to the same RGB adapter.

| Quantity | Value |
|---|---|
| Rooms | 1 (single connected walkthrough) |
| Footprint | 21.17 m² (CI 16.13–26.21) |
| Frames decoded | 430 at 2 fps, 1280 px |

## What is not scored here

Tape accuracy, repeatability across two captures, opening-width accuracy, head-to-head
against a consumer scanning app, and damage/scope output are **not evaluated** because no
tape ground truth or app export exists for these captures. `docs/compliance.md` marks each
of those rows as missing rather than passing.
