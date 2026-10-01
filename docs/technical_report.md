# Technical report

## System

`floorplan` exposes one command and one measurement vocabulary for LiDAR, video and photo captures. The LiDAR path is the substantive implementation: it back-projects confidence-filtered depth with per-frame intrinsics and camera poses, estimates gravity from the trajectory, classifies surfaces from gravity-aligned normals, and creates a metric plan frame. Horizontal-surface peaks yield floor and ceiling. Wall-normal azimuths are clustered into measured direction families rather than forced to world axes.

Finite wall faces come from offset histograms and occupied tangent runs. A face must span structural height. Opposite faces are paired at plausible wall thickness; only paired wall bodies divide indoor support. This prevents sofas and cabinets from becoming walls. Connected components provide room-scale boxes to SAM ViT-B. SAM supplies a proposal score and conservative clipping, while the non-overlapping metric components remain the geometry source. Contours are snapped to nearby measured faces and simplified under an area-preservation bound.

## Uncertainty

Every emitted physical quantity is a `value`, `ci95`, `unit`, and `method`. LiDAR intervals combine surface dispersion and edge-length-dependent uncertainty. RGB intervals are deliberately much wider because scale comes from declared camera height and generic room-envelope evidence. An unobserved RGB ceiling is labeled as a prior rather than a measurement.

## Thin-input tiers

Video is deterministically decoded with ffmpeg and passed to the RGB adapter. Photo input supports one folder per room. The current RGB implementation uses image-line evidence and a camera-height scale prior; it is an executable contract path, not a validated photogrammetry solution. No opening is emitted when its width cannot be defended. This avoids confident garbage but does not satisfy the walk-in accuracy gates.

## Drift

A two-submap ICP experiment did not improve wall smear, demonstrating that handheld drift is distributed. The product currently consumes capture poses as recorded; this is a known automatic failure against the specified drift-accountability gate. The intended next stage is a plane-anchored submap pose graph with an ON/OFF ablation.

## Error budget and failures

Depth quantization is smaller than pose drift and room-boundary ambiguity. Mirrors, glass, low light and furniture reduce face support. Physical pairing misses exterior walls when only the interior face is visible. SAM can overlap or under-segment, so it is prevented from defining metric boundaries directly. RGB scale dominates image-tier error. Damage, concealed-damage rules and scope generation are not implemented.

## Fix loop

The worst observed geometry defect was over-segmentation from single vertical faces. Requiring physically paired faces reduced 19 components to 8 room-scale candidates and produced a readable seven-space replay plan. The full before/after statement and limitations are in `docs/fix_loop.md`.

## Reproduction

Install with `uv sync`. Run `uv run floorplan CAPTURE --tier lidar --out runs/new`. The first run fuses raw frames and writes `metric_state.npz`; later runs replay that deterministic metric state. Models are local and optional. Checked-in run artifacts record exact candidate scores and provenance. Benchmark claims are limited to observed experiment outputs because tape ground truth, repeated captures and consumer-app exports are absent.
