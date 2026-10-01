# Submission overview

## What this is

`floorplan-recon` turns handheld iPhone capture into a measured, stitched floor plan.
One command per capture, one output contract, three input tiers.

```bash
uv run floorplan <capture> --tier {lidar|video|photo} --out runs/<name>
```

## Status at a glance

1. The **LiDAR tier runs cold** from a raw Stray Scanner export: depth, confidence, odometry, intrinsics, RGB. No precomputed state is required.
2. The LiDAR plan is **metric**: measured floor and ceiling elevations, true-angle wall families, paired wall bodies, room areas, ceilings, openings and adjacency, every quantity carrying a 95% interval and a method string.
3. Room proposals come from **measured wall evidence first**, with a segmentation model used only to confirm or reject candidates and to add confidence. Mask geometry never replaces measured boundaries, because overlapping masks were observed corrupting areas.
4. The **photo and video tiers run** through the same command surface and the same JSON contract. Their scale comes from the declared capture height, so their intervals are wide by construction and they emit no openings rather than guessing.
5. **Nothing is claimed that was not measured.** Rows we could not evaluate — tape accuracy, repeatability, head-to-head, damage, concealed damage, scope — are marked missing in `docs/compliance.md` instead of being filled with plausible numbers.

## Deliverables

| # | Deliverable | Path |
|---|---|---|
| 1 | Compliance matrix | `docs/compliance.md` |
| 2 | Capture protocol + device matrix | `docs/capture.md` |
| 3 | Repo, one command per capture | `README.md`, `floorplan/cli.py` |
| 4 | Reproduction bundle | `docs/reproduction.md`, `scripts/regenerate.sh` |
| 5 | Benchmark / verification numbers | `docs/benchmark.md` |
| 6 | Fix loop | `docs/fix_loop.md` |
| 7 | Technical report | `docs/technical_report.md` |
| 8 | Experiment record | `docs/experiments.md` |

Checked-in run artifacts: `runs/lidar/`, `runs/photo/`, `runs/video/`.

## How the LiDAR path works

Poses and depth are fused into a metric point cloud with per-point normals from exact
pinhole derivatives. Gravity comes from the walking trajectory, so the plan is upright
without assuming the capture's world frame. Horizontal surfaces are classified by
normal alignment to gravity and split into floor and ceiling by height.

Walls are found as **finite faces**: peaks along a measured wall-normal direction, with
an occupied run along the tangent, gated on spanning structural height. Wall directions
are clustered from the data into measured families rather than snapped to world axes.
Faces are then **paired** when two parallel faces sit a plausible wall thickness apart
and overlap along their run. Only paired bodies divide indoor support. That single rule
is what stopped furniture from creating rooms.

Connected components of remaining indoor support become room candidates. Each candidate
is offered to the segmentation model as a room-scale box; the returned score is recorded
as segmentation confidence, and clipping is accepted only when it retains most of the
measured component. Contours are snapped to nearby measured faces, repaired, and
simplified under an explicit area-preservation bound.

## Honest limitations

- No tape ground truth exists for these captures, so **no accuracy gate is claimed as passed**.
- Opening widths are measured from gaps between paired wall runs; door/window typing is provisional and unvalidated.
- Poses are consumed as recorded. A two-submap ICP test did not reduce wall smear, so drift remains unresolved and the ablation is not shipped.
- Exterior walls observed from one side only can be missing from the wall rendering even when the room polygon is correct.
- Damage, concealed-damage rules and scope line items are not implemented.
