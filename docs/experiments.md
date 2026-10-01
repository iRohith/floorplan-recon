# Experiment record

This is a compact record of the actual development sequence. Raw scripts remain in the parent workspace; accepted product code was rewritten into this repository.

| Stage | Question | Result |
|---|---|---|
| Depth fusion | Are Stray depth, quaternion poses and intrinsics metrically consistent? | Yes. Floor normal aligned with gravity and depth is millimetres. |
| Rigid drift correction | Can two-submap ICP repair the loop? | No. Median cross-half distance barely moved; distributed drift needs pose-graph or plane anchors. |
| Vertical planes | Can furniture-band clutter be separated from wall support? | Ceiling-reaching finite wall faces produce a cleaner skeleton than 2D morphology. |
| FloorSAM | Do point prompts recover rooms? | No. Point prompts generated tiny patches. Room-scale boxes from physical walls scored about 0.90–0.98. |
| Physical wall pairing | Should every vertical face divide rooms? | No. Pairing parallel faces at plausible wall thickness reduced clutter-driven splits and produced room-scale components. |
| SAM geometry | Can SAM masks directly define metric rooms? | No. Masks overlapped by up to several square metres. SAM validates candidates; paired-wall components retain metric boundaries. |
| Vectorization | How much simplification is safe? | Adaptive simplification up to 0.25 m with no more than 1.5% area change reduces contour noise. |
| Cold run | Does the product start from raw capture rather than cached state? | Yes. The checked-in run fused 9,745 depth/pose frames, then emitted plan JSON and PNG. |
| RGB tiers | Can the mandatory command surface exist without false precision? | Yes, experimentally: photo/video emit the shared measurement shape with widened intervals and explicit warnings. Accuracy remains unverified. |

## Accepted LiDAR run

The checked-in run reports seven candidates before model filtering and produces a measured multi-room plan from raw Stray input. Surface classification measured floor near -1.50 m and ceiling near 0.92 m. Exact numbers and candidate scores are in `runs/lidar/results.json` and `runs/lidar/plan.json`.

## Papers and public methods consulted

- FloorSAM: promptable segmentation on floor-plan BEV imagery.
- Magri and Fusiello: J-Linkage / preference-set plane fitting.
- Segment Anything: box-prompted mask proposals.
- Standard pinhole unprojection, plane fitting, connected components and confidence propagation.

No third-party submission source or prose is included in this repository.
