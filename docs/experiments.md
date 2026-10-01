# Experiment record

Intent, method, finding, and what it changed. Negative results are kept because they
removed whole branches of the pipeline.

## 1. Depth fusion and pose convention

**Intent.** Establish that a Stray Scanner export is metrically usable before building
anything on it.

**Method.** Back-project confidence-filtered depth with per-frame intrinsics, rotate by the
camera quaternion, and transform to world. Fit a plane to the lowest horizontal band.

**Finding.** Depth is millimetres, `point_world = R(q) · point_cam + t`, and depth runs
along `+z` in the camera frame — confirmed by a RANSAC floor returning a normal of
`(0.005, 1.000, 0.000)` with a large inlier share. Trajectory `y`-span is centimetres while
`x`,`z` span metres, so `y` is up. **Kept.**

## 2. Drift correction by two-submap rigid ICP — negative

**Intent.** Repair accumulated pose drift so walls stop smearing.

**Method.** Fuse the first and second halves of the trajectory separately, then align them
with 40 iterations of point-to-point ICP and compare wall smear before and after.

**Finding.** The correction was 8.9 cm and 1.0°, but mean cross-half distance moved from
0.336 m to 0.339 m and wall smear did not improve. **Rigid ICP cannot fix this drift — it
is distributed along the path, not one offset.** The intended replacement is a submap pose
graph with floor/wall plane constraints. Not shipped; recorded as a known gate failure.

## 3. Occupancy lattices and flood fill — negative

**Intent.** Partition free space into rooms from a 2D occupancy grid.

**Method.** Threshold occupancy, take connected free components as rooms, then find
openings at cell boundaries.

**Finding.** Two structural failures. First, a bathroom came out at 62.8 m². Second, and
worse, connected free components can never share an edge, so the boundary-pair lookup that
opened doors was **dead code — zero openings by construction**, not by evidence. Flood fill
also cannot survive 0.5–1.5 m of drift smear. **Branch abandoned**; 3D-native wall evidence
replaced it.

## 4. Vertical-plane RANSAC walls

**Intent.** Find real wall planes in 3D instead of relying on a blurred raster.

**Method.** RANSAC restricted to near-vertical planes inside a band above furniture height,
with inliers split at tangent gaps and merged only on genuine overlap.

**Finding.** Produced wall segments with honest 0.18–0.28 m slab thickness — a direct
measure of smear — and a measured dominant direction about 28° off the capture axes, which
proved the building is not aligned to the device frame. Early versions produced phantom
10–14 m walls by spanning collinear fragments across open space, which is why the
gap-splitting rule exists. **Kept as concept; replaced by finite face extraction.**

## 5. Instance grounding (IGGT) — degenerate on this capture

**Intent.** Use a pretrained instance-grounded reconstruction model to obtain wall identity
directly, rather than inventing geometry.

**Method.** Ran on Modal against 48 keyframes of the multi-room capture. Required import
shims only; model code untouched. The chunked head paths in the release are broken, so
heads were driven unchunked and only the cross-frame aggregator was chunked.

**Finding.** The run completed — 2053/2053 checkpoint keys, ~16 min — but the instance
layer is degenerate: one cluster owns 98.6% of pixels and the rest are 0.15–0.44%. Feeding
that into the plan builder produced **zero walls and zero rooms**. A CPU HDBSCAN over the
feature volume also exceeded the function timeout, so clustering moved to GPU. **Conclusion:
instance identity cannot be the wall source on this data.** This is the experiment that
decided the architecture — geometry must come from measured surfaces, with models used
where they are reliable.

## 6. Open-vocabulary object matching — usable but phrasing-sensitive

**Intent.** Type rooms from their contents instead of guessing.

**Method.** Region proposals, then forced-choice rather than 22-way open vocabulary;
open-vocabulary negation scoring was tested and discarded because it is negation-blind
(probabilities clustered near 0.5 on all prompts).

**Finding.** Forced choice inside a confusable group is sharp where open vocabulary is
noise. Room typing then needed a second fix: 2D centroids of masks scatter across rooms, so
claims are projected to 3D with depth and pose and assigned to the room containing the 3D
point. Look-through contamination (a hallway camera seeing a bathroom through an open door)
was measurably reduced by that quarantine. **Kept, but not on the critical path for
geometry.**

## 7. FloorSAM

**Intent.** Use promptable segmentation on the top-down BEV of the ceiling slab, which is
the cleanest indoor evidence, to get room regions.

**Method.** Ceiling-slab density image, peak prompts, mask proposals, two-stage mask
filtering, contours with vertex snapping to measured lines.

**Finding.** With **room-scale boxes** the model scored 0.89–0.97 and produced sensible
rooms. With **point-scale prompts** it produced dozens of tiny 0.2–0.7 m² patches — proven
useless. Directly adopting mask geometry was also rejected: candidate masks overlapped by up
to 4.88 m², which corrupts floor areas. **Conclusion: the model proposes and scores; measured
wall components own the boundary.**

## 8. J-Linkage multi-plane fitting

**Intent.** Replace greedy RANSAC with a globally consistent multi-plane wall extraction.

**Method.** Preference-set J-Linkage over wall-band points.

**Finding.** Cleaner plane sets than RANSAC, and it confirmed the two measured families.
Infinite fitted lines were unusable as room dividers, however — only **bounded face
segments** worked as prompts and dividers. **Kept as evidence, not as the divider.**

## 9. Finite wall faces and physical pairing — the accepted core

**Intent.** Get dividers that correspond to real walls and nothing else.

**Method.** Within each measured direction family, find peaks along the family normal and
take finite occupied runs along the tangent. Gate a face on spanning structural height.
Then pair two parallel faces only when their separation is 0.06–0.35 m and their runs
overlap by at least 0.35 m. Merge collinear bodies across doorway-scale gaps.

**Finding.** Using every height-spanning face as a divider produced **19 components**,
including furniture-driven cells. Requiring paired faces produced **8 room-scale
components** with plausible areas from about 3.9 to 17.4 m². This single rule is the
difference between a readable plan and a fragmented one. **Adopted.**

## 10. Openings

**Intent.** Report doors and windows with measured widths.

**Method.** Gaps between paired wall runs, bounded to doorway scale, with material across
the gap to reject scan holes.

**Finding.** Doors are found as measured gaps; window typing needs sill and lintel evidence
that this capture does not provide. The current product therefore emits measured **widths**
with a provisional type and says so, rather than asserting door versus window.

## 11. Vector simplification

**Intent.** Remove contour noise without falsifying area.

**Method.** Compare simplification tolerances and measure the induced area change.

**Finding.** Adaptive simplification up to 0.25 m changes area by no more than 1.5%, which
was adopted as the explicit bound. Vertex snapping to measured faces, however, can produce
self-touching rings, and rounding to the emitted precision can reintroduce them — the
emitter now re-validates the exact coordinates it writes, and that fix removed the invalid
polygons found during verification.

## 12. Outcome

Accepted into the product: exact pinhole unprojection, measured gravity, true-angle face
families, structural height gating, physical face pairing, room-scale model validation with
confidence, area-preserving vectorization, and confidence intervals on every dimension.

Rejected after measurement: rigid ICP drift correction, occupancy flood fill, instance
identity as wall source, point-prompt segmentation, direct use of mask geometry as room
boundaries, and open-vocabulary negation scoring.

Still open: plane-anchored drift correction with an ON/OFF ablation, tape-truth benchmarking
and repeatability scoring, opening typing with lintel/sill evidence, an honest RGB scale
estimate, damage detection, and scope generation.
