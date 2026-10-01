# Fix loop: room fragmentation

## Before

Using every ceiling-reaching vertical face as a divider produced 19 components, including furniture-driven cells. This was the worst observed geometry failure because it made a homeowner-readable stitched plan impossible.

## Hypothesis

A real wall usually presents two approximately parallel faces separated by physical wall thickness. Furniture and incomplete observations often provide only one face. Requiring paired faces should suppress false dividers without assuming a room count.

## Shipped fix

`physical_wall_barriers` pairs finite faces only when separation is 0.06–0.35 m and their finite runs overlap by at least 0.35 m. Nearly collinear paired bodies are merged across measured doorway-scale gaps. SAM then validates room-scale components but does not replace metric boundaries.

## After

The paired-wall experiment reduced 19 raw components to 8 room-scale candidates; the accepted replay emitted 7 spaces. The cold path also runs end-to-end. This is meaningful movement, but it is **not a gate pass** because no tape-truth room partition exists for this capture.

## Remaining risk

Some outer walls expose only one observed face and are omitted from wall poché. Opening classification lacks lintel/sill validation. A future fix must add plane-anchored drift correction and evaluate paired captures against tape truth.
