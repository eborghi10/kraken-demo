# Row perception

*[Docs index](index.md) · [Navigation](navigation.md) · [Control](control.md)*

Inside a row the useful structure is a corridor, not a map. The trunks either
side are the only thing that constrains the machine and the lidar sees them
directly, so `kraken_orchard` measures the aisle it is standing in rather than
looking itself up in a map.

Following that needs no estimate of where the robot is in the world, only what
is beside it — which is why it survives the localisation drift that a costmap
search does not. **This measures a row; it does not find one.** Choosing which
row is next is [mission planning](mission-planning.md).

Available only on the O3DE side. The headless model has
[no ranging sensor](simulation.md#3-which-one-to-use).

---

## 1. From cloud to trunks

`cloud_filter` reduces a `PointCloud2` in stages, and reports what survived each
one so a bad fit can be blamed on the right stage — `received`, `above_ground`,
`voxels`, `clusters`, `trunks`:

1. **Ground removal** on a 0.5 m grid, keeping returns in a height band
   (0.40–1.80 m by default). Two thirds of the cloud is ground.
2. **Range gate**, then **voxel downsample** at 0.10 m.
3. **Euclidean clustering** at 0.35 m.
4. **Cluster acceptance** by width and extent: too wide is canopy or a wall, too
   thin is noise.

The defaults describe a collider-based lidar. The tuning actually shipped in
`config/row_follower.yaml` describes the RGL one, and the difference matters:
RGL raycasts **visual** geometry, so what comes back is the canopy surface
rather than a bare trunk. A cluster is then a whole tree, 1.3–1.5 m across, and
`max_trunk_width` goes to 1.80 m to match.

---

## 2. From trunks to a corridor

`fitRow` splits the surviving centres by sign of their lateral coordinate, fits
a line to each side, and reports the corridor between them: `width`,
`lateral_offset`, `row_heading`, and how many trunks each side contributed. The
Kraken's lidar is bolted on without rotation, so the offset and heading read
directly as **the robot's error against the row**.

A fit is refused, with a `reason`, unless it has enough trunks per side
(`min_trunks`), enough longitudinal span (`min_span`), a small enough residual
(`max_residual`) and two sides that agree on heading
(`max_heading_difference`). Only three or four trees per side stay unoccluded,
so the thresholds are looser than a trunk-based fit would need, and the residual
allowance is nearly double because canopy centroids scatter about the row line
far more than trunk centres do.

The result goes out as `kraken_interfaces/msg/RowEstimate`, and the centreline
as a `nav_msgs/Path` so Nav2's controller can drive it through `FollowPath` and
keep its own footprint collision checking. **This replaces the planner inside a
row, not the controller** — what drives the path is still
[`ArcTracker`](control.md).

The detected trunks are republished on `orchard/trunks` for RViz, because a fit
that fails is almost always a filtering problem two stages earlier.

---

## 3. Tests

`colcon test --packages-select kraken_orchard` covers the filter and the fit on
synthetic clouds; neither needs a simulator.
