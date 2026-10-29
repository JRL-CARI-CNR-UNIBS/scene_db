# scene_db: API reference

Three nodes: `scene_db` (database), `scene_updater` (acquisitions) and
`mock_grasp_server` (grasp detection stand-in). Interfaces are in
[`scene_msgs`](../../scene_msgs).

Conventions:

- All geometry is in the fixed frame `fixed_frame` (default `world`).
- Boxes: `pose` = center and orientation (rotation about z only), `size` =
  extent along the box axes (`x` = longer horizontal side).
- Every response has `success`, `message` and `scene_version` (the version
  after the call; unchanged when the call fails or only reads).
- On failure nothing is written.

---

## Node `scene_db`

### Services

#### `~/update_scene` (`scene_msgs/UpdateScene`)

Writes one acquisition in a single transaction. Normally called by
`scene_updater`; any node can call it.

| Request field | Description |
|---|---|
| `stamp` | Stamp of the images (becomes `last_seen` and the grasp stamp). Zero = now |
| `frame_id` | Must equal `fixed_frame` (empty = assume it) |
| `objects[]` | `ObjectUpdate`, see below |
| `relations[]` | External relations to upsert (mechanical). `object_a` / `object_b` may be `#<n>` = the n-th object of this request |

`ObjectUpdate`:

| Field | Description |
|---|---|
| `detection` | `vision_msgs/Detection3D`: `id` = existing object id, or empty for a new object; `bbox` = box; `results[0].hypothesis` = class and score |
| `moved` | The object was matched but its pose changed: emits `object_moved` and recomputes its relations |
| `color`, `rgb` | Measured color |
| `update_grasps` | Replace the stored grasps of the object with `grasps` (an empty list clears them) |
| `grasps[]` | `grasp_detection_msgs/Grasp` in the fixed frame |

Effects:

- matched objects: pose, size, class, color updated; `observations + 1`;
  `confidence` = running mean of the scores; `last_seen = stamp`;
  `state = ok` (a `to_verify` object seen again becomes `ok`);
- new objects: id `<class>_<n>` (spaces become `_`), `n` never reused;
- geometric relations of added and moved objects recomputed; supports
  (`on_top_of`) recomputed for the whole scene;
- external relations upserted with the source policy (see *Relations*).

Response: `object_ids` (id of each request object, same order),
`changed_object_ids` (added or moved).

Errors: wrong frame, unknown id, invalid relation (unknown object, geometric
family, bad state or source, `#n` out of range).

#### `~/query_objects` (`scene_msgs/QueryObjects`)

`filters`: `KeyValue[]`, combined with AND. Unknown keys are an error.

| Key | Value | Meaning |
|---|---|---|
| `id` | string | Exact id |
| `class` | string | Exact class name |
| `color` | string | Exact color name |
| `state` | `ok` / `to_verify` | |
| `region` | `xmin,xmax,ymin,ymax,zmin,zmax` | Box center inside the region |
| `near` | `x,y,z` | Sort by distance from the point; fills `distances` |
| `radius` | meters | With `near`: maximum distance |
| `max_age` | seconds | Seen in the last `max_age` seconds |
| `min_confidence` | 0–1 | Minimum mean score |
| `min_grasp_score` | 0–1 | At least one grasp with this score |
| `free_only` | `true` / `false` | Nothing blocks the object (see *Blocking*) |
| `property.<key>` | string | Property present; equal to the value if the value is not empty |
| `include_grasps` | `true` / `false` | Default true |
| `max_results` | int | Limit (after sorting) |

Results are sorted by id (natural order: `cup_2` before `cup_10`), or by
distance with `near`.

#### `~/query_relations` (`scene_msgs/QueryRelations`)

| Key | Meaning |
|---|---|
| `object_id` | Relations involving the object (either side) |
| `other_id` | With `object_id`: the other side |
| `object_a`, `object_b` | Exact side |
| `type`, `family`, `state`, `source` | Exact value |
| `min_confidence` | Minimum confidence |
| `confirmed` | `true` / `false` |

#### `~/query` (`scene_msgs/Query`)

Composite queries; each element of `results_json` is one JSON object.

| `query_type` | `params` | Results |
|---|---|---|
| `free_objects` | – | `{"id", "class", "color", "center", "state", "best_grasp_score"}` for every free object |
| `blockers_of` | `object_id` | `{"kind", "object_id", "relation_id", "type", "state", ["confirmed"]}`; `kind` = `object_on_top` (remove `object_id` first), `blocking_object` (remove `object_id` first) or `fastening` (release relation `relation_id` first). Empty = free |
| `supported_by` | `object_id` | `{"id"}` of every object resting on it, directly or through others |
| `disassembly_order` | `object_id` (optional; all objects if missing) | Ordered steps: `{"step", "action": "release", "relation_id", "object_id", "other_id", "type"}` or `{"step", "action": "remove", "object_id"}` |

`disassembly_order` is a depth-first visit of the blocking graph; a cycle is
an error (`Cycle in blocking relations: a -> b -> a`).

#### `~/remove_object` (`scene_msgs/RemoveObject`)

Deletes the object with its grasps, properties and relations (cascade);
objects that were `on_top_of` it become `to_verify` (returned in
`to_verify_ids`); geometric relations of its former neighbors are
recomputed. This is the **only** way objects leave the database.

#### `~/set_properties`, `~/remove_properties`

User properties, free `key: value` strings (`located: blue box`,
`owner: anna`). `set_properties` adds or overwrites; `remove_properties`
ignores missing keys. Keys must not be empty.

#### `~/add_relation` (`scene_msgs/AddRelation`)

Adds or updates a mechanical relation between existing objects. Empty
fields get defaults: `family = mechanical`, `state = tightened`,
`source = a_priori`, `confidence = 1`. Returns `relation_id`.

#### `~/remove_relation`, `~/set_relation_state`

Delete a mechanical relation / change its state to `tightened`, `loosened`
or `removed`. Geometric relations cannot be removed or changed (they follow
the boxes).

#### `~/clear` (`std_srvs/Trigger`)

Removes everything; id counters restart from 1; `scene_version` keeps
increasing.

### Topics

| Topic | Type | QoS | Content |
|---|---|---|---|
| `~/objects` | `scene_msgs/SceneObjectArray` | reliable, transient local, depth 1 | Whole database after every change (and at startup) |
| `~/events` | `scene_msgs/SceneEvent` | reliable, depth 100 | One message per change |
| `~/markers` | `visualization_msgs/MarkerArray` | transient local | `objects` (cubes, measured color, faded when `to_verify`), `labels` (ids, `(?)` when `to_verify`), `grasps/<id>` (arrows along the approach, green = high score, best 5) |

Events:

| `event` | When | `details` |
|---|---|---|
| `object_added` | New object | class |
| `object_moved` | Matched with `moved` | |
| `object_updated` | Grasps replaced (not moved), properties changed, became `to_verify` | what changed |
| `object_removed` | `remove_object` | |
| `relation_changed` | Relation added, removed or changed | e.g. `added cup_1 on_top_of box_1` |
| `scene_cleared` | `clear` | |

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `fixed_frame` | `world` | Frame of the stored geometry. A file created with another frame is rejected at startup |
| `database_file` | `~/.ros/scene_db.sqlite` | SQLite file; `:memory:` = no persistence |
| `reset_on_start` | false | Delete the file at startup |
| `on_top_tolerance` | 0.03 | m, see *Geometric relations* |
| `on_top_max_gap` | 0.15 | m |
| `on_top_min_overlap` | 0.3 | fraction |
| `inside_tolerance` | 0.02 | m |
| `near_distance` | 0.3 | m |
| `lateral_tolerance` | 0.05 | m |
| `left_axis` | `[0, 1, 0]` | Direction of "left" in the fixed frame |
| `inferred_min_confidence` | 0.7 | Inferred relations below it need a second observation |
| `blocking_types` | `[closed_by, held_by]` | Mechanical types whose `object_b` must be removed first |
| `publish_markers` | true | |

---

## Relations

A relation reads **`object_a <type> object_b`**: `cup_1 on_top_of box_1`,
`lid_1 screwed_to box_1`, `part_1 closed_by cover_1`.

### Geometric relations (computed)

Computed from the axis-aligned box enclosing each (rotated) box, source and
family `geometric`, state `active`:

| Type | Rule |
|---|---|
| `on_top_of` | B is the support of A: the highest object below A whose footprint covers ≥ `on_top_min_overlap` of A's footprint, with A's bottom between `on_top_tolerance` below and `on_top_max_gap` above B's top, and A's center above B's center. Objects with a top within `on_top_tolerance` of the highest are all supports |
| `inside` | A horizontally within B (± `inside_tolerance`), A's bottom ≥ B's bottom − tolerance, A's top ≤ B's top, and A smaller than B |
| `near` | Centers closer than `near_distance` (both directions) |
| `left_of` / `right_of` | Near, and the center offset along `left_axis` larger than `lateral_tolerance` |

`on_top_max_gap` is large on purpose: with a single view the box of an object
often covers only its top surface, so its bottom looks higher than it is.

### Mechanical relations (external)

Family `mechanical`, state `tightened` / `loosened` / `removed`, source
`a_priori` or `inferred`, optional `fastener` (`screw`, `bracket`, `clip`, ...).

Source policy when the same `(object_a, object_b, type)` arrives again:

| Stored \ incoming | `a_priori` | `inferred` |
|---|---|---|
| none | inserted, confirmed | inserted; confirmed if `confidence ≥ inferred_min_confidence` |
| `a_priori` | overwritten | only `observations + 1`; a different state is reported as a conflict in `message`, the a priori state is kept |
| `inferred` | overwritten, becomes `a_priori` | updated (max confidence); confirmed when seen in another acquisition (different stamp) or with enough confidence |

### Blocking

What prevents picking X:

1. every Y with `Y on_top_of X` → remove Y first (`object_on_top`);
2. every `X <type> B` in state `tightened`:
   - `type` in `blocking_types` (`closed_by`, `held_by`) → remove B first
     (`blocking_object`);
   - other types (`screwed_to`, `bracketed_to`, ...) → release the fastening
     first, i.e. set its state to `loosened` or `removed` (`fastening`).

Unconfirmed inferred relations block too. X is **free** when nothing blocks it.

---

## Node `scene_updater`

### Service `~/acquire` (`scene_msgs/Acquire`)

| Request | Description |
|---|---|
| `detect_grasps` | Also call the grasp detector |
| `timeout` | Seconds per step (0 = parameter) |

Steps:

1. Wait for the first synchronized color / depth / camera_info received
   **after** the request (stamp `t`).
2. Start `DetectObjects` (target frame = `fixed_frame`) and, if requested,
   `DetectGrasps` in parallel on the same three messages; meanwhile read the
   stored objects (`query_objects`).
3. Transform the grasps to the fixed frame (TF at the grasp header stamp, i.e.
   `t`). Assign each grasp to the box that contains its center, expanded by
   `grasp_margin`; with several candidates the deepest containment wins, then
   the nearest center. Objects whose class is in `no_grasp_classes`
   (supports such as the table) are skipped, so the grasps on the objects
   they contain are not stolen by their large box. Unassigned grasps are
   discarded. Keep the best
   `max_grasps_per_object`.
4. Associate the detections that have a 3D box with the stored objects:
   same class (and color with `match_color`), center distance within
   `max(association_distance, association_size_factor × larger side)`,
   minimum total distance (Hungarian). A match farther than
   `max(move_threshold, move_size_factor × larger side)` is *moved*.
   Unmatched detections are new objects.
5. One `update_scene` call. Grasps: with `detect_grasps` every detected
   object gets the new grasps (possibly none); without it, new and moved
   objects get an empty list (old grasps are invalid), the others keep theirs.

Response: `added_ids`, `moved_ids`, `unchanged_ids`, `num_detections`,
`num_grasps`, `num_grasps_assigned`, `scene_version`.

Stored objects that are not detected are **not** changed.

### Topics

| Topic | Type |
|---|---|
| `color/image_raw` | `sensor_msgs/Image` |
| `depth/image_raw` | `sensor_msgs/Image`, aligned to color |
| `color/camera_info` | `sensor_msgs/CameraInfo` |

(sensor data QoS; the launch file remaps them to `/<camera>/color/image_raw`,
`/<camera>/aligned_depth_to_color/image_raw`, `/<camera>/color/camera_info`.)

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `fixed_frame` | `world` | Target frame of detections and grasps |
| `detect_service` | `/yolo_rgbd_detector/detect` | |
| `grasp_service` | `/mock_grasp_server/detect_grasps` | Set to the AnyGrasp server in the real cell |
| `scene_db` | `/scene_db` | Name of the database node |
| `timeout` | 10.0 | s per step |
| `sync_slop` | 0.05 | s |
| `min_score` | 0.0 | Detection score (0 = detector default) |
| `classes` | `[""]` | Class filter (`[""]` = detector default) |
| `grasp_margin` | 0.015 | m |
| `no_grasp_classes` | `[""]` | Classes that never receive grasps, e.g. `["dining table"]` (`[""]` = none) |
| `max_grasps_per_object` | 10 | |
| `min_grasp_score` | 0.0 | Passed to the grasp detector |
| `association_distance` | 0.15 | m |
| `association_size_factor` | 0.5 | |
| `move_threshold` | 0.03 | m |
| `move_size_factor` | 0.2 | |
| `match_color` | false | |

---

## Node `mock_grasp_server`

Service `~/detect_grasps` (`grasp_detection_msgs/DetectGrasps`), same
contract as the AnyGrasp server: color + aligned depth + camera info in,
grasps in the color optical frame out, sorted by score.

Algorithm: largest plane (RANSAC + least squares) = support; points higher
than `min_height` form blobs (connected components in the image); one
top-down grasp per blob: center on the top surface over the footprint center,
approach (`x`) along the plane normal towards the plane, closing direction
(`y`) across the shorter footprint axis, `width` = footprint width + 2 cm.
Blobs wider than `max_width` are skipped. Score = `1 − (width / max_width)²`,
clipped to [0.05, 1]. Objects touching each other form one blob.

| Parameter | Default | Description |
|---|---|---|
| `min_height` | 0.01 | m above the plane |
| `max_width` | 0.1 | m, gripper opening |
| `min_points` | 100 | per blob |
| `max_depth` | 3.0 | m; limit it so that the support is the largest plane in range |

Request options `use_workspace` (limits in the camera frame), `max_grasps`
and `min_score` are honored.

---

## Database schema

```sql
meta(key TEXT PRIMARY KEY, value TEXT)
  -- schema_version, frame_id, scene_version, counter.<class>
objects(id TEXT PRIMARY KEY, class_name, color, r, g, b,
        px, py, pz, qx, qy, qz, qw, sx, sy, sz,
        state, confidence, observations, first_seen_ns, last_seen_ns, grasps_stamp_ns)
properties(object_id → objects ON DELETE CASCADE, key, value, PRIMARY KEY(object_id, key))
grasps(id INTEGER PRIMARY KEY AUTOINCREMENT, object_id → objects ON DELETE CASCADE,
       px, py, pz, qx, qy, qz, qw, score, width, height, depth, stamp_ns)
relations(id INTEGER PRIMARY KEY AUTOINCREMENT,
          object_a → objects ON DELETE CASCADE, object_b → objects ON DELETE CASCADE,
          type, family, fastener, state, source, confidence, confirmed, observations,
          stamp_ns, UNIQUE(object_a, object_b, type))
-- indices: objects(class_name), grasps(object_id), relations(object_a), relations(object_b)
```

Foreign keys are enabled (`PRAGMA foreign_keys = ON`); the file uses WAL
journaling. Times are nanoseconds.

## Python modules

| Module | Content (ROS-free unless noted) |
|---|---|
| `scene_db.store` | `SceneStore` (SQLite store: `update_scene`, `query_objects`, `query_relations`, `composite_query`, `remove_object`, `add_relation`, ...), records `ObjectRecord`, `ObjectInput`, `GraspRecord`, `Event`, `WriteResult`, `StoreConfig` |
| `scene_db.relations` | `Box`, `RelationRecord`, `GeometricConfig`, `geometric_relations`, `support_relations`, `compute_geometric`, `BlockingModel` (`blockers_of`, `is_free`, `supported_by`, `disassembly_order`), `SceneError` |
| `scene_db.association` | `match_grasps`, `containment_depth`, `associate`, `AssociationConfig`, `Candidate`, `transform_pose`, quaternion helpers |
| `scene_db.grasp_planner` | `plan_grasps`, `fit_plane`, `PlannedGrasp` |
| `scene_db.conversions` | Records ↔ ROS messages (ROS) |
| `scene_db.scene_db_node`, `scene_updater`, `mock_grasp_server`, `cli` | Nodes and command line client (ROS) |

`SceneStore` can be used directly in scripts and tests:

```python
from scene_db.store import SceneStore, ObjectInput
store = SceneStore(':memory:', 'world')
store.update_scene(0, [ObjectInput('', 'cup', 0.9, (1.0, 0.2, 0.8), size=(0.08, 0.08, 0.1))])
print(store.query_objects({'class': 'cup'}, now_ns=0))
```

## Command line client

`ros2 run scene_db cli [--db /scene_db] [--updater /scene_updater] [--timeout 30] COMMAND`

| Command | Example |
|---|---|
| `acquire [--grasps]` | `cli acquire --grasps` |
| `list` | `cli list` |
| `query KEY=VALUE...` | `cli query class=cup near=1,0,0.8 radius=0.3` |
| `relations KEY=VALUE...` | `cli relations object_id=cup_1` |
| `q TYPE [KEY=VALUE...]` | `cli q disassembly_order object_id=part_1` |
| `remove ID` | `cli remove cup_2` |
| `set-props ID KEY=VALUE...` | `cli set-props cup_1 located="blue box"` |
| `remove-props ID KEY...` | `cli remove-props cup_1 located` |
| `add-relation A TYPE B [--state] [--source] [--fastener] [--confidence]` | `cli add-relation lid_1 screwed_to box_1 --fastener screw` |
| `remove-relation RID` | `cli remove-relation 7` |
| `set-state RID STATE` | `cli set-state 7 loosened` |
| `clear` | `cli clear` |

The exit code is 0 on success, 1 on a failed call.

## Launch file

`scene_db.launch.py` arguments: `params_file`, `camera` (namespace,
RealSense layout), `fixed_frame`, `database_file`, `reset`, `mock_grasps`,
`use_sim_time`.
