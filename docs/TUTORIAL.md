# scene_db: tutorial

Part A uses only the database: you write objects by hand and practice
queries, relations and removal, without camera or YOLO. Part B connects the
perception pipeline (camera or Gazebo, detector, grasps). Part C shows how a
program (e.g. an orchestrator) uses the database.

```bash
source ~/ros2_ws/install/setup.bash
```

---

## Part A: the database alone

### A1. Start the database

```bash
ros2 run scene_db scene_db_node --ros-args \
    -p database_file:=/tmp/scene_demo.sqlite -p reset_on_start:=true
```

```
[scene_db]: Scene database '/tmp/scene_demo.sqlite' (frame 'world'): 0 objects, version 0
```

In a second terminal watch the changes:

```bash
ros2 topic echo /scene_db/events --field details
```

### A2. Write a scene

The `scene_updater` normally writes acquisitions; here a short script does it.
Save it as `fill_scene.py`:

```python
import rclpy
from rclpy.node import Node
from scene_msgs.msg import ObjectUpdate, Relation
from scene_msgs.srv import UpdateScene
from vision_msgs.msg import ObjectHypothesisWithPose


def obj(cls, color, center, size):
    u = ObjectUpdate(color=color)
    d = u.detection                      # id empty = new object
    d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.center.position.z = center
    d.bbox.center.orientation.w = 1.0
    d.bbox.size.x, d.bbox.size.y, d.bbox.size.z = size
    h = ObjectHypothesisWithPose()
    h.hypothesis.class_id, h.hypothesis.score = cls, 0.9
    d.results.append(h)
    return u


rclpy.init()
node = Node('fill_scene')
cli = node.create_client(UpdateScene, '/scene_db/update_scene')
cli.wait_for_service()
req = UpdateScene.Request(frame_id='world', objects=[
    obj('box', 'brown', (1.0, 0.0, 0.80), (0.30, 0.20, 0.10)),      # #0, on the table
    obj('lid', 'gray', (1.0, 0.0, 0.86), (0.30, 0.20, 0.02)),       # #1, on the box
    obj('cup', 'blue', (1.0, 0.0, 0.92), (0.08, 0.08, 0.10)),       # #2, on the lid
    obj('cup', 'white', (1.0, 0.25, 0.80), (0.08, 0.08, 0.10)),     # #3, beside
])
# The lid is screwed to the box (known from the kit description).
req.relations.append(Relation(object_a='#1', object_b='#0', type='screwed_to',
                              family='mechanical', source='a_priori',
                              state='tightened', fastener='screw', confidence=1.0))
future = cli.call_async(req)
rclpy.spin_until_future_complete(node, future)
print(future.result())
rclpy.shutdown()
```

```bash
python3 fill_scene.py
```

The response lists the new ids in request order: `box_1, lid_1, cup_1,
cup_2`, `scene_version: 1`. `#1` and `#0` in the relation refer to the
objects of the same request, whose ids are not known in advance.

### A3. Look at it

```bash
ros2 run scene_db cli list
```

```
OK: 4 objects (scene version 1)
  box_1        brown   box          (+1.000, +0.000, +0.800) size (0.30, 0.20, 0.10) conf 0.90 obs 1 ok
  cup_1        blue    cup          (+1.000, +0.000, +0.920) size (0.08, 0.08, 0.10) conf 0.90 obs 1 ok
  cup_2        white   cup          (+1.000, +0.250, +0.800) size (0.08, 0.08, 0.10) conf 0.90 obs 1 ok
  lid_1        gray    lid          (+1.000, +0.000, +0.860) size (0.30, 0.20, 0.02) conf 0.90 obs 1 ok
```

```bash
ros2 run scene_db cli relations
```

```
OK: 21 relations (scene version 1)
  [  1] lid_1 screwed_to box_1 (mechanical, a_priori, tightened, screw, conf 1.00)
  [  2] box_1 near cup_1 (geometric, geometric)
  ...
  [  9] cup_1 on_top_of lid_1 (geometric, geometric)
  [ 10] cup_1 right_of cup_2 (geometric, geometric)
  ...
  [ 20] lid_1 on_top_of box_1 (geometric, geometric)
```

The geometric relations were computed by `scene_db`: `cup_1` rests on the
lid (the highest support below it), not on the box. In RViz (fixed frame
`world`) add `/scene_db/markers` to see boxes and ids.

### A4. Typed queries

```bash
ros2 run scene_db cli query class=cup
ros2 run scene_db cli query class=cup color=white
ros2 run scene_db cli query near=1,0.3,0.8 radius=0.2          # sorted by distance
ros2 run scene_db cli query region=0.5,1.5,-0.5,0.5,0.85,2.0     # above z = 0.85
ros2 run scene_db cli query free_only=true
ros2 run scene_db cli relations object_id=lid_1 family=mechanical
```

A wrong key is an error, not an empty result:

```bash
ros2 run scene_db cli query colour=blue
# ERROR: Unknown filter 'colour'. Allowed: id, class, color, ...
```

### A5. What blocks what

```bash
ros2 run scene_db cli q free_objects
# cup_1 and cup_2: nothing on them, no fastenings

ros2 run scene_db cli q blockers_of object_id=box_1
# {"kind": "object_on_top", "object_id": "lid_1", ...}

ros2 run scene_db cli q blockers_of object_id=lid_1
# {"kind": "object_on_top", "object_id": "cup_1", ...}
# {"kind": "fastening", "object_id": "box_1", "relation_id": 1, "type": "screwed_to", ...}

ros2 run scene_db cli q disassembly_order object_id=box_1
# 1 remove cup_1
# 2 release relation 1 (lid_1 screwed_to box_1)
# 3 remove lid_1
# 4 remove box_1
```

After unscrewing the lid, tell the database:

```bash
ros2 run scene_db cli set-state 1 loosened
ros2 run scene_db cli q blockers_of object_id=lid_1     # only cup_1 now
```

`closed_by` and `held_by` work differently: `part_1 closed_by cover_1` means
the cover must be **removed** before the part, not released.

```bash
ros2 run scene_db cli add-relation cup_2 held_by box_1
ros2 run scene_db cli q disassembly_order object_id=cup_2
# cup_1, release screw, lid_1, box_1, then cup_2
```

### A6. Inferred relations

A scene graph network sends relations with `source: inferred` and a
confidence. Below `inferred_min_confidence` (0.7) they stay unconfirmed until
a second acquisition reports them again:

```bash
ros2 run scene_db cli add-relation cup_1 bracketed_to lid_1 --source inferred --confidence 0.5
ros2 run scene_db cli relations type=bracketed_to
# ... (mechanical, inferred, tightened, conf 0.50, unconfirmed)
```

An inferred relation never overrides an a priori one: try
`add-relation lid_1 screwed_to box_1 --source inferred --state removed`: the
response reports the conflict and the a priori state is kept.

### A7. Properties

```bash
ros2 run scene_db cli set-props cup_2 located="blue box" owner=anna
ros2 run scene_db cli query property.located="blue box"
ros2 run scene_db cli query property.owner=          # any value
ros2 run scene_db cli remove-props cup_2 owner
```

### A8. Removing objects

Objects never disappear by themselves. Remove the box:

```bash
ros2 run scene_db cli remove box_1
# OK: Removed 'box_1'
#   to verify: lid_1
```

The relations of `box_1` (including the screw and `held_by`) are gone; the
lid was resting on it and is now `to_verify` (drawn faded with `(?)` in
RViz): its support disappeared, so its pose should be checked. The next
acquisition that sees the lid sets it back to `ok`.

### A9. Persistence

Stop the node (Ctrl-C) and start it again without `reset_on_start`:

```bash
ros2 run scene_db scene_db_node --ros-args -p database_file:=/tmp/scene_demo.sqlite
```

Everything is still there, with the same version and ids. The file is plain
SQLite:

```bash
sqlite3 /tmp/scene_demo.sqlite "SELECT id, class_name, state FROM objects"
```

---

## Part B: perception

### B1. Start the pipeline

With Gazebo (no hardware):

```bash
ros2 launch yolo_rgbd_sim sim.launch.py
```

With a RealSense:

```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
ros2 run tf2_ros static_transform_publisher --z 1.0 --pitch 0.8 \
    --frame-id world --child-frame-id camera_link
source ~/venvs/yolo/bin/activate
ros2 launch yolo_rgbd_detector yolo_rgbd_detector.launch.py
ros2 launch scene_db scene_db.launch.py camera:=camera
```

`scene_db.launch.py` starts `scene_db`, `scene_updater` and the mock grasp
server.

### B2. First acquisition

```bash
ros2 run scene_db cli acquire --grasps
```

```
OK: 4 detections: 4 added, 0 moved, 0 unchanged; grasps 3/5 assigned (1.9 s) (scene version 1)
  added: cup_1, cup_2, bowl_1, dining_table_1
```

What happened: one frame was taken; the detector and the grasp server ran
in parallel on it; each grasp was assigned to the box containing it (2 fell
outside every box and were discarded); everything was written in one
transaction.

```bash
ros2 run scene_db cli list
ros2 run scene_db cli query class=cup min_grasp_score=0.3
```

### B3. Acquire again

Nothing moved:

```bash
ros2 run scene_db cli acquire
# 0 added, 0 moved, 4 unchanged
```

The observations counter grows, `confidence` is averaged, `last_seen` is
updated; without `--grasps` the stored grasps are kept.

Move an object a few centimeters (in Gazebo: select it, translate mode) and
acquire: it is reported as **moved**, keeps its id and loses its grasps
(they no longer match its pose). Acquire with `--grasps` to get new ones.

Move it far (more than ~15 cm): the association gate rejects the match, the
object gets a **new id** and the old entry stays where it was. Remove the
stale one with `cli remove`. This is deliberate: the database never deletes
objects on its own, because an object out of view or occluded is not gone.

Tune the gates in `config/params.yaml` (`association_distance`,
`move_threshold`, ...).

### B4. Using AnyGrasp

The updater calls any server of `grasp_detection_msgs/DetectGrasps`. Start
the AnyGrasp server and point the updater to it:

```bash
ros2 launch scene_db scene_db.launch.py mock_grasps:=false
ros2 param set /scene_updater grasp_service /anygrasp/detect_grasps   # or in params.yaml
```

(The parameter is read at startup: set it in the YAML file or restart the
node.) Grasps must come back in the color optical frame with the image stamp;
the updater transforms them with TF.

### B5. Grasps from close views (YOLO far, grasps near)

Object detectors and grasp detectors like different distances: YOLO needs
the whole object in a reasonably large view (~1 m), a volumetric grasp
detector such as VGN wants precise depth (~0.5 m) and a small volume. Split
the acquisition in two steps.

Start a `MultiviewDetectGrasps` server, e.g. VGN (see
`vgn_grasp_server`, in its own conda workspace):

```bash
ros2 launch vgn_grasp_server vgn_grasp_server.launch.py
```

**1. Objects from far** (one or more views; no grasps):

```bash
ros2 run scene_db cli acquire          # move the camera, acquire again: same ids
ros2 run scene_db cli list
```

**2. Where is the object?** The orchestrator reads its box:

```bash
ros2 run scene_db cli query id=cup_1 include_grasps=false
#   cup_1  cyan  cup  (+1.130, +0.188, +0.823) size (0.14, 0.13, 0.09) ...
```

**3. Close views.** Move the camera around the object (e.g. 3 views, 0.35 m
away and 0.40 m above, 50° apart) and store each one:

```bash
ros2 run scene_db cli capture --clear  # first view: drop old views
ros2 run scene_db cli capture          # second view
ros2 run scene_db cli capture          # third view
```

The camera pose is taken from TF at the image stamp: on a robot it comes from
the kinematics and the hand-eye calibration.

**4. Grasps of that object:**

```bash
ros2 run scene_db cli acquire-grasps cup_1
# OK: 'cup_1': 7/14 grasps from 3 views (0.14 s) (scene version 5)
#   volume (+1.010, +0.073, +0.728) .. (+1.250, +0.303, +1.028)
```

The views are fused in a volume around the box (5 cm margin, floor 5 cm below
the box, 0.30 m high: the layout VGN was trained on), so the grasp detector
never has to cover the whole table. Grasps inside other boxes belong to those
objects and are dropped (7 of 14 above); pose and statistics of `cup_1` do
not change, only its grasps.

In Gazebo, `yolo_rgbd_sim` has a fixed camera: move it with the
`/world/tabletop/set_pose` Gazebo service and publish the matching
`world -> camera_link` transform. Note that rclpy's
`StaticTransformBroadcaster` ignores updates of a frame it already published:
publish a `tf2_msgs/TFMessage` on `/tf_static` yourself, or use `/tf`.

---

## Part C: a client program

A typical consumer: refresh the scene, pick the best free cup, remove it
from the database after the robot has taken it away.

```python
import json
import time

import rclpy
from rclpy.node import Node
from scene_msgs.msg import KeyValue
from scene_msgs.srv import Acquire, Query, QueryObjects, RemoveObject


class SceneClient(Node):
    def __init__(self):
        super().__init__('scene_client')
        self.acquire = self.create_client(Acquire, '/scene_updater/acquire')
        self.query_objects = self.create_client(QueryObjects, '/scene_db/query_objects')
        self.query = self.create_client(Query, '/scene_db/query')
        self.remove = self.create_client(RemoveObject, '/scene_db/remove_object')

    def call(self, client, request):
        client.wait_for_service()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if not res.success:
            raise RuntimeError(res.message)
        return res


rclpy.init()
c = SceneClient()
c.call(c.acquire, Acquire.Request(detect_grasps=True))

kv = lambda k, v: KeyValue(key=k, value=v)   # noqa: E731
res = c.call(c.query_objects, QueryObjects.Request(filters=[
    kv('class', 'cup'), kv('free_only', 'true'), kv('min_grasp_score', '0.3')]))
if res.objects:
    cup = max(res.objects, key=lambda o: o.grasps[0].grasp.score)
    grasp = cup.grasps[0].grasp           # pose in the fixed frame
    print(f'picking {cup.id} at version {res.scene_version}: {grasp.pose.position}')
    # ... move the robot ...
    c.call(c.remove, RemoveObject.Request(object_id=cup.id))
else:
    blockers = c.call(c.query, Query.Request(query_type='free_objects'))
    print([json.loads(r) for r in blockers.results_json])
rclpy.shutdown()
```

Use `scene_version` to detect concurrent changes: if a later response or a
`SceneEvent` has a higher version, re-read before acting on stale data.

Inside a node with other callbacks (TF, timers) do not block in callbacks:
use a `MultiThreadedExecutor` and a `ReentrantCallbackGroup` for the clients,
as `scene_updater.py` does.

---

## Troubleshooting

| Problem | Cause / fix |
|---|---|
| `acquire`: `No synchronized color/depth/camera_info` | Wrong camera namespace (`camera:=`), depth not aligned (`align_depth.enable:=true`), or the camera is not publishing |
| `acquire`: `Service ... not available` | Detector or grasp server not running; check `detect_service` / `grasp_service` |
| `acquire`: TF error | No transform from the camera to `fixed_frame` at the image stamp |
| `acquire_grasps`: `no stored views` | Call `capture` first (views are cleared after each `acquire-grasps` unless `--keep-views`) |
| `acquire_grasps`: 0 grasps | Views do not see the object, wrong camera poses in TF (check `capture` output), or volume too small (`--margin`) |
| Same object twice with different classes (e.g. `cup_1` and `bowl_1`) | The detector changed its mind between views; the association only merges equal classes. Remove the wrong one |
| `stores objects in frame 'world', but the fixed frame is 'map'` at startup | The file was created with another frame: use another `database_file` or `reset_on_start:=true` |
| Objects duplicated after small moves | Increase `association_distance` / `association_size_factor` |
| Unwanted `moved` events from noise | Increase `move_threshold` |
| Missing `on_top_of` | Footprints do not overlap enough (`on_top_min_overlap`) or the gap exceeds `on_top_max_gap` |
| No grasps from the mock | The largest plane in range is not the support (lower `max_depth`), objects touch each other, or are wider than `max_width` |
