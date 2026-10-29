# scene_db

Scene database for manipulation, in ROS 2 Jazzy. It keeps the objects of the
scene with their grasps and relations, persistently, and answers queries
such as "which blue cups are free to pick?" or "in which order do I remove
the parts to reach this one?".

The package contains three nodes:

| Node | Role |
|---|---|
| `scene_db` | The database: a SQLite file (single source of truth) exposed through services and topics |
| `scene_updater` | Runs one acquisition on request: object detection and grasp detection in parallel on the same images, grasp-to-object matching, data association, one atomic `UpdateScene` |
| `mock_grasp_server` | Geometric stand-in for AnyGrasp with the same interface (`grasp_detection_msgs/DetectGrasps`) |

```
 camera ──▶ scene_updater ──(DetectObjects)──▶ yolo_rgbd_detector
  (color,       │  ▲    └───(DetectGrasps)───▶ AnyGrasp / mock_grasp_server
 aligned depth, │  │                             (in parallel, same images)
 camera_info)   │  └ QueryObjects ┐
                └── UpdateScene ──┴──▶ scene_db ──▶ ~/objects, ~/events, ~/markers
                                         ▲  SQLite: objects, grasps, relations
   orchestrator / user / CLI ────────────┘  queries, removal, properties, relations
```

## Main ideas

- **Acquisition on demand.** Nothing changes until someone calls
  `scene_updater/acquire`. AnyGrasp is called only when asked
  (`detect_grasps: true`).
- **Objects persist.** An object that is not seen (occluded, out of view, the
  camera moved) stays in the database. Objects are removed only with
  `remove_object`.
- **Atomic updates.** Each acquisition is one SQLite transaction; every change
  increments `scene_version`, returned by every service.
- **Grasps belong to objects.** A grasp is stored with the object whose box
  (expanded by 1.5 cm) contains it; a moved object loses its grasps unless new
  ones come with the same acquisition.
- **Relations in two families.** *Geometric* ones (`on_top_of`, `inside`,
  `near`, `left_of`, `right_of`) are computed by `scene_db` from the boxes.
  *Mechanical* ones (`screwed_to`, `bracketed_to`, `closed_by`, `held_by`, ...)
  come from outside (CAD / kit description: `a_priori`; scene graph network:
  `inferred`) and have a state (`tightened`, `loosened`, `removed`). A priori
  relations win over inferred ones; low-confidence inferred relations need a
  second observation to be confirmed.
- **Consistent removal.** Removing an object deletes its grasps and relations
  (cascade), recomputes the neighbors' relations and marks the objects that
  were resting on it as `to_verify`.

## Installation

```bash
sudo apt install python3-scipy ros-jazzy-cv-bridge ros-jazzy-message-filters ros-jazzy-tf2-ros
cd ~/ros2_ws
colcon build --packages-up-to scene_db yolo_rgbd_detector
source install/setup.bash
```

Interfaces: [`scene_msgs`](../scene_msgs),
[`object_detection_msgs`](../object_detection_msgs),
`grasp_detection_msgs` (in `manipulation/`).

## Quick start

```bash
# 1. Camera (any RGB-D camera with depth aligned to color) and TF to "world"
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
ros2 run tf2_ros static_transform_publisher --z 1.0 --pitch 0.8 \
    --frame-id world --child-frame-id camera_link

# 2. Detector service
source ~/venvs/yolo/bin/activate
ros2 launch yolo_rgbd_detector yolo_rgbd_detector.launch.py

# 3. Database, updater, mock grasp server
ros2 launch scene_db scene_db.launch.py camera:=camera

# 4. Use it
ros2 run scene_db cli acquire --grasps
ros2 run scene_db cli list
ros2 run scene_db cli query class=cup color=blue free_only=true
ros2 run scene_db cli q free_objects
```

With Gazebo instead of a camera: `ros2 launch yolo_rgbd_sim sim.launch.py`
(see [`yolo_rgbd_sim`](../yolo_rgbd_sim)).

## Interface summary

`scene_db` node (all responses carry `scene_version`):

| Name | Type | Description |
|---|---|---|
| `~/update_scene` | `scene_msgs/UpdateScene` | Write an acquisition (normally called by `scene_updater`) |
| `~/query_objects` | `scene_msgs/QueryObjects` | Filters: `id`, `class`, `color`, `state`, `region`, `near` + `radius`, `max_age`, `min_confidence`, `min_grasp_score`, `free_only`, `property.<key>`, `include_grasps`, `max_results` |
| `~/query_relations` | `scene_msgs/QueryRelations` | Filters: `object_id`, `other_id`, `object_a`, `object_b`, `type`, `family`, `state`, `source`, `min_confidence`, `confirmed` |
| `~/query` | `scene_msgs/Query` | `free_objects`, `blockers_of`, `supported_by`, `disassembly_order` (JSON results) |
| `~/remove_object` | `scene_msgs/RemoveObject` | Remove an object (only way to remove objects) |
| `~/set_properties`, `~/remove_properties` | `scene_msgs/SetProperties`, `RemoveProperties` | User properties |
| `~/add_relation`, `~/remove_relation`, `~/set_relation_state` | `scene_msgs/...` | Mechanical relations |
| `~/clear` | `std_srvs/Trigger` | Remove everything |
| `~/objects` | topic `scene_msgs/SceneObjectArray` (latched) | Whole content after each change |
| `~/events` | topic `scene_msgs/SceneEvent` | One message per change |
| `~/markers` | topic `visualization_msgs/MarkerArray` (latched) | Boxes, ids, grasp arrows |

`scene_updater`: service `~/acquire` (`scene_msgs/Acquire`).
`mock_grasp_server`: service `~/detect_grasps` (`grasp_detection_msgs/DetectGrasps`).

Command line client: `ros2 run scene_db cli --help`.
Graphical interface: [`rqt_scene_db`](../rqt_scene_db)
(`ros2 run rqt_scene_db rqt_scene_db`).

## Inspecting the database

The SQLite file (default `~/.ros/scene_db.sqlite`) can be read with any tool:

```bash
sqlite3 ~/.ros/scene_db.sqlite "SELECT id, class_name, color, px, py, pz, state FROM objects"
sqlite3 ~/.ros/scene_db.sqlite "SELECT object_a, type, object_b, state FROM relations"
```

Write only through the services: they keep relations and versions consistent.

## Open points (interfaces ready, implementation outside this package)

- **Scene graph network**: sends inferred mechanical relations through
  `UpdateScene.relations` (with `#<n>` references to the objects of the same
  request) or `add_relation` with `source: inferred`.
- **CAD / kit import**: sends a priori relations with `add_relation`
  (`source: a_priori`).
- **Orchestrator**: uses the queries and `remove_object`; it is downstream and
  out of scope.

## Testing

```bash
colcon test --packages-select scene_db --event-handlers console_direct+
```

Unit tests cover the SQLite store (transactions, cascades, persistence,
source policy), the relation logic (supports, blocking, disassembly order,
cycles), association, grasp matching and the grasp planner. They need
neither ROS nodes nor a camera.

## Documentation

- [docs/API.md](docs/API.md): services, topics, parameters, database schema, rules
- [docs/TUTORIAL.md](docs/TUTORIAL.md): step-by-step usage, Python clients

## License

Apache License 2.0, see [LICENSE](LICENSE).
