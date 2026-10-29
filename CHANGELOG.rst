^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scene_db
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-10-29)
------------------
* Grasps of one object from close views: ``scene_updater`` services
  ``~/capture_view`` (store frame + camera pose from TF) and
  ``~/acquire_grasps`` (MultiviewDetectGrasps in a volume around the object
  box, keep the grasps in the box, write them with ``set_grasps``).
  New parameters ``multiview_grasp_service``, ``grasp_volume_margin``,
  ``grasp_volume_below``, ``grasp_volume_height``, ``object_grasp_margin``,
  ``max_views``.
* ``scene_db`` service ``~/set_grasps``: replace the grasps of one object
  without changing its pose or statistics.
* ``no_grasp_classes`` parameter of ``scene_updater``: supports (e.g. the
  table) never receive grasps, so their large box does not steal the grasps
  of undetected objects.
* ``association.grasp_volume`` and ``match_grasps(..., eligible=...)``.
* CLI commands ``capture``, ``clear-views``, ``acquire-grasps``.
* Docs: API and tutorial (part B5); CONTRIBUTING.md.

0.2.0 (2026-10-27)
------------------
* SQLite scene database (objects, grasps, geometric and mechanical
  relations, properties) with typed and composite queries, events, markers.
* ``scene_updater``: acquisition with object and grasp detection in parallel,
  grasp-to-object matching, association, atomic update.
* ``mock_grasp_server``: geometric stand-in for AnyGrasp.
* CLI, API reference and tutorial.
