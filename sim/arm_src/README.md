# MuJoCo handoff: kinematics checked, physical simulation blocked

`so101_import_test.urdf` was loaded in MuJoCo 3.14.0 and all 28 canonical link-pose comparisons passed. The importer silently inferred masses; these were rejected as physical robot properties. No dynamics steps were accepted or reported as physical verification.

`so101.xml` preserves the six-joint structural tree and visuals. It deliberately sets `inertiafromgeom="false"` and contains no fabricated inertials. MuJoCo must reject loading it until measured/supported link mass, COM and inertia are entered. Visual geoms have contacts disabled; validated convex contact proxies remain outstanding. This file is a structural template, not a runnable physical model.

The exact CAD collision meshes and raw intentional shaft/horn overlaps are retained in the ROS package. A 0.1 mm convex decomposition trial became impractically fragmented and was abandoned without accepting geometry. Do not replace each concave print with one convex hull. Damping, friction, armature, contact properties and actuator dynamics are unknown; future values must be separately labelled tuning assumptions.

Run `../../solidworks/tools/test_mujoco_urdf_import.py` from the project Python environment for the reproducible forward-kinematic inspection. Evidence: `../../verification/mujoco_urdf_import.json`.
