# SO101 MG996R R3 description

Engineering prototype, not physically tested or payload rated. Generated from the checked SolidWorks reconstruction and measured component transforms.

Run `colcon build --packages-select so101_description`, source the install setup, then `ros2 launch so101_description display.launch.py` on a ROS 2 installation. ROS 2 GUI/build validation is not available in this Windows task environment; standalone Xacro/XML/kinematic validation is separate.

Six revolute joints include the directly driven moving jaw. The base frame retains the source convention. Add a +0.0024 m world-Z placement to match the SolidWorks tabletop scene. Mesh coordinates and URDF origins are metres.

No physical inertial guesses are included. Effort=0 deliberately grants no effort authority; it is not a servo rating. Velocity=10 deg/s is the existing firmware command slew ceiling. This geometry package does not provide a hardware driver, measured feedback, payload rating or physical-dynamics validation.

The servo collision envelope uses its exact source boxes and cylinders. Other collision meshes preserve the CAD solids. No convex print proxy has been accepted; do not let a physics importer silently replace each concave link with one convex hull. Shaft/horn spline engagement is intentional in the source. J2 and J3 maximum-at-other-joints-zero poses penetrate the tabletop.

Original project asset licensing still applies; no new license rights are asserted by this generated package.
