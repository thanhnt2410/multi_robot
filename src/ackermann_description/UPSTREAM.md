# Upstream attribution

Source: https://github.com/Balakarthik15/Ros2_ackermann_robot
Commit: b298e0551aaddef6f78868f9921e5e42cca91edb
License: MIT; see LICENSE.upstream (installed with the description).

vehicle.urdf.xacro is adapted from src/ackermann_gazebo/urdf/bot_sample.xacro.
Changes: parametric namespace, rear-axle root, remove lidar and virtual steering,
remove native drive plugin, passive front rolling joints, lower rolling damping,
ros2_control integration, ground-truth odometry and contact sensor.
No editable mechanical CAD was present in the imported assets.

Mechanical visual upgrade (2026-11-06): the original primitive visuals are replaced
by project-authored CadQuery meshes. The joints, inertias and collision shapes
remain those of this adapted simulation. Source CAD: cad/mechanical_vehicle.py.
The new CAD has 97 part instances, exported to ackermann_mechanical.step.
References consulted without importing code or geometry: cad/REFERENCES.md.
Steering uses two independent direct kingpin servo envelopes; no central tie rod
or mechanically constrained four-bar linkage is simulated.
