Today's Immediate Tasks (in order)
1. Copy mesh assets into our repo
Create src/robot_arm/meshes/m1013_blue/ and copy DAE files from src/docs/RESEARCH/robocob_ws_sim/doosan-robot/dsr_description/meshes/m1013_blue/
Create src/robot_arm/meshes/m1013_collision/ from same source
Create src/robot_arm/meshes/hande/ from src/docs/RESEARCH/robocob_ws_sim/robotiq/robotiq_description/meshes/hande/
Also grab the support cube mesh (bau_tasiyici_v2.stl) from robocob_ws_sim/robot_description/meshes/
2. Write m1013_robocot.urdf.xacro
Port m1013_arm.urdf.xacro + robotiq_hande_gripper.urdf.xacro into a single Gazebo Ignition-compatible xacro
Replace package://dsr_description/ and package://robotiq_description/ mesh paths with package://robot_arm/meshes/
Replace ROS1 <transmission> tags with <ros2_control> block (pattern from current mybot.urdf.xacro)
Add gz_ros2_control plugin (same as current URDF)
Attach RGB-D camera to tool0 (copy camera section from current URDF)
Result: 6 revolute arm joints + 1 prismatic gripper joint + camera, all Ignition-ready
3. Update controllers.yaml
Arm joints: [joint1, joint2, joint3, joint4, joint5, joint6]
Gripper: [robotiq_hande_left_finger_joint] (prismatic)
Joint state broadcaster covers all
4. Update bot.launch.py
Point to new URDF
Keep everything else (Gazebo, bridge, controller spawners)
5. Build & smoke test in Gazebo
colcon build in WSL
ros2 launch robot_arm bot.launch.py
Confirm: M1013 + Hand-E visible in Gazebo, joints respond to controller commands
Week 1 Plan (today through lab visit + 2 days after)
Day 1 (today): Tasks 1-5 above. M1013 spawns in Gazebo, controllable via joint trajectory.
Day 2: MoveIt2 config for M1013 -- new SRDF (arm: joint1-6, gripper: hande finger), kinematics.yaml, collision matrix. Test RViz motion planning.
Day 3 (lab visit): Physical access to ROBOCOB. Document network setup, test Doosan controller ping (192.168.3.5), check Hand-E RS485 connection, take photos for presentation.
Day 4-5: Orchestrator adaptation -- update joint names in explorer, camera_focus, depth_processor. Retune panoramic scan angles for M1013 kinematics. Run full pipeline: scan -> YOLO -> 3D localization with M1013.
Week 2 Plan (before midterm)
Day 6-7: Gripper control node + pick-and-place sequence in sim (approach -> grasp -> lift -> place). Implement HARVESTING and TRANSFERRING states in orchestrator.
Day 8-9: End-to-end demo polish: full cycle (scan -> detect -> pick -> place) running smoothly in Gazebo. Record demo video.
Day 10: Environment config tuning (cluster positions for M1013 reach), presentation slides prep.
Buffer days: Fix whatever broke, fine-tune.
Midterm Deliverables
Gazebo sim demo: M1013 + Hand-E doing full cotton harvest cycle
Lab visit evidence: photos/video of physical ROBOCOB, network connectivity proof
Slide deck: architecture diagram (old 4-DOF vs new M1013), spec comparison table, sim results, hardware plan
Want me to start on Task 1 (copying meshes) and Task 2 (writing the M1013 xacro)?


