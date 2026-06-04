# ME492 Midterm — Speaker Script

---

## SLIDE 1: System Architecture

Last term we designed the system. This term we implemented the full pipeline in simulation and we are planning the physical deployment.

*(point to diagram)*

Our stack runs on ROS2 Humble — MoveIt2 for motion planning, Gazebo for simulation, YOLO11 for detection.

The orchestrator is a state machine that coordinates the full harvest cycle — scan, detect, approach, pick, deliver.

On the left — the software stack. All the compute nodes — vision, planning, control.

On the right — hardware interface. Two paths: simulation via Gazebo, real robot via ROS2 drivers.

For deployment, we plan to  run everything in Docker on a Jetson Xavier — we went this route because of ROS version compatibility issues

Everything else you see here — detection pipeline, gripper selection, clustering — was covered last term. see details in the appendix.

---

## SLIDE 2: Demo Video + Dashboard

*(play video, narrate as it goes)*
Here you see our complete harvest pipeline running end-to-end. On the right is the simulation, on the left is a live dashboard parsed from ROS2 logs.


Right now it's scanning — the arm sweeps three positions, YOLO detects clusters at each one and merge into 3D positions at the end

Now it's approaching cluster 1 — you can see the boll detections on the left, 12 bolls found.

Now the pick cycle — pre-grasp, open gripper, approach the boll, close, retract, go to reservoir, release, return. these are for each boll.

Same routine for cluster 2... and cluster 3.

The video is sped up — simulation runs slow on CPU due to WSL issues. Won't be an issue on the real robot with the Jetson's GPU.

---

## SLIDE 3: Technical Highlights (~2 min)

here are three engineering decisions worth highlighting regarding the software
1. instead of letting moveit2 pick a random inverse kinematic solution(which resulted in weird paths in big moves), compute IK directly with two seeds(second as fallback, as home position) then feed the joint solutions to the OMPL path planner or go direct trajectory for collision free path planning

2. secondly wew did an adversarial test: we laced a white boll in the screne model correctly ignored it resulting in low false positive rates.
3. another one is computing approach vector ensures gripper to face the target instead of random orientation found from path planner, which is critical for consistent camera view during detection. 

---

