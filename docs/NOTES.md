current state: 

- ros2 ws setup complete, i can run demo simulation with 4 component/nodes, talking to each other succesfully.
- sample URDF → rviz showcase is done.

TASKS:

- lets think fundamental parts before building a sophisticated environment & robotic simulation. we need to research-find and download URDF file of a 6 DOF(or 5,6) robotic arm(preferrably with gripper end-effector). we want the file to be highly descriptive i.e. containing all link,visual,geometry,collision,inertial information.
- [x]  research-find and download the URDF file, run it succesfully(rviz or other tools? i need to learn this too.)
    - we can start with the one i’ve put in the research folder but it might need some updates since its too old?
    - after i learn the fundamentals, play with it in the simulation environment , we’ll continue with real-example robot
        - https://github.com/aieask/iris_arm : similar to ours, has gripper
- [ ]  our cursor point we care is lets say the gripper start or end. base-platform being fixed, we want to locate/position the cursor autonomously to fixed and dynamically calculated target views such as ENVIRONMENT_LEFT/RIGHT_EDGE ,  CLUSTER_VIEW_POSITION(single cluster full view according to the vision-ml box prediction) , BOLL_VIEW_POSITION, RESERVOIR_TOP_POSITION. so we can start developing an approach to move the cursor to target position. i’ll first do research on that and we’ll develop step-by-step.
- [ ]  since we’ll do the physical integration second semester and we’ll build the environment based on our simulation, i need to build a simple prototype digital environment and create a map of it to define it. i ‘ll define fixed reference positions of the environment and the position base-platform will be placed. GAZEBO / IGNITION
- for more advanced example projects for inspiration :
    - https://github.com/aieask/iris_arm : similar to ours, has gripper
    - https://github.com/joshnewans/articubot_one
    - https://github.com/erkartik2001/5DoF_RoboticARM/tree/main
- i guess before going into sophisitaction of the simulation my immediate goal is this point.

---

---

FURTHER EFFORTS: 

- turn the solidworks stl → URDF file https://grabcad.com/library/arduino-braccio-robotic-arm-1[-1](https://grabcad.com/library/arduino-braccio-robotic-arm-1)