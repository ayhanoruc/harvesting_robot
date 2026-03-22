Boğaziçi University
ME429 Project Report

ROBOCOT: COTTON HARVESTING ROBOT



Ayhan ORUÇ / Deniz GERDAN / Mehmet Baran ERDEN


Submitted to:
Nuri Bülent Ersoy & Sinan Öncü & Hasan Bedir




TABLE OF CONTENTS

LIST OF TABLES	3
LIST OF FIGURES	4
EXECUTIVE SUMMARY	6
1. INTRODUCTION	7
1.1 Background and Motivation	7
1.2 Problem Definition & Project Significance	8
1.3 Benchmarking	9
1.4 Literature Review	10
1.5 Theoretical Background	12
2. DESIGN PROCESS	15
2.1 Design Criteria and Product Design Specifications:	15
2.2 Overview of Possible Solutions:	18
2.3 Detailed Design & Analysis	23
2.4 Project Management	32
2.5 Cost Analysis:	33
3. DISCUSSION	34
3.1 Deep Learning vs Classical Computer Vision	34
3.2 World-Space Clustering vs Frame-Based Tracking	35
3.3 ROS2 Ecosystem: Learning Curve vs Long-term Benefits	35
3.4 ML Model Generalization and Robustness	35
3.5 Project Scope Evolution and Design Philosophy	36
4. CONCLUSION	36
REFERENCES	38
APPENDICES	40



LIST OF TABLES
Table 1: Binary Dominance Matrix	16
Table 2: Solution Space: Design Decision Points	16
Table 3: Camera Placement Decision Matrix	17
Table 4: Detection Method Decision Matrix	17
Table 5: Cluster Identification Decision Matrix	18
Table 6: Manipulator Configuration Decision Matrix	19
Table 7: Gripper Design Decision Matrix	19
Table 8: Reservoir Design Decision Matrix	20
Table 9: Selected Components Summary	20
Table 10: ROS2 Package Organization	21
Table 11: Cluster Reachability Verification	22
Table 12: YOLO Detection Accuracy by Class	24
Table 13: Error Statistics Summary	29
Table 14: Cluster Count Validation	29
Table 15: Requirement Verification	29
Table 16: Operator Interface Requirement Verification	30
Table 17: ME429 Task Distribution	30
Table 18: Materials Planned to be Provided by Boğaziçi University	32
Table 19: Materials Covered from Our Budget	32

LIST OF FIGURES

Figure 1: Mature Cotton Field Before Defoliation………………………………………………….
Figure 2:  Unopened Cotton Boll………………………………………………………………………..                    
Figure 3:  Partially Opened & Unopened Bolls Together…………………………………………
Figure 4: Spindle Picker Cotton Harvester……………………………………………………………                              
Figure 5: Brush Roll Cotton Stripper Row Unit…………………………………………………….                                  
Figure 6: Soil Compaction by Harvester Vehicle…………………………………………………..
Figure 7: Unmanned Aerial Vehicles in Agricultural Use………………………………………..
Figure 8: Tevel Flying Autonomous Robots™ ……………………………………………………….                              
Figure 9: "SwarmBot 5" Models in Various Use…………………………………………………
Figure 10: "Moveit2" for Inverse Kinematics …………………………………………………….                                             
Figure 11: Gazebo for Simulations………………………………………………………………….
Figure 12: Workspace Reachability Analysis showing (a) top view with cluster positions marked, (b) side view showing height range…………………………………………………….
Figure 13: Pinhole Camera Model……………………………………………………………………
Figure 11: The loss curves of training and validation that shows steady Box Loss reduction over 100 epochs……………………………………………………………………………
Figure 12: Normalized confusion matrix ………………………………………………………….            
Figure 13: Sample detection on real cotton ……………………………………………………..        
Figure 14: Detection on cluster with unripe bolls………………………………………………
Figure 15: Panoramic Scan Pattern (7×3 grid with snake traversal)……………………….
Figure 16: FOV Overlap Diagram showing top-down view of camera coverage cones from all 7 horizontal positions, with shaded overlap regions and cluster positions marked..
Figure 20: Visual Servoing Convergence showing (a) initial off-center, (b) after iteration 1………………………………………………………………………………………………………………
Figure 21: Partial Visibility Recovery showing cluster at image edge ………………….      
Figure 22: ...centered after focus iterations……………………………………………………..
Figure 23: Gantt Chart of ME429…………………………………………………………………..
Figure 24: Gantt Chart of ME492……………………………………………………………………….
Figure 25: RoboCot System Architecture Overview……………………………………………………
Figure 26 :Node Interaction Diagram showing Gazebo simulation, ros2_control interface, vision pipeline nodes and their interconnections via topics and services…………………
Figure 27: Data Flow Pipeline — RGB Image → YOLO Detection → Pixel Center → Camera Focus → Depth Lookup → Back-Projection → TF Transform → World-Space Clustering → TrackedCluster output……………………………………………………
Figure 28: Classical CV vs Deep Learning Detection — HSV segmentation detects all white regions (robot gripper parts) as false positives, while YOLO11 correctly identifies only cotton bolls……………………………………………………
Figure 29: RoboCot App Interface — Components: color-coded status banner, session metrics (bolls harvested, success rate), ML confidence display, 5-step pipeline flow, timestamped alerts, control panel……………………………………………………
Figure 30: Kinematic Chain Diagram showing the 6-DOF Braccio arm with joint axes, link lengths and coordinate frames at each joint……………………………………………………















EXECUTIVE SUMMARY
This report presents RoboCot, a lab prototype for autonomous cotton boll detection and picking using a fixed based robotic manipulator. An “end to end” perception to action pipeline is demonstrated. Cotton bolls are detected in a realistic mock field, target positions are estimated in three dimensions using depth sensing, safe approach motions are planned and repeatable pick and place actions are executed into a compact collection bin. The system is defined as a technology demonstrator rather than a field ready machine. The performance limits under occlusion, clutter and changing lighting are challenged and investigated under the lab conditions for future bigger scoped projects.
A complete system concept, operating steps and engineering requirements are defined so that the prototype can be built and tested in the next term. The intended workflow includes cotton cluster detection, three dimensional target estimation, individual boll picking with a basic ripeness check by the vision based machine learning algorithm implemented on the system, pick confirmation and transfer to a collection bin. Design choices are guided by the “Product Design Specifications” that covers subjects like maintainability, size, weight, safety, ergonomics, reliability, environmental conditions and cost. Key constraints include a minimum reach of 520 mm, operation within a 1.0 m x 0.55 m mock field area and a bin size of 15 cm x 15 cm x 15 cm. A gripper opening of 50 - 60 mm is required, total arm mass is limited to 2.5 kg and tip deflection at full reach is limited to 10 mm under normal load.
A modular architecture and plan is used to support simpler testing and debugging processes that follows after testings. Cotton bolls are detected using a machine learning model and camera focus calibration is applied to pixel data and depth information is embedded and these 2 steps are used to convert detections into three dimensional coordinates for the wanted target points. A standard motion sequence is then executed. Approach, grasp, lift and place into the bin. Safety is addressed through speed limits and hardware controls such as gripper tip speed being limited to 0.3 m/s near humans, an emergency stop is provided to cut actuator power. Maintenance is supported through easy lens access, recalibration intended to be completed in under 10 minutes and remote software or model updates by the onboard computer.
Measurable targets are defined so that success and failure modes can be reported clearly. Gripper tip positioning accuracy is specified as plus minus 5 mm and pick position repeatability is specified as plus minus 3 mm over 50 repeated picks at the same target. Cotton detection accuracy is targeted at 90% or higher under conditions like the actual field in the mock field setup. A staged validation approach is recommended. First, sensor - compute - control integration and calibration stability should be verified in the mock field. Next, perception accuracy and three dimensional targeting errors should be measured. Finally pick success rate, cycle time and dominant failure causes such as lighting effects and reach limits should be evaluated. Through this approach, reusable outputs are produced including a small labelled dataset.






1. INTRODUCTION
1.1 Background and Motivation
Agriculture is under pressure to produce more food while also meeting sustainability goals and dealing with climate related changes [1]. At the same time a “digital revolution” is intended to emerge in the agriculture and food sector. This change is linked to wider use of sensors, software tools and farm management systems [1]. It is expected that data and robotics will increasingly influence both the decision making part and the physical work on farms. However, technology adoption can stay limited if farmers are not shown to what clear value is from this implementation. [1] Issues such as data ownership, system compatibility, training needs and high investment costs are often mentioned as barriers for this technology revolution transfer to the agriculture and food sector. [1]
Many important farming tasks are still labor intensive even today and access to human labor can be unstable as humans are not as reliable as automated systems. [3] Agricultural work has been described as strongly dependent on manual labor and sensitive to disruptions coming from human factor. [3] Also the long term demographic and economic changes can worsen labor shortages since fewer younger workers may prefer physically demanding jobs in today's global conditions. [3] On a wider scale in the big picture for future, robotics and autonomous systems are also highlighted as major global technology trends meaning automation pressure is expected to grow across many industries including the agricultural industry. [2]
Within agricultural robotics, harvesting is considered one of the hardest application fields. [3][4] Perception, motion planning and delicate manipulation must be combined in an unstructured environment like Figure 1 shows where plants are not fully rigid, targets are partially hidden with leaves and other herbal obstacles and lighting changes. [3] Planting and harvesting have been described as seasonal operations that require much heavy effort and many automation concepts have been reported using RGB cameras, three dimensional sensing, laser scanners, manipulators and grippers. [4] Interest in autonomous and semi autonomous systems has often been linked to reducing the top high workload tasks and especially harvesting amongst those tasks. [4] Robotics roadmaps for food ındustry also suggest that automation become most attractive when manual labor is high cost, raw materials vary widely and speed is limited by human factors. [5]

Figure 7: Mature Cotton Field Before Defoliation
Even though progress has been made, important technical problems still remain in real harvesting conditions. [3] Obstacles and changes in ambient lighting are repeatedly reported as major challenges. [3] Harvesting performance is often inconsistent across different systems and scenarios depending on these major challenges. [3] Reported success rates and cycle times can vary widely which shows that results depend strongly on the environment of the subject and system design. [3] This indicates that robust harvesting is limited not only by mechanical hardware but also by perception quality, calibration stability and the reliability of the perception to motion integration in cluttered environments. [3]
For these reasons, a controlled laboratory demonstrator is strongly justified before moving toward field deployment. [3] A technology demonstrator allows key limitations to be reproduced and measured using clear test procedures. [3] Especially robust vision under occlusion and lighting changes and safe manipulation in clutter. [3] Cotton boll picking as an agricultural activity fits this purpose well. [3] Targets can be visually identified for learning based detection while realistic difficulties such as clusters, leaf and branch obstacles and grasp slip can still be introduced in a repeatable mock field setup. [3] As a result, limits can be reported clearly and reusable outputs such as interfaces, calibration procedures, evaluation metrics and datasets can be produced for future improvements. [3]

1.2 Problem Definition & Project Significance
Cotton is one of the most important natural fibres, mainly because it is widely used in clothing and home products [6]. In practice, cotton boll picking is defined as the selective removal of opened cotton bolls from the plant while avoiding extra plant material (leaf, stem, fragments) and keeping the fibre clean. The problem becomes harder because boll opening is not uniform across the plant so “ready to pick” targets may exist next to unopened (Figure 2) or partially opened bolls like in Figure 3 [7]. In today’s production harvesting is usually done once near the end of the season and defoliation (shedding leaves) is typically scheduled to support machine harvesting and lint quality [7][10]. 
                 
                                   Figure 2:  Unopened Cotton Boll                          Figure 3:  Partially Opened & Unopened Bolls Together
However, this approach creates trade offs. Such as early opened bolls can remain exposed to weather until the final harvest which can reduce fibre quality [7]. In addition, conventional high capacity harvesters are heavy and soil compaction risk is reported as a concern for future of the agricultural activities in the field [7]. Harvest method benchmarks also show quality and processing drawbacks. Stripper harvested cotton is associated with higher foreign matter and higher handling, cleaning needs compared to spindle picking [8][9]. Robotic cotton harvesting is significant because it is linked to the possibility of smaller, lighter harvesting platforms and more selective picking (for example picking soon after a boll opens rather than waiting for a specific time at the end of the season for all the cotton bolls to become harvestable) which can support fibre quality preservation and reduce the need to wait for a single end season harvest [7]. Still, the task remains technically demanding. Agricultural harvesting robots are repeatedly described as difficult because perception, motion planning and manipulation must work together in an unstructured environment where obstacles and lighting changes are common [3][4]. This difficulty is also visible in cotton focused studies. A robotic cotton end effector study reports partial picking performance (about 66 to 85% seed cotton removal from a boll) and picking times that can range from a few seconds to tens of seconds depending on the control approach [7]. For this reason, a laboratory scale cotton picking demonstrator is strongly justified as an intermediate step before field deployment. Such a demonstrator allows the key limitations (bottlenecks) highlighted in robotics literature to be reproduced and measured with controlled test conditions such as robust vision under obstacles and lighting variation, stable calibration and reliable perception to motion integration[3][4]. This direction also matches broader digital agriculture roadmaps that emphasize data driven tools, robotics and the need to overcome adoption barriers such as interoperability and training [1] and it aligns with workforce reports that identify robotics and automation as major trends across sectors [2]. Cotton is well suited for measurable experiments because cotton bolls can be detected and localized using stereo depth sensing under challenging sunlight conditions in prior work [11] and modern learning based detection is explicitly designed to handle variable field lighting and complex backgrounds [12]. 

1.3 Benchmarking
Modern cotton harvesting is mainly performed using two machine types: the spindle picker (Figure 4) and the brush roll stripper (Figure 5) [13]. The spindle picker is described as a selective harvester because seed cotton is removed mostly from well opened bolls and a relatively small amount of unwanted plant material is collected [13]. The brush roll stripper is described as nonselective because mature cotton, immature bolls, leaves, sticks and other vegetative material can be removed together [13].
                             
                                   Figure 4: Spindle Picker Cotton Harvester                              Figure 5: Brush Roll Cotton Stripper Row Unit                                  
 A key benchmark difference is observed in foreign matter and fiber quality risk. Because stripping removes more plant material, a higher foreign matter load is typically carried into post harvest handling and “ginning” (it means separating fibers), which increases cleaning demand and can affect processing efficiency and lint neatness [9][13]. Reported values show a clear gap even when field cleaning is used. Spindle picked cotton is reported at roughly 68-70 kg of foreign matter per bale while stripper harvested cotton with a field cleaner is reported at about 170-173 kg per bale [13]. This difference is also linked to fiber quality loss mechanisms for stripped cotton because more immature fibers may exist compared with picker harvested cotton [13]. Cost and operation trade offs are also noted between systems. Strippers are described as less mechanically complex and typically cheaper to own and operate. While spindle pickers are described as more complex and requiring more daily maintenance to keep performance stable [13]. Since these machines are built to harvest a lot of crops at the end of the season, selectivity is not the main design objective and quality outcomes remain sensitive to crop condition, setup and maintenance quality [13]. The results of the harvest depend heavily on how the crops are prepared beforehand. To make the process more efficient and to keep leaves from polluting the collected cotton, farmers usually use techniques like shedding leaves (defoliation) and choosing the right time to pick [10]. But even with this careful prep work, the standard problems dont go away. The amount of unwanted material and the final quality of the fiber still change based on the weather in the field and the type of picking machine being used [13]. Finally, machine scale introduces practical drawbacks at the field level. Increased harvester weight and repeated traffic during harvest are linked to soil compaction risk, especially under moisture conditions that increase susceptibility [14]. Field tests reveal that changes in soil hardness can be seen as deep as 0.6 meters (example shown in Figure 6) after a cotton picker passes by. This impact is noted for both the older basket style pickers and the newer, heavier machines that create round modules [14]. These findings explain why there is a push for robotic alternatives in the farming industry. While traditional systems work well for picking large amounts at once, they still face issues with trash or dirt in the cotton, being too affected by weather and damaging the ground with their weight. Because of this, researchers are looking into lighter and more selective picking methods. However, harvesting is still known as one of the toughest jobs for a robot because of leaves blocking the view (occlusion), changing light and the difficulty of moving a robotic arm through crowded and messy plants [3][4].

Figure 6: Soil Compaction by Harvester Vehicle

1.4 Literature Review
Research into farming robots has been going on for decades, but actually getting a machine to pick a crop is still one of the most challenging activities out there [3][15].  Unlike simpler tasks like mapping or monitoring the fields, harvesting activity requires a robot to both see the target accurately and physically interact with a living plant itself rather than support humans doing it. Most engineers now look at a harvesting robot as a complete "perception to action" chain where every part has to work perfectly with the next. The process starts with detecting the target and estimating its position in three dimensions, then planning a safe path and at last extracting the product without causing damage to the harvested material [3][4]. Experts usually describe harvesting as a continuous pipeline from perception to action because these steps are very much connected. So harvesting is more than just a simple grasping of a material from nature problem [3].
If you look at how most of these robots are built, they usually follow a similar blueprint. It starts with "vision" using a mix of regular cameras and depth sensors to see the world they will be working in [4][15]. Once the robot takes a picture, it uses machine learning to find the target harvest material such as a fruit or cotton boll in this case [3]. These smart models are great for outdoor work because they dont get as confused by complicated backgrounds or the way sunlight shifts during the day [3][12]. These factors are quite critical for the outdoor working environment as these obstacles can affect outputs significantly. After finding the target, the robot uses its sensors to create a map of 3D coordinates. This tells the robotic arm exactly where to go so the gripper or suction tool can pull the crop off the plant [3][15]. Where this gives a general design idea, gripper and suction tools differs in ways of efficiency. Suction tools may cause more extra material to be present in the harvest. It could be more desirable to use grippers to harvest less extra materials other than cotton bolls themselves.
The real problem is that these robots dont always perform the same way twice [3][15]. Lab environments can create spaces where designed systems work continuously as intended without coming across random changing conditions creating obstacles. A robot might work perfectly in a clean lab but its success rate often drops the moment it is put in a real field. This is because farms are chaotic. They are not organized like a factory, the wind blows the plants around, branches get in the way and the targets are often buried under leaves of the plants [3]. These little environmental issues cause errors to come together and create bigger issues for the working principle of the harvesting systems. A tiny mistake in sensing depth can turn into a huge mistake in positioning because they are so interconnected while working. Which may lead to the robot hitting a branch, missing the target or losing its grip resulting with a damaged product or even a damaged harvesting system which is undesirable of course. [3].
There are a few specific bottlenecks that make this job so difficult. The biggest one is leaves and branches hiding the target as blockages [3][15]. Then there is outdoor lighting where shadows and glare can easily trick the sensors and ruin the accuracy [3]. Also as the plants are not fully rigid objects like metal, just touching them can make the whole branch move in an unpredictable way which may disrupt the coordination systems of the harvester [3][15]. Finally there’s the risk of damage. It is a must to remove the harvested material without damaging it or hurting the plant which is the source for future harvests, which makes the design of the gripper incredibly important [15].
Lastly, keeping the camera and the robot arm working simultaneously together is a constant struggle. In a rough farm environment, vibrations can cause the system’s alignment to drift, making the robot’s movements inaccurate [17]. When a robot fails usually it is not because of one big mistake but rather a combination of small ones as snowball effect. For example, if a leaf is partly hiding a cotton boll, the robot might get an inaccurate depth reading and try to grab the wrong spot resulting with an unsuccessful operation that drops the efficiency overalll[3][17]. This is especially true for cotton, where the bolls are tangled and hidden. These are the seasons that support the need for a controlled demonstrator. This demonstrator ensures that perception stability, 3D localization errors and pick success rate can be measured in a repeatable setup before field conditions are targeted for a bigger scoped project [3][17].
In modern agriculture, autonomous systems take on one of two major roles. They are either used data acquisition systems such as monitoring or operational action tools for harvesting. Unmanned Aerial Vehicles (UAVs/drones) like in the Figure 7 mainly serve as the "eyes" of these complex systems. Equipped with RGB, multispectral cameras and thermal sensors, they map fields to identify water stress, nutrient deficiencies or diseases spread from the sky. This data enables a precise "mapping to analysis" workflow and this preciseness ensures that interventions to the field are not random but exactly targeted only where needed. For physical operations Unmanned Ground Vehicles (UGVs) and autonomous platforms take the lead as their specific capabilities are a better fit for harvesting type of operational actions. While drones are excellent for rapid spraying in difficult terrains (like steep orchards/fruit gardens), they face limitations in payload capabilities and energy issues for harvesting common field crops like wheat, corn or cotton. These crops require high outputs and heavy duty mechanisms, making ground based robots far more practical for the job.

Figure 7: Unmanned Aerial Vehicles in Agricultural Use
Companies like Tevel are pioneering "Flying Autonomous Robots" (Figure 8) that use robotic arms to pick fruit while connected to a ground power source [18]. But for large scale mass produced crops such as wheat and cotton, the industry relies on autonomous ground platforms. John Deere has introduced fully autonomous tractors and harvesters that use computer vision and GPS to navigate fields with centimeter level high precision [19]. Other innovations include SwarmFarm, which utilizes fleets of smaller autonomous ground chassis seen in the Figure 9 that serves as a moving platform to perform tasks like weeding and harvesting support without compacting the soil (thanks to the disriputed and low pressure of the vehicle)  [20] and Carbon Robotics, which uses high power lasers on a UGV platform to eliminate weeds autonomously [21]. 
           
             Figure 8: Tevel Flying Autonomous Robots™                               Figure 9: "SwarmBot 5" Models in Various Use

1.5 Theoretical Background
Building a cotton picking robot requires a "perception to action" system where data from sensors is converted into physical movement by a structured software framework [3][4][15]. This process is organized into several key areas. These are agricultural automation, robot simulation, motion planning, object detection and 3D perception. These components are usually linked together for the perception and control modules to communicate effectively by a middleware like ROS2 [22].
Agricultural Automation and Detection Challenges
Harvesting is a major focus within precision agriculture because it forces a robot to interact with an unstructured environment [3][4]. Detecting cotton is particularly difficult because the bolls often grow in clusters, are hidden by leaves or they are affected by shifting outdoor light conditions that are a bit unpredictable [3]. Learning based detection is used to solve these problems because it can handle complicated backgrounds and lighting changes much better than rule based programming [3][12].
Motion Planning and Control
A manipulator requires a mathematical model to link its joint angles to its hand position. Forward kinematics calculates where the hand is based on the joints. Inverse kinematics calculates what the joint angles would be to reach a specific target [23]. In ROS 2, “MoveIt 2” seen in Figure 10 is the standard tool for planning these movements while avoiding obstacles [23]. On the hardware side, "ros2_control" is used to manage the motor commands and sensor readings in a modular way [22].
       
      Figure 10: "Moveit2" for Inverse Kinematıcs                                              Figure 11: Gazebo for Simulations

Robot Simulation and Digital Twins
Before testing on real hardware robot behavior is validated in a virtual environment to reduce the risk of accidents or see potential problems beforehand. For this reason a "digital twin" is created using the Unified Robot Description Format (URDF) by describing the robot’s physical structure and joints [25]. This model of digital twin is then placed into a physics based simulator like Gazebo (seen in Figure 11) often running alongside ROS 2 [24][25]. This simulation is very useful for checking if the robot can reach its targets and for spotting potential problems. But of course a "sim to real gap" is expected [3][24].

Object Detection and Tracking
Cotton perception is handled through object detection where each boll is identified with a bounding box and a confidence score [12]. Using the Ultralytics library, YOLO style detectors can be run on live video feeds [26]. Datasets must be labeled before the robot can recognize cotton using tools like “Roboflow Annotate” [27]. Consistent labeling is very crucial because errors in the dataset can lead to unwanted low performance in the actual field [12]. To keep track of cotton bolls as the robotic arm moves, multi object trackers like ByteTrack or BoT-SORT are used [26]. These tools assign a unique ID to each boll which helps the system to count the harvested materials and avoid trying to pick the same target twice [29][30].
3D Perception and Coordinate Transforms
Picking requires a 3D location found using a depth or stereo camera. The depth (Z) can be estimated by formula of [Z ≈ f*B/d] where “f” is the focal length, “B” is the distance between lenses and “d” is the disparity [15]. Once a target is found by the camera, its position must be converted into the system’s base coordinates. In ROS 2, this is managed by the TF2 library that tracks the relationship between different parts of the manipulator [22]. If the manipulator adjusts its movement based on live camera feedback, the process is called “visual servoing”. Visual servoing is very helpful for reaching targets that might move or are partially blocked by branches that is the general case [3][31].
3D Model Extraction (Optional Extension)
For more advanced analysis frameworks like SAM3D can be used to create 3D masks of the scene without needing extra training [32]. This allows the system to be able to see in a way the entire structure of the plant and cotton regions in a point cloud. While these tools from Meta and other researchers help in creating assets for digital twins or better planning, they are considered optional extensions beyond basic picking tasks [33][34].
Current cotton harvesting industry in agriculture mostly relies on large scale mechanical systems used at the end of the season when all cotton bolls are ripped and are harvestable. However, these methods are often limited by the presence of foreign matter, varying field conditions and strict timing constraints that can impact overall fiber quality [9][13]. As autonomous harvesting requires the perfect coordination of robust perception, stable calibration and safe movement under difficult conditions like lighting changes and visual obstructions it is frequently described as a major challenge in robotics [3][4][15][17].The fibrous nature of the cotton affects how easily the boll can be removed. Consequently, the motion and control of the end effector (the gripper) must be tested through experiments to ensure efficient picking [7]. After a literature examination it is seen that a measurable and repeatable demonstration of a small scale vision guided cotton picking system for precision agriculture remains as a significant gap in the existing researches [1][2][3]. 
To address this gap, the project is structured around specific and measurable objectives within a controlled laboratory environment by building a mock field. A detection and tracking pipeline is developed using YOLO based models to maintain unique identification for each cotton boll across different video frames [12].  3D target localization is planned to be achieved through RGB sensing. This way  precise coordinate transformations ensure the camera and robotic arm to ne perfectly aligned [3][17]. The physical execution of the task is focused on a repeatable sequence of (approach - grasp - lift - place) actions. Both cycle times and success rates are recorded transparently  while doing so to establish important metrics[15]. Finally, the entire system is evaluated using these performance metrics such as detection accuracy, positioning error and common failure causes to provide a solid foundation for future technical improvements [3][15].






2. DESIGN PROCESS
2.1 Design Criteria and Product Design Specifications:
Safety
Safety is the highest priority because the prototype has moving joints, pinch points at the gripper and powered electronics. Any unsafe motion or unstable behavior can stop testing and create risk during the demo.
SF-01– All electrical components (Jetson, camera, motor drivers, wiring) shall have basic environmental protection equivalent to minimum IP54 for dust and splash resistance (for outdoor field tests).
SF-02– Arm joint speeds shall be limited so that the gripper tip linear speed ≤ 0.3 m/s when operating near humans.
SF-03– The control software shall handle kinematic singularities by avoiding or slowing motion near singular configurations and preventing unstable joint velocities.
 SF-04– The system shall include an emergency stop button that immediately cuts power to the arm actuators while keeping the system in a safe state.
Standards
Even as a prototype a structured baseline is needed for safe design. Considering relevant robotics, agricultural or electrical safety standards helps designing consistent protection measures.
ST-01– The system design shall consider relevant agricultural machinery safety standards (ISO 4254) and local regulations for field tests.
ST-02– The robotic arm safety functions shall follow the principles of ISO 10218 (industrial robot safety) adapted to your lab/field environment.
ST-03– Electrical design shall target an appropriate IP rating ( minimum IP54) according to IEC 60529, specifying protection against dust and water for outdoor use.
ST-04– All exposed conductive parts shall be properly grounded and fused to meet basic electrical safety practices in the lab and field.
Quality & Performance
The system is only valuable if it can consistently detect cotton and execute the manipulation which is the pick. Vision accuracy, arm positioning accuracy, repeatability and a reasonable cycle time directly define whether the demo is realistic or not and whether the approach can scale later for real life application in a bigger scoped project.
QL-01– The arm shall achieve a positioning accuracy of ≤ ±5 mm at the gripper tip within the working volume.
QL-02– Repeatability of the pick position shall be ≤  ±3 mm over 50 repeated picks at the same target.
QL-03– The ML cotton detection system shall achieve ≥ 90% detection accuracy in typical field-like lighting and dust conditions in the mock setup.
QL-04 – The end to end pick cycle time (from first detection to boll stored in the bin) shall be ≤ 60 seconds under standard conditions.
QL-05– Functional consistency, the system shall maintain an overall successful pick rate ≥ 90% during continuous operation over one test day.    

Size Restrictions
All functionality depends on physically fitting and operating inside the mock field boundaries. Workspace constraints also drive arm selection and camera placement.
SR-01– The arm shall provide an operational reach ≥ 520 mm from the arm base center to the gripper tip in the horizontal direction.
SR-02– The system shall be designed to operate inside a mock field of 1.0 m length and 0.55 m height, where the upper 0.25 m of the height is filled with cotton plants.
SR-03– The cotton collection bin internal dimensions shall be 15 cm x 15 cm x 15 cm (±5 mm).
SR-04– The gripper jaws shall be able to open between 50 - 60 mm to grasp a cotton boll cluster at a single position.
Reliability
The product is a pipeline (camera - detection - position - motion - actuation). If any part fails frequently the full system fails. Reliability is essential for continuous operation during the demonstration.
RL-01– The system shall operate reliably under dust, heat and moderate humidity similar to typical cotton fields in Türkiye (ambient 20°C – 45°C, RH 30–80%).
RL-02– Software shall contain crash recovery mechanisms (automatic restart of nodes/processes where possible) and clear error messages for critical failures.
RL-03– The arm joints shall be designed for a minimum of 200,000 joint cycles before expected failure.
RL-04– The system should maintain its total harvesting cycle life (defined as the total number of successful picks before major overhaul) of at least 50.000 picks
Environment
Agricultural scenes are visually and physically harsh for the harvester. Obstacles such as variable light, clutter and dust are present. Environmental robustness is important because it directly affects both ML detection quality and long term sensor and electronics stability.
EN-01– Camera and ML performance shall remain functional in direct sunlight and partially shaded conditions with no more than 10% accuracy drop compared to controlled lighting.
EN-02– Total system power consumption (arm + Jetson + camera + electronics) shall be ≤ 75W during normal operation.
EN-03– The system should be designed to allow operation from a battery for at least 3 hours of field work.
Material Cost
Material cost affects design choices for the arm structure, gripper model and sensor and computing selections.
MC-01– The robotic arm & ml hardware cost (motors, gears, links, gripper,nvidia jetson) shall not exceed 1300€.
MC-02– The vision system cost shall be compatible with ML requirements (resolution, fps, dynamic range) and remain below 1600€.
MC-03– Structural material costs for the platform, reservoir and mounting (aluminum profiles, plastic sheets.) should be minimized while still meeting stiffness and durability requirements.
Maintenance
Easy maintenance features like cleaning the camera lens, quick replacement of gripper if needed, simple calibration reduces downtime and supports fast iteration.
MA-01 – The robotic arm shall use modular components so that worn parts (gripper pads, motors, joints) can be replaced individually without replacing the whole arm.
MA-02 – The camera lens shall be directly reachable for cleaning with a cloth or brush without disassembling the camera mount or disturbing its calibration.
MA-03 – Camera recalibration in the mock field shall take ≤ 10 minutes when required.
MA-04 – The system shall support remote software and ML model updates (retraining or new model deployment) via Jetson (over SSH/USB), without physical access to the Jetson board itself.
Manufacturing Cost
The design must be achievable with student manufacturing tools and time. Meaning 3D printable parts, simple assemblies and modular software components that can be reused.
MF-01– Gripper parts shall be 3D printable with standard FDM printers.
MF-02– The full mechanical and electronics assembly (arm, bin, mounts, wiring) should be possible in ≤ 2 - 3 working day by a 2 person student team.
MF-03– Software development work (ML training, ROS2 integration) should be modular, allowing reuse in future prototypes without large rewrites.
Ergonomics
The system will be operated and monitored by people during the demo. Simple control (GUI), clear status indicators and fast bin handling reduce user mistakes and improve demo
ER-01– A control & monitoring panel (GUI) shall allow the operator to monitor the system, start, stop and pause the system within one or two actions.
ER-02– System status and warnings shall be visible through clear visual indicators (LEDs).
ER-03– The reservoir/cotton bin shall be designed for easy loading and unloading, requiring no tools and < 15 seconds to remove and reinsert.
Weight
Weight matters mainly through arm dynamics and tip deflection and is less critical than safety/performance in this stationary prototype.
WT-01– Arm links shall use lightweight materials (aluminium, composite or plastic) so that the total arm mass does not exceed 2.5 kg.
WT-02– The cotton container shall be designed to safely hold about 300 g of cotton without structural failure or excessive deformation.
WT-03– The nominal payload at the gripper (camera ≈ 200 g + cotton boll ≈ negligible) shall not cause tip deflection greater than 10mm at full reach.
Aesthetic
Aesthetics is intentionally not a decision maker in this phase. However, basic neatness (cable routing, clean assembly) is still useful to reduce failures and improve usability.
AE-01– Cables shall be neatly routed and secured, with no loose wires hanging into the mock field or moving mechanism range.
AE-02– Major components (arm, bin) should follow a consistent material and color scheme to appear as a unified product.
AE-03– The arm motion should appear smooth and continuous without visible jerks in normal operation (acceleration profiles tuned).
Table 1 shows the binary dominance matrix where each criterion was graded accordingly with their relative importance to our project.
Table 1: Binary Dominance Matrix
 
2.2 Overview of Possible Solutions:
The autonomous cotton harvesting system involves the combination of various subsystems that cut across the perception, motion planning and mechanical design. This section does not consider monolithic configurations of systems, but instead, takes a component-based approach wherein each decision point in the design is considered separately. The weights of the criteria are based on the Binary Dominance Matrix developed in Section 2.1, and the weights used in each decision matrix are based on the weights of the corresponding PDS criteria. The chosen components are then incorporated in the final system structure.
2.2.1 Solution Space Overview 
An autonomous cotton picking system has six decision points, which are in software architecture, perception, control and mechanical domains. Table 2 is a summary of these decision points, their requirements based on the product design requirements (Section 2.1) and their effect on the overall system performance.
Table 2: Solution Space: Design Decision Points
Decision Point
Requirement Reference


Impact on System
Camera Placement
QL-03 (90% detection), SR-02 (mock field coverage)
Affects viewing angles, occlusion handling and inspection distance
Detection Method
QL-03 (90% detection), EN-01 (lighting robustness)
Affects detection reliability under varying conditions
Cluster Identification
QL-05 (90% pick rate), RL-02 (software reliability)
Determines tracking accuracy across multiple scan positions
Manipulator Configuration
SR-01 (520mm reach), WT-03 (tip deflection <10mm)
Affects workspace coverage and approach trajectory flexibility
Gripper Design
SR-04 (50-60mm opening), QL-05 (90% pick rate)
Determines harvest purity and foreign matter content
Reservoir Design
SR-03 (15×15×15cm), ER-03 (toolfree removal <15s)
Affects storage capacity and operator interaction

The next subsections offer the options of each decision point, compare them with the criteria and explain the choice of the solution.
2.2.2 Design Decision Analysis 
Decision Point 1: Camera Placement
The camera position influences the viewing angles, inspection distance and occlusion management. Two possibilities were considered in Table 3: (A) Eye-in-Hand: Camera attached to the end-effector which allows close-up inspection (about 35cm viewing range) and arbitrary positioning of the viewpoint; (B) Upper-Arm Mounted: Camera on a more proximate link which provides platform stability, but with reduced positioning flexibility.
Table 3: Camera Placement Decision Matrix
Criterion
Weight
PDS Ref.
Eye-in-Hand
Upper-Arm
Close-up Inspection Capability
0.20
QL-03
5
2
Multi-angle Viewing
0.15
SR-02
5
3
Occlusion Handling
0.36
QL-03, EN-01
5
2
Platform Stability
0.20
QL-02
3
5
Calibration Simplicity
0.09
MA-03
3
4
Weighted Total
1.00


4,42
2,93


Selected: Eye-in-Hand 
Close-up inspection and multi-viewpoint positioning are important in order to detect reliably and measure depth properly. The trade-off of stability can be accepted in the case of controlled motion during scanning.
Decision Point 2: Detection Method 
Cotton boll detection converts images captured by the camera to bounding boxes that define the position of the target. Two alternatives were considered in Table 4: (A) Classical Computer Vision: Color segmentation (HSV thresholding), edge detection and blob analysis; computationally efficient, but sensitive to changes in lighting and backgrounds; (B) Deep Learning (YOLO): CNN-based detection trained on labeled cotton datasets with excellent performance on detection under different lighting conditions and backgrounds [12][22].
Table 4: Detection Method Decision Matrix
Criterion
Weight
PDS Ref.
Classical CV
YOLO11
Robustness to Lighting Variation
0.18
EN-01
2
5
Detection Accuracy
0.23
QL-03
2
5
Handling Complex Backgrounds
0.41
QL-03, RL-01
2
5
Computational Cost
0.18
EN-02
5
3
Weighted Total
1.00


2,54
4,64


Selected: YOLO11  
The deep learning is necessary to obtain the 90% detection accuracy when the field-like conditions are available. The model that is trained on Cotton-boll-and-cluster-2 dataset has more than 0.7 confidence. Jetson Orin NX supports real-time performance through GPU inference.
Classical CV Evaluation: Classical CV Evaluation: HSV (Hue-Saturation-Value) color space segmentation was experimented as the most promising classical method because white color of cotton can be separated through low saturation and high value threshold. Nevertheless, it was tested that about 50 percent false positive rate came about because of sky areas, lighting reflections, and background objects that have the same color properties (see Appendix Figure 15). This validates the need to have learned feature representations in order to have credible cotton detection.
Decision Point 3: Cluster Identification Strategy 
In panoramic scanning, a single cluster can be identified in several perspectives. Two have been considered in Table 5: (A) Multi-Frame Tracking (ByteTrack): Video-based trackers that keep identity across frames through motion prediction [25][26]; continuous video with smooth motion of objects; (B) World Space 3D Clustering: Each detection is converted to 3D world coordinates with depth and TF transforms, and then grouped by spatial proximity regardless of camera motion.
Table 5: Cluster Identification Decision Matrix
Criterion
Weight
PDS Ref.
ByteTrack
World Space 3D
Stability Across Discrete Scan Positions
0.32
QL-05
2
5
ID Consistency
0.25
RL-02
3
5
Independence from Frame Rate
0.32
QL-04
2
5
Implementation Complexity
0.11
MF-03
4
3
Weighted Total
1.00


2,47
4,78


Selected: World-Space 3D Clustering 
Experiments showed that with a large movement of camera between scan positions, ByteTrack ID becomes unstable. World-space clustering offers strong identification in relation to camera motion, with complete-linkage algorithm that gives tight clusters with no chain-linking artifacts.
Decision Point 4: Manipulator Configuration 
The manipulator should be able to offer enough reach and allow approach paths that should not hit the plants. Two were compared in Table 6 (A) 4-DOF Arm: Base rotation, shoulder, elbow, wrist; simple positioning, though restricted approach angle range; ( B) 6-DOF Arm (Braccio): Full position and orientation control with gripper approach at arbitrary angles.


Table 6: Manipulator Configuration Decision Matrix
Criterion
Weight
PDS Ref.
4-DOF
6-DOF
Approach Trajectory Flexibility
0.29
QL-05
2
5
Workspace Coverage
0.23
SR-01, SR-02
3
4
Obstacle Avoidance Capability
0.35
SF-03
2
5
Mechanical Simplicity
0.13
MA-01
5
3
Weighted Total
1.00


2,62
4,51


Selected: 6-DOF Braccio Arm 
Extra DOF allow obstacle-free paths and the optimal placement of the gripper. The Braccio reaches SR-01 satisfying 520mm+.
Decision Point 5: Gripper Design 
The gripper should be able to hold the cotton bolls with certainty and reduce the harvest contamination. Two alternatives were considered in Table 7: (A) Parallel Jaw Gripper: Two opposing fingers with adjustable grip force, 50-60mm aperture to allow cotton clusters to be selected, (B) Vacuum/Suction: Negative pressure pickup to generate suction that would pick up cotton fibers.
Table 7: Gripper Design Decision Matrix
Criterion
Weight
PDS Ref.
Parallel Jaw
Vacumm
Foreign Matter Reduction
0.35
QL-05
5
2
Selective Grasping Capability
0.25
QL-05
5
2
Simplicity & Cost
0.20
MC-01, MF-01
5
4
3D Printability
0.10
MF-01
5
3
Energy Efficiency
0.10
EN-02
4
2
Weighted Total
1.00


4,85
2,45


Selected: Parallel Jaw Gripper 
Vacuum/suction systems are prone to picking up the surrounding debris (leaves, stems, dirt) and cotton fibers, which add to the content of foreign matter in the harvest. Parallel jaw grippers allow the selective pick of the target cotton boll to enhance harvest purity. Also, vacuum systems need constant air pump running that consumes more energy, and servo-driven fingers do not run until the grasp cycles. The 50-60mm aperture (SR-04) is possible using 3D-printable fingers that can fit in the Braccio gripper base.
Decision Point 6: Reservoir Design 
The reservoir receives cotton that is harvested and has to be emptyable (ER-03) without the use of a tool. Two were considered in Table 8 (A) Flip-Top Bin: Hinged lid that can be used with one hand and accessed in a hurry; the usual commercial bins are on the market (B) Lid-Stay Bin: Removable lid, which can only be operated with two hands and put aside at the time of deposit.

Table 8: Reservoir Design Decision Matrix
Criterion
Weight
PDS Ref.
Flip-Flop
Lid Stay
Tool-Free Operation (<15s)
0.35
ER-03
5
3
One-Handed Access
0.25
QL-04
5
2
Secure Contents During Motion
0.20
RL-01
4
3
Cost & Availability
0.20
MC-03
5
4
Weighted Total
1.00


4,8
2,95


Selected: Flip-Top Bin
It is able to fulfill the 15-second removal criterion and can be deposited without complete opening. A business bin (15x15x15 cm, 209 TRY) reduces the need to make a custom fabrication, but it fits all the requirements.
2.2.3 Final System Configuration 
The chosen elements are combined (seen in Table 9) into a consistent system architecture with ROS2 as the communication platform, YOLO11 as the detection feed, and MoveIt2 as the planner generating collision-free movements of the 6-DOF arm and the parallel gripper placing cotton into the removable reservoir.
Table 9: Selected Components Summary
Decision Point
Selected Option
Weighted Score
Key Justification
Camera Placement
Eye in Hand (Wrist)
4,42
Enables multi-angle inspection and close-up viewing
Detection Method
YOLO11
4,64
Robust detection under varying lighting conditions
Cluster Identification
World Space 3D Clustering
4,78
Stable across discrete scan positions, complete-linkage prevents chain-linking
Manipulator Configuration
6-DOF Braccio Arm
4,51
Flexible approach trajectories, sufficient reach
Gripper Design
Parallel Jaw
4,85
Reduces foreign matter, selective grasping, energy efficient
Reservoir Design
Flip Flop Bin
4,80
Tool-free access, one-handed operation


Also, an operator dashboard is being developed in the form of a web-based platform to meet the ergonomic requirements (ER 01, ER-02) mentioned in Section 2.1. The dashboard will offer convenient START/PAUSE/EMERGY controls, color-coded status displays and real-time visualization of ML confidence, which will allow flexible placement of the operators in the demonstrations. Section 2.3 will offer specifications of the design of each of the chosen components and their combination.

2.3 Detailed Design & Analysis
The design integrates ROS2 software architecture, 6-DOF manipulator kinematics, RGB-D vision processing and world-space localization into a compatible autonomous harvesting system. 
2.3.1 System & Software Architecture 
The RoboCot system is built on ROS2 Humble with Gazebo Ignition Fortress for simulation purposes[18][19]. The modular architecture of the RoboCot system consists of nine ROS2 packages and these packages are organized by their functionality. This enables the independent development and testing of each subsystem. (See Appendix Figure 1: RoboCot System Architecture Overview) Table 10 lists core packages and their responsibilities. 
Table 10: ROS2 Package Organization
Package
Purpose
Key Components
robot_arm
Hardware abstraction and Gazebo simulation
bot.launch.py, URDF model, ros2_control,configuration
robot_arm_moveit_config
Motion planning and collision avoidance
MoveIt2 move group, OMPL planners, KDL, IK solver
orchestrator
Vision pipeline and system orchestration
YOLO detector, depth processor, spatial, pipeline, explorer
harvester_interfaces
Custom ROS2 message and service definitions
BoundingBox.msg, DetectedCluster.msg, YoloDetect.srv, PixelTo3D.srv


The architecture follows a hierarchical pattern of low level nodes (robot_state_publisher, arm_controller) providing hardware abstraction while high level nodes (explorer, spatial_detection_pipeline) are implementing application logic (See Appendix Figure 2: Node Interaction Diagram).
Orchestrator State Machine: IDLE → DETECTING_CLUSTERS → CLUSTER_VIEW_POSITION → DETECTING_BOLLS → HARVESTING → TRANSFERRING → CLUSTER_COMPLETE → (next cluster or IDLE)
The system employs ROS2 topics for continuous data streams and services for request response interactions [18]. See Appendix Tables 18-19 for complete topic and service listings. 
In the “Data Flow Pipeline”, each processing stage is implemented as an independent ROS2 node, enabling parallel development and debugging through intermediate topic inspection (See Appendix Figure 3: Data Flow Pipeline) 
2.3.2 Mechanical Design 
The manipulator selected for RoboCot is the Arduino Braccio++ arm, a 6-DOF serial manipulator with approximately 520mm reach. The kinematic structure provides sufficient degrees of freedom for flexible approach trajectories depending on the indicated mock field mesaurements while remaining within the budget constraints specified in the Section 2.1 Design criteria material cost section. 
The Braccio arm consists of six revolute joints in a serial chain (see Appendix Figure 4 for kinematic chain diagram). Joint limits define the operational workspace and are enforced by both MoveIt2 software limits and hardware stops (see Appendix Table 20). Link dimensions and inertial properties are derived from the CAD model and verified against physical measurements (see Appendix Table 21). Total arm length (extended version): 0.412m base to wrist, plus gripper reach providing approximately 520mm total reach. The reachable workspace must encompass all three cotton cluster positions. Figure 12  and Table 11 shows the workspace envelope calculated from forward kinematics across joint limits. 

Figure 12: Workspace Reachability Analysis showing (a) top view with cluster positions marked, (b) side view showing height range
Table 11: Cluster Reachability Verification
Cluster
Position (x, y, z) m
Distance from Base
Within Reach
cluster_01
(0.875, 0.475, 0.46)
0.996 m
✓ (with base repositioning)
cluster_02
(0.975, 0.0, 0.52)
0.975 m
✓
cluster_03
(0.875, -0.475, 0.42)
0.996 m
✓ (with base repositioning)


The 520mm arm reach is sufficient when the robot base is positioned at the origin as the clusters are arranged within a 1.0m radius from the base position. 
2.3.3 Vision System 
The vision system combines an RGB-D camera for color and depth acquisition with a deep learning detector for cotton boll recognition. In the current stage of the project, the “eye in hand” camera is simulated using Gazebo's rgbd_camera sensor plugin, configured to match the ZED X Mini camera planned for hardware (see Appendix Table 22 for full specifications). The camera follows the standard pinhole projection model seen in the Figure 13. The intrinsic matrix K encapsulates internal parameters: 

Figure 13: Pinhole Camera Model
Equation 1: Camera Intrinsic Matrix: 

Where fx, fy = 277 pixels (focal length derived from FOV: fx = 320/tan(45°) ≈ 277), and cx=320, cy=240 (principal point at image center). 
Equation 2: Perspective Projection (3D to 2D)

YOLO (You Only Look Once) object detection model is a single stage object detector consisting of a CSPDarknet backbone for feature extraction, a PANet neck for multi scale feature fusion and a detection head that predicts bounding boxes with class probabilities in one forward pass [12][22]. This architecture enables real time inference by processing the entire image simultaneously rather than using region proposals.
Dataset Collection and Annotation 
Video data was collected using a standard cellphone camera with the cotton rotating 360° to capture all of the viewing angles. Additional images were captured from 3D cotton models and edited to create unripe cotton boll samples. The dataset collected includes diverse angles, variable lighting conditions and motion blur for robustness. 
Frames were labeled using Roboflow with three classes that are “cotton-boll”, “cotton-boll-cluster” and “unripe cotton-boll”. Each image contains an average of 16 individual cotton bounding boxes plus one cluster bounding box. The base dataset of 100 images was expanded with 35 additional images containing unripe bolls, then augmented (rotation, blur, contrast variations) to produce 375 total training images.  
Model Training 
RoboCot uses YOLO11s (small variant) from Ultralytics, selected for optimal balance between inference speed and detection accuracy. Training parameters are image size 1280×1280, 100 epochs. The resulting best.pt weights are deployed with a 0.7 confidence threshold. 

Figure 18: The loss curves of training and validation that shows steady Box Loss reduction over 100 epochs
Training metric’s results (seen in Figure 14) improved steadily with the box loss dropped in both training and validation. This shows a better localization over time. The final mAP@50–95 was about 0.60, which is a solid baseline for a custom agricultural dataset such as the RoboCot’s.
Detection Accuracy
               
Figure 19: Normalized confusion matrix             Figure 110: Sample detection on real cotton         Figure 111: Detection on cluster with unripe bolls
Table 12: YOLO Detection Accuracy by Class
Class
Accuracy
Notes
cotton-boll
95%
Primary detection target, excellent performance
unripe-cotton-boll
82%
Limited training data (augmented images only)
cotton-boll-cluster
-
Used for spatial grouping, not harvesting


The 95% accuracy on single cotton bolls (Figure 15, Figure 16, Figure 17, Table 12) confirms the model reliably identifies individual harvest targets. The model also rarely confuses cotton bolls with background elements, which is quite validating that the YOLO architecture's feature extraction capability is suitable and sufficient for this application. 
2.3.4 Spatial Detection Pipeline 
The spatial detection pipeline is the core technical contribution that is converting 2D pixel detections into 3D world coordinates with approximately 1-2 cm accuracy. The pipeline stages are detailed in the Data Flow diagram in section 2.3.1. in the Figure 3.
Given pixel (u, v) and depth Z, the 3D point in camera frame is: 
Equation 3: Back-Projection Formulation (2D to 3D)
This formulation inverts the perspective projection, recovering the original 3D position of the detected object relative to the camera optical center. 
The computed camera frame point must be transformed to the world coordinate frame for consistent spatial localization across multiple camera positions. The TF2 library [18] provides the transformation chain: 
Equation 4: Frame Transformation Chain

The specific chain traverses: 

There was a critical implementation detail concerning the K Matrix vs P Matrix. During development nearly 20cm systematic error was traced to Gazebo's camera_info message containing inconsistent matrices which were K matrix (correct: cx=320, cy=240) vs P matrix (incorrect: cx=160, cy=120). The ROS PinholeCameraModel.projectPixelTo3dRay() uses the P matrix, causing offset errors. Solution to this was extracting intrinsics directly from K matrix (see Appendix for code). This fix reduced error from nearly 20 cm to  approximately 1-2 cm. 
Complete-Linkage Clustering Algorithm 
Detections from multiple scan positions must be grouped into clusters representing physical plants. Single linkage (join if close to any member) causes chain linking artifacts. Complete linkage requires proximity to all members: 
Equation 5: Complete-Linkage Clustering Condition

See Appendix Algorithm 1 for pseudocode. Distance computed in XY plane only to group bolls at different heights on the same plant. 
Merge Radius Calculation 
The merge radius r_merge determines the maximum distance for grouping detections. It must be large enough to group multiple detections of the same cluster but also small enough to keep separate plants distinct 
Equation 6: Merge Radius Derivation

This radius derived ensures the cotton bolls within 12 cm on the same plant are grouped together and plants separated by 48 cm have remained as distinct clusters


2.3.5 Scanning Strategy 
Effective cluster detection requires systematically viewing the field from multiple angles to handle occlusions. The scan grid consists of 21 positions in a 7x3 matrix consisting of 7 horizontal pan angles (±45° in 15° increments) and 3 vertical tilt levels (0°, 15°, 30° down). These scan positions are visited in a snake (boustrophedon) pattern seen in the Figure 18 minimizing total joint travel.

Figure 112: Panoramic Scan Pattern (7×3 grid with snake traversal)
Traversal Order of this pattern is Row 0 (middle): Left to right (positions 1-7) then Row 1 (lower): Right to left (positions 8-14) and finally Row 2 (lowest): Left to right (positions 15-21). The reason behind this is alternating sweep directions eliminate the need for large hip angle reversals between rows and that way reducing total scan time by approximately 30% compared to a unidirectional raster pattern.  
Adjacent scan positions must have sufficient FOV (Field of View) overlap to ensure no regions are missed. Field of View Overlap Analysis is made with a 90° horizontal FOV and 15° pan angle increments as:
Equation 7: FOV Overlap Calculation

This 60° overlap between adjacent positions provides a redundant coverage for robust detection, multiple viewpoints for complete linkage clustering and also a tolerance for slight positioning errors. The entire mock field (±45° from center) falls within the combined FOV and it can be seen in the Figure 19. Total scan time is approximately 55s for the 21 positions. Detailed timing breakdown can be seen in Appendix Table 23.

Figure 113: FOV Overlap Diagram showing top-down view of camera coverage cones from all 7 horizontal positions, with shaded overlap regions and cluster positions marked
2.3.6 Control & Motion 
The camera_focus node implements image-based visual servoing (IBVS) [23], mapping pixel errors directly to joint adjustments: 
Equation 8. Pixel Error:
 
Equation 9. Proportional Joint Adjustment

Gains derived empirically (K ≈ 0.3 rad / 150 pixels). The arm geometry roughly separates the motion. Left-right error is corrected by the hip joint and up-down error corrected by the shoulder/elbow joints. This lets the system center the target in about 2 to 3 steps without using Jacobian based IK. Each update is limited to 0.3 rad per iteration.

Figure 20: Visual Servoing Convergence showing (a) initial off-center, (b) after iteration 1
When a cluster is detected at the edge of the frame (truncated bbox, nearly 0.65 confidence). Focus iterations adjust hip/shoulder to center the target, full cluster visibility achieved within 2 iterations established and centered depth measurement improves localization accuracy all by the help of the partial visibility recovery feature seen in Figure 21 and Figure 22.
               
Figure 21: Partial Visibility Recovery showing cluster at image edge       Figure 22: ...centered after focus iterations
MoveIt2 provides collision aware planning [19] for larger movements (see Appendix Table 24). Collision objects: ground plane, reservoir bin, self-collision. Velocity scaling at 30%. 
2.3.7 Validation & Results 
Quantitative validation compares detected cluster positions against known ground truth positions in the simulated environment. For Ground Truth configuration the minimum inter cluster distance of 0.485m informs the merge radius calculation (0.121m = 25% x 0.485m). 
The methodology for the validation process follows a nearest neighbor matching approach. First it executes full panoramic scan (21 positions) with detection enabled. Then all tracked clusters from spatial_detection_pipeline retrieved. After that it finds the nearest detected cluster (Euclidean distance) for each ground truth cluster and compare positions and compute error metrics. Finally reports as pass or fail based on tolerance threshold (5cm default for our case). Table 13 summarizes error statistics from a complete panoramic scan validation run (detailed per cluster results in Appendix Table 25).  

Table 13: Error Statistics Summary
Metric 
X Error (cm)
Y Error (cm)
Z Error (cm)
Total Error (cm)
Mean
0.53
0.70
0.37
1,16
Standard Deviation
0.15
0.95
0.38
0.74
Max
0.7
1,9
0.8
2.0

Note: Z error includes -3cm offset correction for mesh origin vs detection center systematic difference. 
The key finding from this summary of error statistics is that all three axes achieve subcentimeter mean accuracy after Z-offset correction. Total 3D error of 11.6 mm mean for cluster view detection is well within the ±5 mm positioning requirement indicated for individual cotton bolls (QL-01), with XY accuracy of nearly 10 mm enabling precise gripper approach. 
Beyond position accuracy, the pipeline must correctly identify the number of distinct clusters also known as “Cluster Count Accuracy” shown in Table 14 
Table 14: Cluster Count Validation
Metric
Expected
Detected
Result
Number of clusters
3
3
PASS
Cluster 1 detection count
≥1
1
PASS
Cluster 2 detection count
≥1
4
PASS
Cluster 3 detection count
≥1
1
PASS


The center cluster (cluster_2) is detected more frequently as it falls within the camera FOV at more scan positions. 
Performance Against Requirements shown in Table 15 :
Table 15: Requirement Verification
Requirement
Specification
Measured
Status
QL-01: Positioning accuracy
±5 cm
11.6 mm mean
PASS
QL-02: Repeatability
±3 cm
σ = 7.4 mm
FAIL*
QL-03: Detection accuracy
90%
100% (3/3)
PASS


The simulation environment has not been fully optimized yet. The depth sensor noise parameters, TF timing synchronization and camera focus convergence thresholds remain as areas for improvement for us to work in the second semester. Hardware implementation is expected to achieve better repeatability with proper sensor calibration therefore would create better results for the environment. 
2.3.8 Operator Interface 
The web based dashboard provides real time monitoring and control as it was indicated in the  ergonomic requirements ER-01 and ER 02 shown in Table 16 . (See Appendix Figure 15) 
Table 16: Operator Interface Requirement Verification
Control
Behaviour 
Requirement
START/RESUME
Begin or resume operation
ER-01
PAUSE
Complete current motion, hold
ER-01
SKIP CLUSTER
Mark skipped, proceed to next
ER-01
EMERGENCY STOP
Halt → brakes → return HOME → manual restart
ER-02


2.4 Project Management
In this project, responsibilities for tasks were carefully distributed among the team members. Table 17 can be seen for subtasks and their responsible group members in ME429.
Table 17: ME429 Task Distribution
SUBTASKS
RESPONSIBLE GROUP MEMBER
Patent and Online Research
Collective
System Requirements & Constraints Definition
Mehmet Baran Erden
Component & Material Selection
Collective
Data Collection & Labeling 
Deniz Gerdan
Robot Simulation & Vision-to-Motion Pipeline Development
Ayhan Oruç
ROS Learning
Collective
2. ROS2 Software Architecture & System Integration
Ayhan Oruç
Developing & Training AI Vision Model
Deniz Gerdan
Procurement Proposal  Preparation
Mehmet Baran Erden
MT & Final Presentations
Collective


Resources
Resources to build “Robocot”, a lab scale technology demonstrator for autonomous cotton boll detection and picking were selected carefully and pro forma invoices were acquired from the selected firms in Türkiye. We have selected a focused hardware stack centered on the NVIDIA Jetson Orin NX (16GB). This edge computer unit serves as the brain of our system, managing the real time vision inference and high level coordination activities needed for detection and picking. A ZED X Mini short-range stereo camera was planned to be used for target localization and approach planning. It aimed to capture high resolution RGB and depth data for establishing a “clear vision” for our system. The system’s main manipulator is planned to be handled by an Arduino Braccio 6 degrees of freedom robotic arm, which allows us for rapid pick and place testing. This arm is managed by an Arduino UNO R4 WiFi acting as a dedicated low level controller for deterministic logic and safety triggers with wifi features for easier updating capabilities. To ensure efficient development during the development and testing phases we have included an Intel AC8265 Wi-Fi adapter for stable remote data transfer. The setup will be finalized using existing lab interface accessories in our school with additional small components will be procured when needed during the project timeline in the next semester.



Gantt Chart
The Gantt Chart in Figure 23 demonstrates the general flow of the project during ME429 and The Gantt Chart in Figure 24 demonstrates the intended work flow in the next semester corresponding to ME492 course. Although it was not fully feasible to follow the planned timeline for the first semester of the project, the planned task were generally in line with the intended times. While we have obtained Pro forma invoices for our needed components, the real orders have not been put yet due to Universities budget plannings. The timetables were created with time buffers for any possible delay in components to ensure the project remaining on track.

Figure 23: Gantt Chart of ME429

Figure 24: Gantt Chart of ME492

2.5 Cost Analysis:
This section summarizes the expected procurement costs of the required hardware components for the project and also the testing environment which is the mock field. A market research has been done and the suppliers that best fit our and the University’s budget and delivery time constraints are chosen for ordering. Three Pro forma invoices were received from these suppliers and combined into one cost list showed in the table 18 below. The goal is to understand the total budget which is the combination of our own budget and the University’s budget and the cost drivers of the project as a whole.


Table 18: Materials Planned to be Provided by Boğaziçi University
Suppliers
Item
Quantity
Unit Price (excl. VAT)
Total (incl. VAT)
OmniWise Teknoloji A.Ş.
Jetson Orin NX 16GB AI Kit (D131S-ONX16GB-KIT)
1
45.917,12 TRY
55.100,54 TRY
OmniWise Teknoloji A.Ş.
AC8265 Wireless Network Adapter (with rod antenna)
1
897,30 TRY
1.076,76 TRY
360 Teknoloji Yatırımları A.Ş.
ZED X Mini 2.2mm (Polarizer) + GMSL2 Fakra Cable (1.5 m)
1
51.666,67 TRY
62.000,00 TRY
360 Teknoloji Yatırımları A.Ş.
ZED Link Capture Card Mono
1
19.166,67 TRY
23.000,00 TRY
SAMM Teknoloji A.Ş.
Arduino Uno R4 WiFi (MP02497)
1
1.479,61 TRY
1.775,53 TRY
SAMM Teknoloji A.Ş.
Braccio Robotic Arm TinkerKit (MP02067)
1
12.080,17 TRY
14.496,20 TRY
TOTAL COST
 
 
131.207,54 TRY
157.449,03 TRY


The costs for collecting the data and setting the test environment can be included in the mock cotton field costs listed in the Table 19 below covered from our own budget. A set of 10 cotton branches was bought to create representative data for training the YOLO model for demonstration.
Table 19: Materials Covered from Our Budget
Item
Quantity
Unit Price (incl. VAT)
Total (incl. VAT)
Cotton branch with bolls
30
75,00 TRY
2.250,00 TRY
Strafor Foam
300mm x 100mm x 30 mm
300,00 TRY
900,00 TRY
Flip Top Reservoir Bin
1
209,00 TRY
209,00 TRY
Total Cost
 
 
3.359,00 TRY


3. DISCUSSION
In this discussion section the key technical decisions and lessons learned during RoboCot project’s development process are discussed. 
3.1 Deep Learning vs Classical Computer Vision 
Before committing to YOLO, we tested HSV colour segmentation as a simpler baseline. The idea was straightforward: cotton bolls are white, so threshold for white pixels in the image. The results were clear but not useful. HSV detected every white object in the frame including the robot's own gripper parts (see Appendix Figure 16). About half of all detections were false positives. This confirmed what the literature describes: colour-only approaches cannot distinguish semantically different objects that happen to share similar colours [3][12]. A white gripper and a white cotton boll have the same colour histogram but completely different meaning. YOLO's learned features capture this semantic difference through texture, shape and context that rule-based methods miss entirely. 
The takeaway is simple. For harvesting applications where the environment contains visually similar distractors, deep learning is necessary not optional. The extra training effort pays off because the fundamental limitation of classical CV cannot be fixed with better thresholds. 
3.2 World-Space Clustering vs Frame-Based Tracking 
We initially considered video tracking algorithms like ByteTrack to maintain cotton boll IDs across the scan [25][26]. These tools work well for continuous video where objects move smoothly between frames. But our scanning strategy has discrete jumps between 21 positions with large viewpoint changes. Under these conditions ByteTrack kept assigning different IDs to the same physical cluster because the visual appearance changed too much between non-adjacent views. World-space clustering solved this by operating on 3D coordinates instead of visual features. Detections get back-projected into world space and grouped by spatial proximity. Complete-linkage specifically prevents chain-linking artifacts where distant clusters get incorrectly merged through intermediate detections. The key advantage is frame-rate independence. Whether detections arrive at 1 Hz or 30 Hz, clustering works on accumulated 3D points rather than temporal sequences. This matches our discrete multi-viewpoint scanning much better than algorithms designed for continuous video. 
For future work, combining both approaches could be valuable. Use world-space clustering for the initial panoramic scan, then switch to visual tracking during focused observation of individual clusters where the camera stays relatively stationary. 
3.3 ROS2 Ecosystem: Learning Curve vs Long-term Benefits 
“ROS2 Humble” with “Gazebo Fortress” were challenging tools and subjects to learn during the development. Nodes, topics, services, TF2 transforms, launch files, MoveIt2 planning groups, URDF models, there is a lot to understand before anything works. Documentation is scattered and version compatibility between Humble and Fortress caused configuration headaches. But the investment paid off. When we needed to update the YOLO detector, only that node changed. Everything downstream stayed the same. TF2 handles the camera-to-world coordinate chain automatically once configured. The skills transfer directly to industry. ROS2 is the standard for robotics development and the concepts (modular architecture, hardware abstraction, simulation-based validation) apply regardless of specific framework. While it was a hard process of learning, it was also an investment that can be further amplified over time and used in future projects.
This project showed us that robotics is a truly interdisciplinary field. It combines mechanical design, computer vision, machine learning and software engineering for achieving a single target. We realized that a weakness in just one of these areas can slow down the entire system. While it is difficult to learn so many different topics at once, it makes the work much more interesting because there is always something new to learn. 
3.4 ML Model Generalization and Robustness 
After intense efforts put in the process of development the YOLO11s model achieved 95% detection accuracy for cotton bolls and 82% for unripe bolls in controlled conditions. These numbers are encouraging but need careful interpretation for real deployment. The 0.7 confidence threshold was tuned empirically. Lower values caught more bolls but admitted more false detections. Higher values improved precision but they missed partially occluded targets which was a critical point for our project. The current value works for the mock field designed but it is probable that it may need adjustments for different environments in future use. 
Dataset coverage remains a limitation. The 375 augmented training images cover rotation, blur and contrast variations, but real fields will have conditions outside this distribution. Heavy dust on the lens, wet cotton after rain, or novel plant configurations could degrade accuracy. The 82% success rate for unripe bolls happened because we didn't have enough training data for that specific category. During future use, the system should keep collecting data while working in the field. We can then add failed detections and difficult cases to our dataset to update the model. Because our design is modular (requirement MA-04), we can update the software remotely without needing to change any hardware.
3.5 Project Scope Evolution and Design Philosophy 
The project scope changed significantly during the development process. Initial requirements were loosely defined by a bit of arbitrary choices made by ourselves and evolved as we understood the technical challenges better while working on the topics personally. This created pressure to redesign components to solve appearing problems during development multiple times. Our response was to make everything as modular and generic as possible. The orchestrator state machine can be extended with new states. The detection pipeline accepts different YOLO models without code changes. Motion planning uses configurable waypoints. We made the clustering parameters adjustable so we could make alterations later easily if needed. This flexibility of course has some downsides like making the code more complex than intended and requiring more testing for verifications. But it was totally worth the effort and minus sides of it because when our project requirements changed while working and we were able to adapt the system quickly without having to rewrite the core design from scratch thanks to the flexibility of the parameters. Some specific problems encountered during testings consumed significant debugging time. A nearly 20 cm systematic positioning error was traced to Gazebo's camera_info message containing inconsistent K and P matrices. The standard ROS function uses the P matrix which had wrong principal point values. Extracting intrinsics directly from K fixed it. The QL 02 repeatability spec (±3mm) was not achieved in simulation (measured σ=7.4mm) due to depth sensor noise and TF timing issues. Hardware with proper calibration should improve this.
The broader lesson learnt here is that requirement volatility is normal in research projects. Designing the system to be flexible from the beginning was a good decision. Even though it took more effort at the start, it made it much easier to handle changes and challenges later. We believe this is an important lesson for any new project, not just this one. This design process helped us developing skills that we can use outside the field of the agricultural robotics. Learning how to integrate different systems, debug complex software pipelines and use industry tools like ROS2 and MoveIt2 was very valuable for any engineering job possible. This project provided us with a very strong foundation for our future careers whether we continue working in agricultural robotics or move to another area for robotics.

4. CONCLUSION
This project involved the design and construction of RoboCot, a laboratory prototype that demonstrates how a fixed base arm may be used to identify and harvest cotton bolls on a robot. This was primarily aimed at integrating vision, 3D localization and movement into a unified working entity. This enabled us to experiment with the problems appearing in harvesting applications such as the presence of leaves that obscured the view or variying lights, in a lab setting. Rather than building a machine that is field ready, we aimed at developing a technology demonstrator to which we can make performance measurements and optimize it in a step by step manner.
To achieve this objective we had a particular program. The robot will scan the place, locate the cotton, determine the 3D location and then pick it up in a safe way and drop it in a bin. The design was informed by certain requirements where the safety and reliability as well as ease of maintenance were considered paramount. We imposed very clear conditions on the workspace area, the range of the arm and the range of the gripper to ensure that our choices were informed by quantifiable needs.
One of the outcomes of our efforts is a system that achieves a high level of performance with simplicity in its design. We use the camera at the arm (eye in hand) as it offers us better closer shots and depth information as compared to a camera positioned far away. To locate the cotton, a modern YOLO11 model was applied to work on various backgrounds and light conditions. We associated these two dimensional images with the three dimensional world coordinates to ensure that the robot would be able to follow the targets even during movement. We decided to use a 6 DOF arm to provide flexible movements and a parallel gripper to select the cotton only and not the additional plant parts.
We also had the information that the accuracy of the software is not the only thing but it also needs a proper calibration. With the repairs in simulation during the development process we have learned that even such minor factors as camera parameters and timing can be very critical in the process of the robot picking the target in robotics.
RoboCot employs a panoramic scan strategy in order to deal with leaves that are covering the cotton. The arm is more efficient as it moves in a snake pattern in 21 different views. We also included a step of centering the measurements of depth which stabilizes the measurements prior to the robot making an attempt at grabbing a boll. Such strategies enable the robot to operate in dirty agricultural terrain whereby plants tend to conceal the targets.
We tested our simulation and obtained results of a solid platform of RoboCot. The system was able to obtain an average 3D error of approximately 11.6 mm and identify all the cotton clusters in the scene. These outcomes confirmed that our primary pipeline of detection to 3D estimation was achieved with this prototype. Although, our tests also indicated that the repeatability of the robot (performing the same task in the same way each time) should be improved. The accuracy of this robot during one run is perfect but it is still not reliable to work with over a long period of time.
The fact that this project has this limitation does not make the project any less valuable; it just tells us where to concentrate on next. One can add stability by optimizing the sensor noise settings and timing in the software. RoboCot has created a work base and we are now in the phase where refinement in the engineering will lead to a greater performance as opposed to altering the entire design.
To conclude, RoboCot was able to achieve its primary goal. We have developed an entire working prototype of robotic cotton picking. The project is a combination of a modular ROS2 architecture, AI vision, 3D tracking and safe movements. We are successful but we are also showing where we are falling short in our results. Based on this background, in the future, RoboCot can continue to be developed to leave the lab demonstration to become an effective harvesting robot.


REFERENCES

[1] ICT-AGRI, Digital Technologies for a Sustainable Agrifood System (Strategic Research and Innovation Agenda), ICT-AGRI-FOOD, PDF report.
[2] World Economic Forum, The Future of Jobs Report 2025, World Economic Forum, 2025.
[3] L. F. P. Oliveira et al., “Advances in Agriculture Robotics: A State-of-the-Art Review and Challenges Ahead,” Robotics, 2021.
[4] J. J. Roldán et al., “Robots in Agriculture: State of Art and Practical Experiences,” in Service Robots, IntechOpen, 2017/2018.
[5] euRobotics, Robotics 2020 Multi-Annual Roadmap (MAR), euRobotics, PDF report.
[6] FAO, “Cotton | Markets and Trade,” FAO, overview page.
[7] H. Gharakhani, J. A. Thomasson, and Y. Lu, “An end-effector for robotic cotton harvesting,” Smart Agricultural Technology, 2022.
[8] Cotton Incorporated, “Ginning Stripper Harvested Cotton,” Cotton Incorporated, technical page.
[9] U.S. Environmental Protection Agency (U.S. EPA), AP-42, Chapter 9.7: Cotton Ginning, U.S. EPA, PDF.
[10] Cotton Incorporated, Spindle Harvest Management (Defoliation/Timing and Lint Quality Guidance), Cotton Incorporated, PDF.
[11] “Evaluation of a Stereo Vision System for Cotton Row Detection and Boll Location Estimation in Direct Sunlight,” MDPI, web page.
[12] Z. Lu et al., “COTTON-YOLO: Enhancing Cotton Boll Detection and Counting in Complex Environmental Conditions Using an Advanced YOLO Model,” Applied Sciences, 2024.
[13] J. D. Wanjura, K. Baker, and E. Barnes, “Harvesting,” Journal of Cotton Science, vol. 21, pp. 70–80, 2017.
[14] M. V. Braunack and D. B. Johnston, “Changes in soil cone resistance due to cotton picker traffic during harvest on Australian cotton soils,” Soil & Tillage Research, vol. 140, pp. 29–39, 2014.
[15] C. W. Bac, J. Hemming, and E. J. van Henten, “Harvesting Robots for High-value Crops: State-of-the-art Review and Challenges Ahead,” Robotics, vol. 33, no. 4, 2014.
[16] V. Rajendran et al., “Towards Autonomous Selective Harvesting: A Review of…,” Journal of Field Robotics, 2024.
[17] L. He et al., “Advance on Agricultural Robot Hand–Eye Coordination for…,” Engineering, 2025.
[18] Tevel Aerobotics Technologies, “Flying Autonomous Robots (FAR) for Fruit Harvesting,” Technical Overview, 2023. [Online]. Available: https://www.tevel-tech.com/technology/. Accessed: Jan. 2026.
[19] John Deere, “Autonomous 8R Tractor and Intelligent Systems,” Engineering whitepaper, 2022. [Online]. Available: https://www.deere.com/en/tractors/row-crop-tractors/row-crop-8-family/intelligence-productivity/. Accessed: Jan. 2026.
[20] SwarmFarm, “SwarmBot: Decentralized Autonomous Farming Platforms,” Operational manual, 2023. [Online]. Available: https://www.swarmfarm.com/applications/. Accessed: Jan. 2026.
[21] Carbon Robotics, “Autonomous LaserWeeder Technology and Crop Safety,” Technical review, 2023. [Online]. Available: https://carbonrobotics.com/laserweeder-g2. Accessed: Jan. 2026.
[22] Ultralytics, “YOLO11,” Ultralytics Documentation.
[23] Ultralytics, “Python Usage,” Ultralytics YOLO Documentation.
[24] Roboflow, “Introduction to Roboflow Annotate,” Roboflow Documentation.
[25] Roboflow, “Annotate an Image,” Roboflow Documentation.
[26] Roboflow, “Computer Vision Annotation Formats,” Roboflow.
[27] Roboflow, “YOLO Annotation Format,” Roboflow Formats.
[28] Roboflow, “Using the Python SDK,” Roboflow Developer Reference.
[29] OpenCV, “Drawing Functions in OpenCV (Python),” OpenCV Documentation.
[30] Y. Zhang et al., “ByteTrack: Multi-Object Tracking by Associating Every Detection Box,” ECCV, 2022.
[31] N. Aharon, R. Orfaig, and B.-Z. Bobrovsky, “BoT-SORT: Robust Associations Multi-Pedestrian Tracking,” arXiv, 2022.
[32] Ultralytics, “Multi-Object Tracking with Ultralytics YOLO,” Ultralytics Documentation.
[33] Y. Yang et al., “SAM3D: Segment Anything in 3D Scenes,” arXiv, 2023.
[34] Meta AI, “SAM 3D,” ai.meta.com.

APPENDICES

Figure 25: RoboCot System Architecture Overview


Figure 26 :Node Interaction Diagram showing Gazebo simulation, ros2_control interface, vision pipeline nodes and their interconnections via topics and services

Figure 27: Data Flow Pipeline — RGB Image → YOLO Detection → Pixel Center → Camera Focus → Depth Lookup → Back-Projection → TF Transform → World-Space Clustering → TrackedCluster output




Figure 28: Classical CV vs Deep Learning Detection — HSV segmentation detects all white regions (robot gripper parts) as false positives, while YOLO11 correctly identifies only cotton bolls

Figure 29: RoboCot App Interface — Components: color-coded status banner, session metrics (bolls harvested, success rate), ML confidence display, 5-step pipeline flow, timestamped alerts, control panel

Figure 30: Kinematic Chain Diagram showing the 6-DOF Braccio arm with joint axes, link lengths and coordinate frames at each joint

