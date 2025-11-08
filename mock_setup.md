| Type    | Example Name         | Purpose                     |
| ------- | -------------------- | --------------------------- |
| Topic   | `/cluster_command`   | From orchestrator to others |
| Topic   | `/detected_clusters` | From vision to orchestrator |
| Topic   | `/system_status`     | Global state broadcast      |
| Topic   | `/reservoir_level`   | From actor to logger        |
| Service | `/move_to_cluster`   | Orchestrator → actor        |
| Service | `/classify_ripeness` | Orchestrator → vision       |
| Action  | `/harvest_boll`      | Orchestrator → actor        |

All of these can use standard messages (std_msgs/String, geometry_msgs/Point, etc.) for now — no need for custom interfaces until later.

Suggested Development Flow

    - Implement one publisher + subscriber pair per node.

    - Test communication with ros2 topic echo and ros2 topic pub.

    - Add one service and one action per node.

    - Launch all nodes using a launch file under harvesting_ws/src/orchestrator/launch/harvest_sim.launch.py.

    - Use rqt_graph or ros2 topic list to visualize the data flow.