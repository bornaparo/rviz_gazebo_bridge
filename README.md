## Rviz <-> Gazebo bridge package

Package that allows you to spawn all collision objects from moveit planning scene to gazebo ignition, and to load all spawned gazebo ignition models as collision objects into moveit planning scene 
 
### Usage
```cpp
#include "rviz_gazebo_bridge/rviz_gazebo_bridge.h"
...
std::string robot_name = "baxter"; //name of your robot model
auto rviz_gazebo_bridge = std::make_shared<RVizGazeboBridge>(robot_name);
rviz_gazebo_bridge->load_from_rviz_to_gazebo();
rviz_gazebo_bridge->load_from_gazebo_to_rviz();
```