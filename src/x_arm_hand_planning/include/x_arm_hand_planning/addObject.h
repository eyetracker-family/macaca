// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

void addBall(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
             const std::string& referenceLinks,
             const std::string& objectName,
             geometry_msgs::Pose& poses,
             double objectSize,
             int8_t action);
