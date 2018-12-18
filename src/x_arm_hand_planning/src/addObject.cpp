#include "../include/x_arm_hand_planning/addObject.h"

void addBall(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
             const std::string& referenceLinks,
             const std::string& objectName,
             geometry_msgs::Pose& poses,
             double objectSize,
             int8_t action)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id =objectName;
  collision_objects[0].header.frame_id = referenceLinks;

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[1].primitives[0].SPHERE;
  collision_objects[0].primitives[0].dimensions.resize(1);
  collision_objects[0].primitives[0].dimensions[0] = objectSize;

  /* Define the pose of the object. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = poses.position.x;
  collision_objects[0].primitive_poses[0].position.y = poses.position.y;
  collision_objects[0].primitive_poses[0].position.z = poses.position.z;
  collision_objects[0].primitive_poses[0].orientation.w = poses.orientation.w;
  collision_objects[0].primitive_poses[0].orientation.x = poses.orientation.x;
  collision_objects[0].primitive_poses[0].orientation.y = poses.orientation.y;
  collision_objects[0].primitive_poses[0].orientation.z = poses.orientation.z;
  // END_SUB_TUTORIAL
/*  enum {
    ADD = 0,
    REMOVE = 1,
    APPEND = 2,
    MOVE = 3,
  };
*/
  collision_objects[0].operation = action;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}
