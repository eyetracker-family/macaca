#include "../include/x_arm_hand_planning/collisionCheck.h"

 collision_detection::CollisionResult collisionCheck(void)
{
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    planning_scene.checkSelfCollision(collision_request, collision_result);

    ROS_INFO_STREAM("1. Self collision Test: "<< (collision_result.collision ? "in" : "not in")
                    << " self collision");

    return collision_result;
}
