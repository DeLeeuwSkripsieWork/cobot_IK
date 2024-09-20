#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rotate_ur5e_base_node");
    ros::NodeHandle nh;

    // Initialize MoveGroup interface for the UR5e arm
    static const std::string PLANNING_GROUP = "manipulator";  // UR5e's MoveIt group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // Define the joint values for all joints (use desired values)
    std::vector<double> joint_group_positions = {M_PI/2, -M_PI/4, M_PI/2, M_PI+(M_PI/4), -M_PI/2, 0.0}; // Adjust as needed

    // Set the base joint (shoulder_pan_joint) to a desired angle (e.g., 45 degrees)
    double target_angle_in_radians = M_PI / 4;  // Example: 45 degrees rotation therefore M_PI = 180 degrees
    joint_group_positions[0] = target_angle_in_radians;  // Base joint (shoulder_pan_joint)

    // Set the new joint values as the target
    move_group.setJointValueTarget(joint_group_positions);

    // Plan and execute the motion
    move_group.move();  // This both plans and executes

    ROS_INFO("Successfully rotated the base.");

    ros::shutdown();
    return 0;
}