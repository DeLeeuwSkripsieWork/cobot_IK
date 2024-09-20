#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
const double tau = 2 * M_PI;
/*
void close_gripper(moveit::planning_interface::MoveGroupInterface& move_gripper)
{
    move_gripper.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.785);
    move_gripper.move();
}
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(2.0);

    moveit::planning_interface::MoveGroupInterface group("manipulator");
    //moveit::planning_interface::MoveGroupInterface gripper("gripper");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    // Target position 1
    geometry_msgs::Pose target_pose1;
    tf2::Quaternion orientation;
    orientation.setRPY(tau/2, tau, -tau/2);
    target_pose1.orientation = tf2::toMsg(orientation);
    target_pose1.position.x = -0.0;
    target_pose1.position.y = -0.6;
    target_pose1.position.z = 0.1;
    group.setPoseTarget(target_pose1);

    

    // visualize the planning
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit::planning_interface::MoveItErrorCode success = group.plan(my_plan);
    ROS_INFO("visualizeing plan %s", success.val ? "":"FAILED");

    // move the group arm
    group.move();

    ros::WallDuration(1.0).sleep();
   // close_gripper(gripper);

//new code position 2


// Target position 2
    geometry_msgs::Pose target_pose2;
    tf2::Quaternion orientation2;
    orientation2.setRPY(-tau/2, tau, tau);
    target_pose2.orientation = tf2::toMsg(orientation2);
    target_pose2.position.x = -0.11;
    target_pose2.position.y = 0.5;
    target_pose2.position.z = 0.11;
    group.setPoseTarget(target_pose2);

    // Visualize and move to target pose 2
    moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
    moveit::planning_interface::MoveItErrorCode success2 = group.plan(my_plan2);
    ROS_INFO("Visualizing plan 2: %s", success2.val ? "SUCCESS" : "FAILED");
   // if (success2) {
        group.move();
   // }

    ros::WallDuration(1.0).sleep();
    //close_gripper(gripper);


//new code end

    ros::shutdown();
    return 0;

}