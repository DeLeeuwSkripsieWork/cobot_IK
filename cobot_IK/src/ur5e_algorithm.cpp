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



void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(4);

    // Add roof boundary
    collision_objects[0].id = "roof_boundary";
    collision_objects[0].header.frame_id = "base_link";

    // Define primitive dimension, position of the roof boundary
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 2;
    collision_objects[0].primitives[0].dimensions[1] = 2;
    collision_objects[0].primitives[0].dimensions[2] = 0.1;
    // pose of roof boundary
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.0;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.7;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    // Add roof boundary to the scene
    collision_objects[0].operation = collision_objects[0].ADD;


    // Add the backwall boundary
    collision_objects[1].id = "backwall_boundary";
    collision_objects[1].header.frame_id = "base_link";

    // Define primitive dimension, position of the backwall boundary
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.2;
    collision_objects[1].primitives[0].dimensions[1] = 2;
    collision_objects[1].primitives[0].dimensions[2] = 0.8;
    // pose of backwall boundary
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = -0.4;
    collision_objects[1].primitive_poses[0].position.y = 0;
    collision_objects[1].primitive_poses[0].position.z = 0.3;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    // Add backwall boundary to the scene
    collision_objects[1].operation = collision_objects[1].ADD;





    // Add the object to be picked
    collision_objects[2].id = "Rightwall";
    collision_objects[2].header.frame_id = "base_link";

    // Define primitive dimension, position of the object
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 1;
    collision_objects[2].primitives[0].dimensions[1] = 0.2;
    collision_objects[2].primitives[0].dimensions[2] = 0.8;
    // pose of object
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.0;
    collision_objects[2].primitive_poses[0].position.y = 0.9;
    collision_objects[2].primitive_poses[0].position.z = 0.3;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    // Add tabe 2 to the object
    collision_objects[2].operation = collision_objects[2].ADD;



// Add the main table
    collision_objects[3].id = "main";
    collision_objects[3].header.frame_id = "base_link";

    // Define primitive dimension, position of the table main
    collision_objects[3].primitives.resize(1);
    collision_objects[3].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[3].primitives[0].dimensions.resize(3);
    collision_objects[3].primitives[0].dimensions[0] = 2.0;
    collision_objects[3].primitives[0].dimensions[1] = 2.0;
    collision_objects[3].primitives[0].dimensions[2] = 0.1;
    // pose of table main
    collision_objects[3].primitive_poses.resize(1);
    collision_objects[3].primitive_poses[0].position.x = 0;
    collision_objects[3].primitive_poses[0].position.y = 0;
    collision_objects[3].primitive_poses[0].position.z = -0.1;
    collision_objects[3].primitive_poses[0].orientation.w = 1.0;
    // Add tabe main to the scene
    collision_objects[3].operation = collision_objects[3].ADD;



    planning_scene_interface.applyCollisionObjects(collision_objects);

}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    sleep(2.0);

    moveit::planning_interface::MoveGroupInterface group("manipulator");
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());


   
    addCollisionObject(planning_scene_interface); //new
//new code position point 1 -> 1.5


// Target position point 1 start
    geometry_msgs::Pose target_pose2;
    tf2::Quaternion orientation2;
    orientation2.setRPY(-tau/2, tau, tau);
    target_pose2.orientation = tf2::toMsg(orientation2);
    target_pose2.position.x = -0.00;
    target_pose2.position.y = 0.6;
    target_pose2.position.z = 0.1;

    while(target_pose2.position.y>=0 && target_pose2.position.y<=0.6 && target_pose2.position.x>=0 && target_pose2.position.x<=0.6){
    group.setPoseTarget(target_pose2);

    // Visualize and move to target pose 2
    moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
    moveit::planning_interface::MoveItErrorCode success2 = group.plan(my_plan2);
    ROS_INFO("Visualizing plan 2: %s", success2.val ? "SUCCESS" : "FAILED");
   
        //group.move();

         if (success2 == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            // Execute the plan
            moveit::planning_interface::MoveItErrorCode execution_success = group.move();
            if (execution_success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                // Increment only if both planning and execution are successful
                target_pose2.position.y=target_pose2.position.y-0.1;
                target_pose2.position.x=target_pose2.position.x+0.1;
            }
        }
   

    ros::WallDuration(0.1).sleep();
    

}


//new code end 1.5 -> 2

    target_pose2.position.x = 0.6;
    target_pose2.position.y = 0.0;

    while(target_pose2.position.y>=-0.6 && target_pose2.position.y<=0 && target_pose2.position.x>=0 && target_pose2.position.x<=0.6){
    group.setPoseTarget(target_pose2);

    // Visualize and move to target pose 2
    moveit::planning_interface::MoveGroupInterface::Plan my_plan2;

    moveit::planning_interface::MoveItErrorCode success2 = group.plan(my_plan2);
    ROS_INFO("Visualizing plan 2: %s", success2.val ? "SUCCESS" : "FAILED");


       // group.move();
    if (success2 == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            // Execute the plan
            moveit::planning_interface::MoveItErrorCode execution_success = group.move();
            if (execution_success == moveit::planning_interface::MoveItErrorCode::SUCCESS)
            {
                // Increment only if both planning and execution are successful
                target_pose2.position.y=target_pose2.position.y-0.1;
                target_pose2.position.x=target_pose2.position.x-0.1;
            }
        }
   

    ros::WallDuration(0.1).sleep();
}

 //new code start 1.5 -> 2

    ros::shutdown();
    return 0;

}