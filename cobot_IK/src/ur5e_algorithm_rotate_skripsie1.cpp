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


    std::vector<double> joint_group_positions = {M_PI/2, -M_PI/4, M_PI/2, M_PI+(M_PI/4), -M_PI/2, 0.0}; // Adjust as needed




while(joint_group_positions[0] >= (-M_PI/2) ){
    group.setJointValueTarget(joint_group_positions);

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
                joint_group_positions[0]=joint_group_positions[0]-0.1;
                
            }
        }
   

    ros::WallDuration(0.1).sleep();
    

}


    ros::shutdown();
    return 0;
}