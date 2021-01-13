#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal create_goal(const float x, const float y, const float qz=0.0, const float qw=1.0, const double stamp=0.0){
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now() + ros::Duration(stamp);

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.z = qz;
  goal.target_pose.pose.orientation.w = qw;

  return goal;
}
// 0.7071

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  goal = create_goal(8.72774124146,1.64151453972,0.0,1.0,0.0); // Table room, pick up coordinate 

  // Send the goal position and orientation for the robot to reach
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    ROS_INFO_STREAM("Pick up zone achieved!!! Position-> x:" << goal.target_pose.pose.position.x <<" y:"<< goal.target_pose.pose.position.y << std::endl );
    
    ros::Duration(5.0).sleep();
    
    goal = create_goal(1.11344361305,1.08418285847,1.0,0.0,0.0); // Drop off the object on top of other robot
   
    // Send the goal position and orientation for the robot to reach
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO_STREAM("Drop off zone achieved!!! Position-> x:" << goal.target_pose.pose.position.x <<" y:"<< goal.target_pose.pose.position.y << std::endl );
    }
    else{
      ROS_INFO_STREAM("The robot failed moving to drop off zone at position-> x:" << goal.target_pose.pose.position.x <<" y:"<< goal.target_pose.pose.position.y << std::endl );
    }
  }
  else {
    ROS_INFO_STREAM("The robot failed moving to pick up zone at position-> x:" << goal.target_pose.pose.position.x <<" y:"<< goal.target_pose.pose.position.y << std::endl );
  }
  ros::Duration(5.0).sleep(); // give some time to check console and marker end
  return 0;
}
