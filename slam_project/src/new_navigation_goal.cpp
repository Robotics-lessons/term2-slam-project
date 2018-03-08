#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    ros::init(argc, argv, "new_navigation_goal");

  	// Spin a thread
  	MoveBaseClient ac("move_base", true);

  	// Wait for the action server to come up
  	ROS_INFO("Waiting for the move_base action server");
  	ac.waitForServer(ros::Duration(5));
  
  	ROS_INFO("Connected to move_base server");

  	move_base_msgs::MoveBaseGoal goal;

	// Send goal pose
    goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	// Do NOT modify the following for final submission.
  	goal.target_pose.pose.position.x = 0.995; 
	goal.target_pose.pose.position.y = -2.99;

  	goal.target_pose.pose.orientation.x = 0.0;
        goal.target_pose.pose.orientation.y = 0.0;
        goal.target_pose.pose.orientation.z = 0.0;
        goal.target_pose.pose.orientation.w = 1.0;


        ROS_INFO("Sending goal.");
        ROS_INFO("count time (seconds)...");
  	ac.sendGoal(goal);

        time_t start,end;
        time(&start);        
//        ros::Time lasttime=ros::Time::now();
  	ac.waitForResult();

  	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    	   ROS_INFO("Excellent! Your robot has reached the goal position.");
  	else
    	ROS_INFO("The robot failed to reach the goal position");

//        ros::Time currtime=ros::Time::now();
//        ros::Duration diff=currtime-lasttime;
        time(&end);
        long elapsed_secs = difftime(end,start);

//        long seconds = diff.toSec();
        int dmm=elapsed_secs/60;
        int dss=fmod(elapsed_secs,60);

        ROS_INFO("Time Elapsed : total %ld secs", (long)elapsed_secs);
        ROS_INFO("ROS Time Elapsed : %d mins, %d secs", (int)dmm, (int)dss);

  	return 0;
}
