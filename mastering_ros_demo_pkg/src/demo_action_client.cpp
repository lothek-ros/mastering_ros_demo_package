#include "ros/ros.h"
#include <iostream>
//in the action client, we need to include this 3 to get the action client APIs, which are used to implement action clients
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "mastering_ros_demo_pkg/Demo_actionAction.h"

int main (int argc, char **argv) {
	ros::init(argc, argv, "demo_action_client");

	if(argc != 3){
		ROS_INFO("%d",argc);
		ROS_WARN("Usage: demo_action_client <goal> <time_to_preempt_in_sec>");
		return 1;
	}

	// create the action client
	// true causes the client to spin its own thread
	//this will create an action client instance
	actionlib::SimpleActionClient<mastering_ros_demo_pkg::Demo_actionAction> ac("demo_action", true);

	ROS_INFO("Waiting for action server to start.");

	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");

	// send a goal to the action
	//this line will wait for an infinite time if theres is no action server running on the system
	//it will exit only when there is an action server running on the system
	mastering_ros_demo_pkg::Demo_actionGoal goal;
	goal.count = atoi(argv[1]);
	ROS_INFO("Sending Goal [%d] and Preempt time of [%d]",goal.count, atoi(argv[2]));
	ac.sendGoal(goal);

	//wait for the action to return
	//create an instance of a goal and send the goal value from the frist command line argument
	bool finished_before_timeout = ac.waitForResult(ros::Duration(atoi(argv[2])));
	//Preempting task
	ac.cancelGoal();

	//this line will wait for the result from the server until the given seconds
	if (finished_before_timeout) 	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
		//Preempting the process
	
		//if it is not finished, it will preempt the action
		ac.cancelGoal();

	}
	else
		ROS_INFO("Action did not finish before the time out.");

	//exit
	return 0;
}
