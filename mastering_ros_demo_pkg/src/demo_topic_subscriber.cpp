#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>

void number_callback(const std_msgs::Int32::ConstPtr &msg) {
	//callback function that wil execute whenever a adata comes to the /numbers topic
	//whenever a data reaches this topic, the function will call and extract the value and print it on the console
	ROS_INFO("Received [%d]:", msg->data);
}
int main(int argc, char **argv) {
	ros::init (argc, argv, "demo_topic_subscriber");
	ros::NodeHandle node_obj;
	
	//topic name neede to subscribe, the buffer size, and the callback function.
	ros::Subscriber number_subscriber = node_obj.subscribe("/numbers",10,number_callback);
	
	//this is an infinite loop in which the node will wait in this step.
	//this code will fasten the callbacks whenever a data reaches the topic
	ros::spin();
	return 0;
}	
