#include "ros/ros.h" 		// main header of ROS "asdf"
#include <std_msgs/Int32.h>	//for handling the integer data message type
#include <iostream>		

int main (int argc, char **argv){
	//initial a ROS node name
	ros::init(argc, argv, "demo_topic_publisher");	
	//use to communicate with the ROS system
	ros::NodeHandle node_obj;				
	//create a topic publisher, name the topic /numbers with msgs type std_msgs. 
	//number 10 its the buffer msgs before sending
	ros::Publisher number_publisher = node_obj.advertise<std_msgs::Int32>("/numbers",10);
	
	//freq of sending the data
	ros::Rate loop_rate(10); 	
	int number_count = 0;		
	while (ros::ok() ) {
		//creates an intger ROS message
		std_msgs::Int32 msg;	
		//assigns an integer value to the message	
		msg.data = number_count;
		//log the ROS information	
		ROS_INFO("%d",msg.data);
		//publish the message to the topics /numbers	
		number_publisher.publish(msg); 
		//read and update all ROS topics. the node will not publish without a spin() or spinOnce()
		ros::spinOnce();
		//provide the necessary delay to achieve a freq of 10Hz		
		loop_rate.sleep();		
		++number_count;
	}
	return 0;
}
