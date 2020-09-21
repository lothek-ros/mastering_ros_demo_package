#include "ros/ros.h"
#include "mastering_ros_demo_pkg/demo_srv.h"
#include <iostream>
#include <sstream>
using namespace std;

int main (int argc, char **argv) {
	ros::init(argc, argv, "demo_service_client");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	//this line creates a service client that has the message type mastering_ros_demo_pkg::demo_srv
	//and communicate to a ROS service named demo_service
	ros::ServiceClient client = n.serviceClient<mastering_ros_demo_pkg::demo_srv>("demo_service");
	while (ros::ok()) {
		mastering_ros_demo_pkg::demo_srv srv;
		//this line will create a new service object instance
		std::stringstream ss;
		ss << "Sending from here";
		srv.request.in = ss.str();
		
		if (client.call(srv) ) { //Fill the req instance with a string called "Sending from Here"
			//this will send the service call to the server
			//if it is sent succesfully, it will print the response and req
			//if it failed, it will do nothing
			ROS_INFO("From Client [%s], Server says [%s]", srv.request.in.c_str(), srv.response.out.c_str() );
		}
		else {
			ROS_ERROR("Failed to call service:");
			return 1;
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
