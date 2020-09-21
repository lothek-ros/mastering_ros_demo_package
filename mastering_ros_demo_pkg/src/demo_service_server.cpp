#include "ros/ros.h"
//generated header, contains our service definition and we can use this in our code
#include "mastering_ros_demo_pkg/demo_srv.h"
#include <iostream>
#include <sstream>	//for getting string streaming classes
using namespace std;

//this is the server callback fuction executed when a request is received on the server.
//the server can receive the request from clients with a message type of req & res
bool demo_service_callback(
  mastering_ros_demo_pkg::demo_srv::Request &req,
  mastering_ros_demo_pkg::demo_srv::Response &res ) {
	std::stringstream ss;
	//"Received Here" is passing to the service response instance.
	ss << "Received Here";		
	//out is the field name of the response that we have given in demo_srv.srv
	//this response will to to the service client node
	res.out = ss.str();		
	ROS_INFO("From Client [%s], Server says [%s]", req.in.c_str(), res.out.c_str() );
}

int main (int argc, char **argv) {
	ros::init(argc, argv, "demo_service_server");
	ros::NodeHandle n;
	//this creates a service called demo_servie and a callback function is executed when a req comes to this service. the callback function is demo_service_callback
	ros::ServiceServer service = n.advertiseService ("demo_service", demo_service_callback);
	ROS_INFO("Ready to receive from client.");
	ros::spin();
	return 0;
}
	
