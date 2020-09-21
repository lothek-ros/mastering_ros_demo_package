#include "ros/ros.h"
#include "std_msgs/Int32.h"
//this header is the standard action library to implement an action server node
#include <actionlib/server/simple_action_server.h>

//this header is generated from the stored action files. it should include accessing our action definition
#include "mastering_ros_demo_pkg/Demo_actionAction.h"

#include <iostream>
#include <sstream>

//this class contains the action server definition
class Demo_actionAction {
	protected:
		ros::NodeHandle nh_;
		// NodeHandle instance must be created before this line. Otherwise strange error may occur.
		
		//create a simple action server instance with our custom action message type
		actionlib::SimpleActionServer<mastering_ros_demo_pkg::Demo_actionAction> as; 
		// create messages that are used to published feedback/result
		//create a feddback instance for sending feedback during the operation
		mastering_ros_demo_pkg::Demo_actionFeedback feedback;
		//create a result instance for sending the final result
		mastering_ros_demo_pkg::Demo_actionResult result;

		std::string action_name;
		int goal;
		int progress;

	public:
		//this is an action constructor, and an action server is created here by taking an argument such as
		//Nodehandle, action_name, and executeCB, where executeCB is the action callback
		//where all the processing is done
		Demo_actionAction(std::string name) :
			as(nh_, name, boost::bind(&Demo_actionAction::executeCB, this, _1), false),
			action_name(name) {
				
				//this line registers a callback when the action is preempted.
				//the preemtCB is the callback name executed when there is a preempt request from the action client
				as.registerPreemptCallback(boost::bind(&Demo_actionAction::preemptCB, this));
				
				as.start();
		}

		~Demo_actionAction(void) {
			//empty line
		}

		void preemptCB(){
			ROS_WARN("%s got preempted!", action_name.c_str());
			result.final_count = progress;
			as.setPreempted(result,"I got Preempted"); 

		}
		//this is the callback definition which is executed when the action server receives a goal value
		//it will execute callback functions only after checking
		//whether the action server is currently active or it is preempted already
		void executeCB(const mastering_ros_demo_pkg::Demo_actionGoalConstPtr &goal) {
			if(!as.isActive() || as.isPreemptRequested()) return;
			
			ros::Rate rate(5);
			ROS_INFO("%s is processing the goal %d", action_name.c_str(), goal->count);
			
			//this loop will execute untul the goal value is reached.
			//it will continuously send the current progress as feedback
			for(progress = 1 ; progress <= goal->count; progress++){
			
			if(!ros::ok()){		//Check for ros
				result.final_count = progress;
				as.setAborted(result,"I failed !");
				ROS_INFO("%s Shutting down",action_name.c_str());
				break;
			}
			
			//inside this loop, it will check whther the action server is active or it is preempted
			//if it occurs, the function will return
			if(!as.isActive() || as.isPreemptRequested()){
				return;
			}
			
			//if the current value reaches the goal value, then it publishes the final result
			if(goal->count <= progress){
				ROS_INFO("%s Succeeded at getting to goal %d", action_name.c_str(), goal->count);
				result.final_count = progress;
				as.setSucceeded(result);
			}
			else{
				ROS_INFO("Setting to goal %d / %d",feedback.current_number,goal->count);
				feedback.current_number = progress;
				as.publishFeedback(feedback);
			}
			rate.sleep();
			}	
		}
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "demo_action");
	ROS_INFO("Starting Demo Action Server");
	
	//in main(), we create an instance of Demo_action_Action, which will start the action server
	Demo_actionAction demo_action_obj(ros::this_node::getName());
	ros::spin();
	return 0;
}
