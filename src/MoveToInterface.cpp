#include <unistd.h>
#include "experimental_2/ackKill.h"
#include "../include/MoveToInterface.h"

/*
	This node is used as the action interface for the movement actions of our plan
	In this action interface, movement between waypoints is implemented to be done in the simulation
	
	Actions: - /move_base :	it allows the robot to navigate towards a goal 
	        				in an environment with obstacles, allowing it to use both a local and a global planner
*/

namespace KCL_rosplan {

	// Initialization
	MoveToInterface::MoveToInterface(ros::NodeHandle &nh) {
		
	}
	
	void killCallback(const experimental_2::ackKill::ConstPtr& msg) {
		ros::shutdown();
	}
	
	/*
		This function concretely implement the movement actions
		By getting parameters value from the dispatched plan, it is possible to see if a waypoint name is present
		If that is the case, it means that the robot should move to that waypoint
		So we set the goal position to the marker position in the environment and send it to the move_base action server
 	*/
	bool MoveToInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
		
		// Initialization of action client for move_base action
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("/move_base", true);
		move_base_msgs::MoveBaseGoal goal;
		ac.waitForServer();
		
		// If a waypoint name is present in the parameters[1], the goal for the robot
		// is set at the correspondant position of that waypoint
		if(msg->parameters[1].value == "wp0"){
			goal.target_pose.pose.position.x = 0.0;
			goal.target_pose.pose.position.y = 1.0;
			goal.target_pose.pose.orientation.w = 1.0;
			goal.target_pose.header.frame_id = "map";
		}
		else if (msg->parameters[1].value == "wp1"){
			goal.target_pose.pose.position.x = 6.0;
			goal.target_pose.pose.position.y = 2.0;
			goal.target_pose.pose.orientation.w = 1.0;
			goal.target_pose.header.frame_id = "map";
		}
		else if (msg->parameters[1].value == "wp2"){
			goal.target_pose.pose.position.x = 7.0;
			goal.target_pose.pose.position.y = -5.0;
			goal.target_pose.pose.orientation.w = 1.0;
			goal.target_pose.header.frame_id = "map";
		}
		else if (msg->parameters[1].value == "wp3"){
			goal.target_pose.pose.position.x = -3.0;
			goal.target_pose.pose.position.y = -8.0;
			goal.target_pose.pose.orientation.w = 1.0;
			goal.target_pose.header.frame_id = "map";
		}
		else if (msg->parameters[1].value == "wp4"){
			goal.target_pose.pose.position.x = -7.0;
			goal.target_pose.pose.position.y = 1.5;
			goal.target_pose.pose.orientation.w = 1.0;
			goal.target_pose.header.frame_id = "map";
		}
		
		// The goal is sent to the move_base action server
		ac.sendGoal(goal);
		sleep(1);
		ac.waitForResult();
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}
}


int main(int argc, char **argv) {

	// Node initialization
	ros::init(argc, argv, "my_rosplan_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	
	ros::Subscriber sub1 = nh.subscribe("/kill_nodes",1,KCL_rosplan::killCallback);
	
	// Action Interface initialization
	KCL_rosplan::MoveToInterface my_aci(nh);
	my_aci.runActionInterface();
	return 0;
}
