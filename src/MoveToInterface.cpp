#include <unistd.h>
#include "../include/MoveToInterface.h"

namespace KCL_rosplan {

	MoveToInterface::MoveToInterface(ros::NodeHandle &nh) {
		// here the initialization
	}
	
	bool MoveToInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		std::cout << "Going from " << msg->parameters[0].value << " to " << msg->parameters[1].value << std::endl;
		actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("/move_base", true);
		move_base_msgs::MoveBaseGoal goal;
		ac.waitForServer();
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
		ac.sendGoal(goal);
		sleep(30);
		ac.waitForResult();
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "my_rosplan_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::MoveToInterface my_aci(nh);
	my_aci.runActionInterface();
	return 0;
}
