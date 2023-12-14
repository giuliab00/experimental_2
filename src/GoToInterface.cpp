#include <unistd.h>
#include "GoToInterface.h"

namespace KCL_rosplan {

	GoToInterface::GoToInterface(ros::NodeHandle &nh) {
		// here the initialization
	}
	
	bool GoToInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
		// here the implementation of the action
		std::cout << "Going from " << msg->parameters[1].value << " to " << msg->parameters[2].value << std::endl;
		actionlib::SimpleActionClient<motion_plan::PlanningAction> ac("/reaching_goal", true);
		motion_plan::PlanningGoal goal;
		ac.waitForServer();
		if(msg->parameters[2].value == "wp0"){
			goal.target_pose.pose.position.x = 0.0;
			goal.target_pose.pose.position.y = 0.0;
			goal.target_pose.pose.orientation.w = 0.0;
		}
		else if (msg->parameters[2].value == "wp1"){
			goal.target_pose.pose.position.x = 6.0;
			goal.target_pose.pose.position.y = 2.0;
			goal.target_pose.pose.orientation.w = 0.0;
		}
		else if (msg->parameters[2].value == "wp2"){
			goal.target_pose.pose.position.x = 7.0;
			goal.target_pose.pose.position.y = -5.0;
			goal.target_pose.pose.orientation.w = 0.0;
		}
		else if (msg->parameters[2].value == "wp3"){
			goal.target_pose.pose.position.x = -3.0;
			goal.target_pose.pose.position.y = -8.0;
			goal.target_pose.pose.orientation.w = 0.0;
		}
		else if (msg->parameters[2].value == "wp4"){
			goal.target_pose.pose.position.x = -7.0;
			goal.target_pose.pose.position.y = -1.5;
			goal.target_pose.pose.orientation.w = 0.0;
		}
		ac.sendGoal(goal);
		ac.waitForResult();
		ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
		return true;
	}
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "my_rosplan_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	KCL_rosplan::GoToInterface my_aci(nh);
	my_aci.runActionInterface();
	return 0;
}
