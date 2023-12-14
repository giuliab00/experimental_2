#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <cmath>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include "std_srvs/Empty.h"
#include "rosplan_dispatch_msgs/DispatchService.h"

class PlanLauncher {
	
	private:
	
		ros::NodeHandle nh_;
	
	public:
	
		PlanLauncher() : nh_("~") {
			
		}
		
		bool init_plan() {
		
			std_srvs::Empty srv1;
			std_srvs::Empty srv2;
			rosplan_dispatch_msgs::DispatchService srv3;
			
			bool done = false;
		
			// init two services
			ros::ServiceClient prob_gen = nh_.serviceClient<std_srvs::Empty>("/rosplan_problem_interface/problem_generation_server");
			ros::ServiceClient prob_solve = nh_.serviceClient<std_srvs::Empty>("/rosplan_planner_interface/planning_server");
			ros::ServiceClient plan_disp = nh_.serviceClient<rosplan_dispatch_msgs::DispatchService>("/rosplan_plan_dispatcher/dispatch_plan");
			
			// TODO: CHANGE THE CONDITION OF THE WHILE WITH A VARIABLE AMONG done AND goal
			while (not done) {
			
				// call the two services to start the plan
				prob_gen.call(srv1);
			
				prob_solve.call(srv2);
			
				plan_disp.call(srv3);
			
				// get info about if we succeeded or failed
				done = srv3.response.success;
				bool goal = srv3.response.goal_achieved;
				
				ROS_INFO("My boolean value: %s", done ? "true" : "false");
				ROS_INFO("My boolean value: %s", goal ? "true" : "false");
			
			}
		}
	
};

int main(int argc, char **argv){
	// init ros Node
	ros::init(argc, argv, "plan_launcher");	
	
	// create PlanLauncher
	PlanLauncher node;
	
	// call PlanLauncher method
	node.init_plan();
	
}
