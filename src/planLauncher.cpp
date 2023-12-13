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
			rosplan_dispatch_msgs::DispatchService srv2;
			std_srvs::Empty srv3;
		
			// init two services
			ros::ServiceClient prob_gen = nh_.serviceClient<std_srvs::Empty>("//rosplan_problem_interface/problem_generation_server");
			ros::ServiceClient plan_disp = nh_.serviceClient<rosplan_dispatch_msgs::DispatchService>("/rosplan_plan_dispatcher/dispatch_plan");
			ros::ServiceClient prob_solve = nh_.serviceClient<std_srvs::Empty>("/rosplan_planner_interface/planning_server");
			
			// call the two services to start the plan
			prob_gen.call(srv1);
			
			plan_disp.call(srv2);
			
			prob_solve.call(srv3);
			
			// get info about if we succeeded or failed
			bool done = srv2.response.success;
			bool goal = srv2.response.goal_achieved;
			
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
