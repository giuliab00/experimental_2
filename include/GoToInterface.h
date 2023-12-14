#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <motion_plan/PlanningAction.h>
#include "rosplan_action_interface/RPActionInterface.h"

namespace KCL_rosplan {
        class GoToInterface: public RPActionInterface {
                private:
                                                
                public:
                        /* constructor */
                        GoToInterface(ros::NodeHandle &nh);
                        
                        /* listen to and process action_dispatch topic */
                        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
        };
}
