#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <sstream>
#include <iostream>
#include <unistd.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "rosplan_action_interface/RPActionInterface.h"
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

namespace KCL_rosplan {
        class MoveToInterface: public RPActionInterface {
                private:
                                                
                public:
                        /* constructor */
                        MoveToInterface(ros::NodeHandle &nh);
                        
                        /* listen to and process action_dispatch topic */
                        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
        };
}
