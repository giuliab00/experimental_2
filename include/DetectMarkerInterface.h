#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <experimental_2/findMarkerAction.h>
#include "rosplan_action_interface/RPActionInterface.h"
#include <unistd.h>
#include <string>

namespace KCL_rosplan {
        class DetectMarkerInterface: public RPActionInterface {
                private:
                        ros::ServiceClient clientMarkerVision;                                             
                public:
                        /* constructor */
                        DetectMarkerInterface(ros::NodeHandle &nh);
                        
                        /* listen to and process action_dispatch topic */
                        bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
        };
}
