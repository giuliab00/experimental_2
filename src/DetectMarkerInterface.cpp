#include "my_rosplan_interface/DetectMarkerInterface.h"
#include <unistd.h>
#include <string>
#include "experimental_2/markerVision.h"

namespace KCL_rosplan {

    DetectMarkerInterface::DetectMarkerInterface(ros::NodeHandle &nh) {
        // here the initialization
        clientMarkerVision = nh.serviceClient<experimental_2::markerVision>("/markerVision");
    }

    bool DetectMarkerInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        // here the implementation of the action
        ROS_INFO("Searching marker %s ", msg->parameters[2].value.c_str());
        
        std::string marker_name = msg->parameters[2].value.c_str();
        int markerID;
        if(marker_name == "mk11"){
            markerID = 11;
        }else if(marker_name == "mk12"){
            markerID = 12;
        }else if(marker_name == "mk13"){
            markerID = 13;
        }else if(marker_name == "mk15"){
            markerID = 15;
        }
        
        ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
        return true;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_rosplan_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    KCL_rosplan::DetectMarkerInterface my_aci(nh);
    my_aci.runActionInterface();
    return 0;
}