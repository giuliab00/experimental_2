#include <DetectMarkerInterface.h>
#include <unistd.h>
#include "experimental_2/markerVision.h"
#include "ros/ros.h"

namespace KCL_rosplan {

    DetectMarkerInterface::DetectMarkerInterface(ros::NodeHandle &nh) {
        // here the initialization
        clientMarkerVision = nh.serviceClient<experimental_2::markerVision>("/markerVision");

        ROS_INFO("waiting for /markerVision service...");
        if(!clientMarkerVision.waitForExistence(ros::Duration(5.0))){
                ROS_ERROR("Service /markerVision not avaible");                
        }
                
        ROS_INFO("service /markerVision available");
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
        experimental_2::markerVision MarkerVisionSrv;
        MarkerVisionSrv.request.markerID = markerID;
        if(clientMarkerVision.call(MarkerVisionSrv)){
            ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
            return true;
        }

        ROS_INFO("Action (%s): failed", msg->name.c_str());
        return false;      
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_rosplan_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    KCL_rosplan::DetectMarkerInterface my_aci(nh);
    my_aci.runActionInterface();
    return 0;
}
