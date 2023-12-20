#include "../include/DetectMarkerInterface.h"
#include <unistd.h>

namespace KCL_rosplan {

    DetectMarkerInterface::DetectMarkerInterface(ros::NodeHandle &nh) {
        // here the initialization
        
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

        experimental_2::findMarkerGoal goal;
        goal.markerId = markerID;

        /*JUST TO MAKE THE THING WORKS*/
        actionlib::SimpleActionClient<experimental_2::findMarkerAction> ac("/findMarkerAction", true);
        ROS_INFO("Waiting for action server to start.");
        ac.waitForServer();    
        ROS_INFO("Action server /findMarkerAction available");

        ac.sendGoal(goal);
        
        bool res = ac.waitForResult(ros::Duration(60.0));
        if(res){
                ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
                return true;
        }else{
                ROS_INFO("Action (%s): failed", msg->name.c_str());
                return false; 
        }

    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_rosplan_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    ROS_INFO("Belin siamo in DetectMarkerInterface");
    KCL_rosplan::DetectMarkerInterface my_aci(nh);
    my_aci.runActionInterface();
    return 0;
}
