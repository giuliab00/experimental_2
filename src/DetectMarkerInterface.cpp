#include "../include/DetectMarkerInterface.h"
#include "experimental_2/ackKill.h"
#include <unistd.h>

/*
	This node is used as the action interface for the detect action of our plan
	In this action interface, an action client that calls for a find marker action is implemented
	
	Actions: - /findMarkerAction :		It is used to search for a marker with a specific ID
 */

namespace KCL_rosplan {

    // Initialization
    DetectMarkerInterface::DetectMarkerInterface(ros::NodeHandle &nh) {
        
    }
    
    void killCallback(const experimental_2::ackKill::ConstPtr& msg) {
        ros::shutdown();
    }

    /*
        This function concretely implement the detect action
        By getting parameters value from the dispatched plan, it is possible to see if a marker ID is present
        If that is the case, it means that the robot should rotate on itself in order to find that marker
        So we set the marker ID as a goal for the find marker action and then call the correspondant action server
    */
    bool DetectMarkerInterface::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {
        ROS_INFO("Searching marker %s ", msg->parameters[1].value.c_str());
        
        // Get the value of parameters[1] and check if it contains a marker ID 
        std::string marker_name = msg->parameters[1].value.c_str();
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

        // Set the marker ID found as goal for the find marker action
        experimental_2::findMarkerGoal goal;
        goal.markerId = markerID;

        // Action client initialization
        actionlib::SimpleActionClient<experimental_2::findMarkerAction> ac("/findMarkerAction", true);
        ROS_INFO("Waiting for action server to start.");
        ac.waitForServer();    
        ROS_INFO("Action server /findMarkerAction available");
        
        // Sending the marker ID to the action server
        ac.sendGoal(goal);
        sleep(1);
        
        // We wait one minute for the completition of the action
        bool res = ac.waitForResult(ros::Duration(60.0));
        
        // We check whether the action succeded or failed and then we print a message to let the user know about it
        if(res){
                ROS_INFO("Action (%s) performed: completed!", msg->name.c_str());
                return true;
        }else{
                ROS_INFO("Action (%s): failed", msg->name.c_str());
                return false; 
        }
        return true;

    }
}

int main(int argc, char **argv) {

    // Node initialization
    ros::init(argc, argv, "my_rosplan_action", ros::init_options::AnonymousName);
    ros::NodeHandle nh("~");
    
    ros::Subscriber sub1 = nh.subscribe("/kill_nodes",1,KCL_rosplan::killCallback);
    
    // Action interface initialization
    KCL_rosplan::DetectMarkerInterface my_aci(nh);
    my_aci.runActionInterface();
    return 0;
}
