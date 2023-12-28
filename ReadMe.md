Assignment 2 of Experimental Robotics Laboratory
================================================

The goal of this assignment is to develop a ROS package that lets a mobile robot endowed with a camera:
- Find all markers in the environment knowing the position they are visible from
- Go back to the initial position 

The assignment is implemented both in simulation  and with the real robot.

How to download
----------------------

In order to run the solution it is necessary to have the following ROS package:
* OpenCV: that must be the same version of your ROS, in our case *noetic*. 
```bash
git clone https://github.com/ros-perception/vision_opencv
git checkout noetic
```
* ArUco: in order to have the models of the marker in the simulation and all the libraries provided by ArUco to recognize the markers. The following is for ROS noetic
```bash
git clone https://github.com/CarmineD8/aruco_ros
```
In order to have the marker visible in the gazebo simulation move the folder **models** into the folder .\gazebo

* Rosbot: for the simulation is important to have the rosbot model. This can be obtained with the following lines. Once again remember to branch to the correnct implementation for your ROS version.
```bash
git clone https://github.com/husarion/rosbot_ros
git checkout noetic
```

* ROSPlan:it is needed for the planning, in fact it provides a set of tool for AI Planning integrated with the ROS framework.
```bash
sudo apt get update
(sudo) apt install flex bison freeglut3-dev libbdd-dev python3-catkin-tools ros-$ROS_DISTRO-tf2-bullet
git clone https://github.com/KCL-Planning/ROSPlan
```

add the string “-Wno-error=deprecated-copy” in line 92 of the CMakeLists.txt file from the rosplan_dependies package.
Then build the workspace

* gmapping: In order to avoid obstacles in the environment the rosbot creates a map of the environment while it goes by using a laser scan. This is done using the gmapping slam algorith of ROS.
```bash
sudo apt get update
sudo apt-get install libsuitesparse-dev
sudo apt-get install ros-<ros_distro>-openslam-gmapping
```

* MoveBase: to move the rosbot in the environment the MoveBase package of the ROS Navigation stack wich is a motion planner using both a local and global planner and avoiding obstacles.
```bash
sudo apt-get update
sudo apt-get install ros-<ros_distro>-navigation
```

* Lastly you can finally download our package and build the workspace. 
```bash
git clone https://github.com/giuliab00/experimental_2
```

SIMULATION 
------------------------------------

### How to run the solution

If you have followed the previous steps, it is possible to start the simulation using the following commands:

```bash
roslaunch experimental_2 sim_aruco2.launch
```
Once the simulation is open you can start the complete ehaviour of the rosbot, from the planning till the motion and the detection with the following line: 
```bash
roslaunch experimental_2 rosbot.launch
```
while this launches the complete behaviour of the rosbot, from the planning till the motion and the detection.

### PDDL
In order to make a plan it has been created in PDDL firstly a *domain* then a *problem* which will be later processed by a popf solver. The plan will be generated solved parsed and then dispatched.

#### Domain
``` bash
experimental_2/pddl/experimental_2.pddl
```
The domain involves the robot navigating through various waypoints, detecting markers, and performing actions such as moving between waypoints, returning and leaving home.

The domain incorporates different **types**, including:
- waypoints (locations the robot can visit); 
- a specific home location, introduced for a better performance;
- markers (objects the robot can detect).
 

**Predicates** are used to describe the robot's location, whether it has seen a marker, and the visibility of markers from specific waypoints.

The key actions are durative actions, meaning they have a specified duration:

- *move*: Rosbot moves from one waypoint to another in 30 time units.
- *go_home*: Rosbot returns home from a waypoint in 30 time units.
- *leave_home*: Rosbot leaves home and moves to a specified waypoint in 30 time units.
- *detect*: Rosbot detects a marker from a specific waypoint in 10 time units.

It may seem redundant, but the actions go and leave home have been introduced in order to make the plan artificially more efficient. Before this improvement the Rosbot was going to a waypoint and then back home before going to the next waypoint.

#### Problem
``` bash
experimental_2/pddl/problem_experimental_2.pddl
```

The objects in the problem include:
- home location (wp0);
- waypoints (wp1, wp2, wp3, wp4);
- markers (mk11, mk12, mk13, mk15). 

The initial conditions indicate that the robot starts at home (wp0), and various markers are initially visible from specific waypoints, as it was indicated in the slides.

The goal of the problem states that the robot must achieve the following:
- Detect markers mk11, mk12, mk13, and mk15.
- Return to the home location (wp0).

#### Plan
The resulting plan is at this directory:
``` bash
ROSPlan/rosplan_planning_system/common/plan.pddl
```
It contains the instructions that the Rosbot has to follow to achieve the goal.

### Architecture and Pseudocode


In order to achieve the solution it has been thought of the following architecture:
![experimental_architecture](https://github.com/giuliab00/experimental_2/assets/114082533/52ba3d31-bdad-400b-9363-d0b0ab892b1e)

In this architecture there are nodes used to found and dispatch the plan (knowledge Base, Problem Interface, Planner Interface, Parsing Interface, Plan Dispatcher and Action Interface) taken from the ROSPlan package. Moreover there are nodes to control the behaviour: 

The **PlanLauncher Node** start the planning procedure by making al the request to have in the end the dispatch of the plan. 

The **DetectMarker Action Interface** corresponds to the detect pddl action. When the detect action is dispatch it makes an Action Request to the findMarker node. The **findMarker Node** makes the robot rotate on himself until the marker isn't seen. To see the marker the **markerDetector Node** is used. 

The **MoveTo Action Interface** corresponds to three different pddl action (move, leave_home and go_home) and makes the robot move in the environment by sending the waypoint to reach to the **MoveBase Node**. 

Lastly the ActionService nodes(findMarker and MoveBase) comunicate either with the simulation environment or the actual rosbot publishing the command velocity to make the robot move.


#### planLauncher Node
This node named is responsible for coordinating and monitoring the execution of the plan. IT utilizes ROS services to generate, solve, parse, and dispatch plans. The PlanLauncher class encapsulates this functionality, with the main loop in the init_plan method ensuring the plan's success and goal achievement. The main function initializes the ROS node and calls the plan initialization method.
```    cpp
Include necessary libraries

Define PlanLauncher class
    def __init__(self):
        Initialize ROS NodeHandle

    def init_plan(self):
        Initialize services
        done = False

        while not done:
            Trigger services to start the plan
            Dispatch the plan

            Extract success and goal achievement status
            
def main():
    Create PlanLauncher instance
    Call the init_plan method
```
#### MoveToInterface Node
This node defines a ROS action interface for a robot to move to specified waypoints using the MoveBase action server.

```    cpp
Include necessary libraries

Define the MoveToInterface class within the KCL_rosplan namespace
     Constructor for the MoveToInterface class
    
    Implementation of the action in the concreteCallback method
    
        Setting target position and orientation based on the waypoint
        if wp0
            Set goal for waypoint 0
        else if wp1
            Set goal for waypoint 1
        else if wp2
            Set goal for waypoint 2
        else if wp3
            Set goal for waypoint 3
        else if wp4
            Set goal for waypoint 4
        
        Send goal to MoveBase
        sleep(1);
        waitForResult();
        return true;
   

def main():
    Initialize ROS node
    Create MoveToInterface instance and run the action interface
```

#### DetectMarkerInterface Node
This node ereditates from the Action Interface of ROSplan and is the one active when a detect action is dispatched from the plan. In particular it makes a request to the findMarker Action Server and wait for the result for 60 seconds. If the goal isn't reach in 60 seconds it return false otherwise true. 
In order to say that this node ereditates from the Action Interface one a header file has been written DetectMarkerInterface.h . This node have been developed in c++ and can be found in the src folder. 
```cpp
#include DetectMarkerInterface.h
#include unistd.h

Constructor 

bool concreteCallback(){
    get the id of the marker to found;
    create actionGoal goal;
    set the ID of the marker to found;
    create actionClient;
    send goal to ActionClienr
    wait result for 60 seconds;
    if(result){
        Action complete;
        return true;
    }
    else {
        Action did not complete;
        return false;
    }
}

main(){
    init Node
    start DetectMarkerInterface
    run ActionInterface

}
```

#### markerDetector Node
This node is the one recognizing marker and computing the values to tell the navlog about the distance between the rosbot and the marker. To detect the marker the ArUco marker detector has been used, then if the marker to found has been detect the distance between the marker center and the camera center and the dimension of pixel of the side of the marker are computed. To comunicate with the navlog Node a custom message is published containing, the id of the marker found, an ack, the size of the side and the distance between centers. This node has been implemneted in c++ due to the ArUco library be mainl written in c++.  (found in the src folder with the name markerDetctor.cpp)

```    cpp
Include needed library
Define a MarkerDetector Class {
    Initialize variable for aruco marker detection (detector, marker size, camera parameter), CV image,  
    Initialize publishers and subscriber

    image_callback() function {
        create a cv_bridge
        try{
            copy image
            get center of the camera
            clear detected_markers list
            detect marker
            for(marker in detected_markers){
                draw detected marker on image
                if(marker.id == marker_to_ found.id){
                    get center of the marker
                    compute distance between marker and camera center along x axis
                    compute marker size dimension in pixel
                    set msg field
                    pusblish(msg)
                }
            }
            visualize Camera POV 
        }
        catch{
            error
        }
    }

    find_marker_callback() function{
        set the id of the marker to found to the received one
    }

    camera_info_callback() function {
        set the Aruco Camera Parameter from camera info
   }

   exit_callback() function {
        shutdown the node
   }

}
main(){
    init ros Node
    create markerDetector node
    ros spin node
}

```

#### findMarkerAction Node
This node implement and Action Server that upon request make the robot turn on himself until the marker to found is not seen. To recognize the marker it publish the ID of the marker to found to markerDetector and wait for it to recognize. When MarkerDetector found the marker it will publish the information on the topic to which findMarker is subscribed and so findMarker stop the robot and Action is done. This code have been written in python and can be found in the scripts folder. 
```    python

import needed

class finMarker(object):
    Constructor:
        Initialize variables
        Initialize actionServer as
        Initialize publisher and subscribers

    Callback for markerPose subscription:
        if(marker found  == marker to found):
            Acknowledge marker detection
        else
            marker not found 

    Action Callback:
        publish(marker to found)
        while(marker not found)
            robot turn on itself
            publish ActionFeedback time elapsed
        stop 
        set ActionResult = True
        publish ActionResult

main():
    start node
    create server
	rospy.spin()


```

### Video
Here it is possible to find the video showing the behaviour in the simulation:


https://github.com/giuliab00/experimental_2/assets/114082533/923899ea-85ce-4414-8390-6cddc93c2ce1



ROSBOT REAL LIFE
------------------------------------
### What changes
The behavipur of the robot and the robot architecture remain the same. Nevertheless, in order to run the solution with the rosbot some slight changes must be applied.
First of all the positions of the waypoint are surely different from the simulation. The waypoint have been placed into the lab in correspondance of the marker that have been hang. Then after deciding the placement of the starting waypoint (0,0) all the coordinates have been roughly measured as follows:

- marker 11 is visible from the position x = 2.2, y = 1.8
- marker 12 is visible from the position x = -2.4, y = 1.0
- marker 13 is visible from the position x = -1.2, y = 1.0
- marker 15 is visible from the position x = 2.4, y = 1.0

Another important change that has been implemented is the maximum obstacle height detected, found in the costmap common parameters, that has been increased to 0.6 in order to map all the obstacles.

### How to run the solution

By sharing the roscore with the rosbot, after enabling it already in the starting waypoint, it will be needed to launch the following command:

```bash
roslaunch experimental_2 rosbot.launch check tabi
```

### Video
Here it is possible to find the video showing the behaviour in the lab

https://github.com/giuliab00/experimental_2/assets/114100814/03702977-c51c-442b-aa60-66129e1c62c8

Drawback and Possible improvements
-------------------------
It is worth to underline how we took inspiration from the previous assignment possible improvement making the whole project more modular. 
Nevertheless there are possible improvement regarding both the simulation and the real rosbot.
One of them would concern dispatching again wether the plan fails. 
