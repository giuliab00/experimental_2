Assignment 2 of Experimental Robotics Laboratory
================================================

The goal of this assignment is to develop a ROS package that lets a mobile robot endowed with a camera:
- Find all markers in the environment knowing the position they are visible from
- Go back to the initial position 

The assignment is implemented both in simulation  and with the real robot.

How to download
----------------------

In order to run the solution it is necessary to have the following ROS package:

```bash
git clone rosplablabla
```

```bash
git clone qualcosaltrodisicuro
```


Lastly you can finally download our package. 
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
this will open the simulation environment
```bash
roslaunch experimental_2 rosbot.launch
```
while this launches the complete behaviour of the rosbot, from the planning till the motion and the detection.

### PDDL
In order to make a plan it has been created in PDDL firstly a *domain* then a *problem* which will be later processed by a popf solver. The plan will be generated solved parsed and then dispatched.

#### Domain
...

#### Problem
...

#### Plan
...

### Architecture and Pseudocode

In order to achieve the solution it has been thought of the following architecture:

...

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
   

Main function
    Initialize ROS node
    Create MoveToInterface instance and run the action interface
```

#### DetectMarkerInterface Node
This node
```cpp
Francesca
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
This node
```    cpp
Francesca
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
There are different improvement regarding both the simulation and the real rosbot.
