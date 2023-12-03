#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "opencv2/core/mat.hpp"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <sstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <aruco_ros/aruco_ros_utils.h>
#include <unistd.h>
#include "rosplan_action_interface/RPActionInterface.h"

namespace KCL_Rosplan {
	
	class GoToInterface: public RPActionInterface
	{
		private:
		
		public:
			/* constructor */
			GoToInterface(ros::NodeHandle &nh);
			/* listen to and process action_dispatch topic */
			bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg);
	};
}
