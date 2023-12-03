#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "opencv2/core/mat.hpp"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <sstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <aruco_ros/aruco_ros_utils.h>
#include <unistd.h>
#include "experimental_2/markerVision.h"

class MarkerDetector {

	private:
		/*------------Aruco------------*/		
		//aruco marker detector used to recognize the markers
		aruco::MarkerDetector mDetector_;
		//vector where all recognized markers are save
		std::vector<aruco::Marker> markers_;
		//aruco camera parameter neeeded to identify marker position
		aruco::CameraParameters camParam_;		
		bool useRectifiedImages = true;
		
		//marker size
		double marker_real_size_ = 0.2;
		double marker_size_ = marker_real_size_;
		/*-----------------------------*/

		//CV image decompressed from it_
		cv::Mat inImage_;

		//
		sensor_msgs::ImagePtr outputMsg;

		bool POV_window_b_;

		//ROS pubs/subs
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;
		image_transport::Publisher debug_pub_;

		//Subscriber to get the camera info 	
		ros::Subscriber cam_info_sub;	

		//To 
		ros::Subscriber killer_sub_;

		ros::ServiceServer markerVisionSrv;

	public:

		MarkerDetector() : nh_("~"), it_(nh_) {

			//create the subscribe to camera image /camera/rgb/image_raw/compressed
			image_sub_ = it_.subscribe("/camera/color/image_raw",1, &MarkerDetector::image_callback, this);

			//create subscriber to camera info in order to set the proper Aruco Camera Parameter
			cam_info_sub = nh_.subscribe("/camera/color/camera_info", 1, &MarkerDetector::cam_info_callback, this);
			
			//Pub debug
			debug_pub_ = it_.advertise("/debug/image_raw", 1);	

			//Service
			markerVisionSrv = nh_.advertiseService("/markerVision", &MarkerDetector::vision_callback, this);

			killer_sub_ = nh_.subscribe("/task_complete",1, &MarkerDetector::killer_callback, this);

			nh_.param<bool>("pov_window", POV_window_b_, true);

			camParam_ = aruco::CameraParameters();

			//Debug window for local running
			if(POV_window_b_){
				//Create window for RosBot POV
				cv::namedWindow("ROSBot POV", cv::WINDOW_AUTOSIZE);	
			}

		}

		/*
			Callback function of /camera/color/image_raw
			each time an image is published update the list of marker detected
			If the marker to found is detected compute it's distance from the center of the camera, to notify
			if there's a misalignemet, and how many pixel is the side of the marker to see if the rosbot is still distant 
	 	*/
	 	void image_callback(const sensor_msgs::ImageConstPtr& msg) {
			ros::Time curr_stamp = msg->header.stamp;
			cv_bridge::CvImagePtr cv_image_ptr; 

			try {
				//get CV Image from sensor image
				cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
				inImage_ = cv_image_ptr->image;
				
				//get center of the camera Frame
				int width = static_cast<float>(inImage_.cols);
				int height = static_cast<float>(inImage_.rows);

				//Define the center of the camera image
				cv::Point2f camera_center(width/2.0f, height/2.0f);
   
				// clear out previous detection results
				markers_.clear();

				// detect marker
				mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);
				
				for (std::size_t i = 0; i < markers_.size(); ++i) {
					// draw detected markers on the image for visualization
					markers_[i].draw(inImage_, cv::Scalar(0, 0, 255), 2);
					//if the marker to found is detected
				}

				//Visualize the camera POV
				if(POV_window_b_){
					cv::imshow("ROSBot POV", inImage_);
					cv::waitKey(1);
				}				

				/*Debug publisher*/
				outputMsg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, inImage_).toImageMsg();
				debug_pub_.publish(outputMsg);

			}catch (cv_bridge::Exception& e) {
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}
		}

		/*Service callback*/
		bool vision_callback(experimental_2::markerVision::Request& req, experimental_2::markerVision::Response& res){
			for(std::size_t i = 0; i < markers_.size(); ++i) {
				if(markers_[i].id == req.markerID) {
					res.ack = 1;
					return 1;
				}
			}
			return 0;
		}

		/*
			wait for one camerainfo to get Aruco CameraParam, then shut down that subscriber
		*/ 
		void cam_info_callback(const sensor_msgs::CameraInfo &msg) {
			/*Get one camera info to properly set the ArucoCameraParameter*/ 
			camParam_ = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);
			//kill the subscriber
			cam_info_sub.shutdown();
		}

		/*
		        If recived a message of finish work this callback 
		        ends the node
		*/
		void killer_callback(const std_msgs::Bool &msg){
		        if(msg.data) ros::shutdown();
		}
};

int main(int argc, char **argv){
	// init ros Node
	ros::init(argc, argv, "marker_detector");	
	
	//create markerDetecor
	MarkerDetector node;
	
	//spin Node
	ros::spin();
}
