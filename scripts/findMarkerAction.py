#!/usr/bin/env python
import rospy
import math
import time
import actionlib
from experimental_2.msg import markerDistance
from experimental_2.msg import findMarkerGoal, findMarkerFeedback, findMarkerResult
from geometry_msgs.msg import Twist, Pose2D
from std_msgs.msg import Bool, Float32, Int32

class FindMarker(object):
    _feedback = findMarkerFeedback()
    _result = findMarkerResult()

    # Constructor
    def __init__(self, name):
        self._action_name = name 

        # Attributes
        self.ack = False
        self.to_found = 0
        self.cmd = Twist()

        # Action Server
        self._as = actionlib.SimpleActionServer("/findMarkerAction",
                                                experimental_2.msg.findMarkerAction,
                                                execute_cb= self.execute_callback,
                                                auto_start= False)

        # Publisher for /cmd_vel
		self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribe to the topic for goal position from marker detection
		self.sub_marker_dist = rospy.Subscriber("/markerDistance", markerDistance, self.clbk_vision)

        # Publish for /requestMarkerId
		self.pub_marker_id = rospy.Publisher("/requestMarkerId", Int32, queue_size=10)

        # Start the server
        self._as.start()
    
    # Callback for markerPose subscription
	def clbk_vision(self, msg):
		if(msg.marker_id == self.to_found):
			# Acknowledge marker detection
			self.ack = msg.ack			
		else :
			#wrong/old marker found 
			self.ack = False

    def execute_callback(self, goal):

        rospy.loginfo("Executing find marker id : %d" % goal.markerId)
        starting_time = time.time()

        # Send the actual desired markerId
        self.to_found = goal.markerId
        self.pub_marker_id.publish(self.to_found)

        while(not(self.ack)):
            self.cmd.angular.z = 3
		    self.cmd_pub.publish(cmd)
            self._feedback.time_elapsed = time.time() - starting_time
            # publish the feedback
            self._as.publish_feedback(self._feedback)

        
        self.cmd.angular.z = 0
		self.cmd_pub.publish(cmd)

        self._result.found = True
        rospy.loginfo('Action Succeeded, markerId : %d founded' % goal.markerId)
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('findMarker')
    server = FindMarker(rospy.get_name())
    rospy.spin()
