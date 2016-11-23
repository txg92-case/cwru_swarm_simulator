#!/usr/bin/env python

import rospy
import roslib
import sys
import pudb
import tf

from swarm_sim.msg 			import Pose
from nav_msgs.msg			import Odometry
from threading 				import Thread
from obj_tr_constants 		import FRAME_ID

'''
The purpose of this class is to simulate a state tracker that is able to identify
the different agents in the state space, assign IDs to each and emit their position.
This position is simulated in our case using the ground truth by adding error.
'''

class sec_state_trk:

	def __init__ (self, turtle_List, ID_List):
		for i in range(len(turtle_List)):
			rospy.Subscriber(turtle_List[i] + "/pose", Pose, self.sns_callback)
			rospy.Subscriber(turtle_List[i] + "/pose_noisy", Pose, self.sns_callback_noisy)
			self.truth_pub 		= rospy.Publisher(ID_List[i] + "/truthOdom", Odometry, queue_size=1)
			self.sec_state_pub 	= rospy.Publisher(ID_List[i] + "/noisedOdom", Odometry, queue_size=1)
			self.odom_msg 		= Odometry()
			self.odom_msg_nd 	= Odometry()

	def sns_callback(self, point):
		#TODO: add noise
		x_p, y_p, th_p 							= point.x,point.y,point.theta

		self.odom_msg.pose.pose.position.x 		= x_p
		self.odom_msg.pose.pose.position.y  	= y_p

		quat = get_quat(0,0,th_p)
		self.odom_msg.pose.pose.orientation.x 	= quat[0]
		self.odom_msg.pose.pose.orientation.y 	= quat[1]
		self.odom_msg.pose.pose.orientation.z 	= quat[2]
		self.odom_msg.pose.pose.orientation.w 	= quat[3]
		self.odom_msg.header.frame_id 			= FRAME_ID

		self.truth_pub.publish(self.odom_msg)

	def sns_callback_noisy(self, point):
		x_p, y_p, th_p 								= point.x,point.y,point.theta

		self.odom_msg_nd.pose.pose.position.x 		= x_p
		self.odom_msg_nd.pose.pose.position.y  		= y_p

		quat = get_quat(0,0,th_p)
		self.odom_msg_nd.pose.pose.orientation.x 	= quat[0]
		self.odom_msg_nd.pose.pose.orientation.y 	= quat[1]
		self.odom_msg_nd.pose.pose.orientation.z 	= quat[2]
		self.odom_msg_nd.pose.pose.orientation.w 	= quat[3]
		self.odom_msg_nd.header.frame_id 			= FRAME_ID

		self.sec_state_pub.publish(self.odom_msg_nd)

def main(turtle_List,ID_List):
	sec_state_trk(turtle_List,ID_List)
	rospy.spin()


'''
Other methods
'''
def get_quat(roll, pitch, yaw):
	return tf.transformations.quaternion_from_euler(roll, pitch, yaw)


if __name__ == '__main__':
	rospy.init_node("secure_state_sim", anonymous=True)

	turtle_List = ["/turtle2"]#,"/turtle3"]
	ID_List		= ["/F9DRR"]#,"/A4DFB"]

	#for i in range(TURTLE_COUNT): #For expansion (not only one observer)
	turtle_thread 			= Thread(target = main, args = (turtle_List,ID_List, ))
	turtle_thread.setDaemon(True)
	turtle_thread.start()

	rospy.spin()