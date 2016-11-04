#!/usr/bin/env python
import rospy
import sys
import tf
import pudb

import numpy 	as np

from visualization_msgs.msg import Marker, MarkerArray
#from scipy 					import spatial, stats
from geometry_msgs.msg 		import Twist
from nav_msgs.msg			import Odometry
from time 					import sleep
from math 					import pi
from collections			import deque
from bayes 					import compromized_state
from random 				import gauss
from threading 				import Thread, RLock
from obj_tr_constants		import x,y,th,v_x, v_th, data, pred_beh, bc_interval, array_size
from obj_tr_constants		import policy, zone, FRAME_ID

rand_len		= 200

#COMPARE_SIZE 	= 3
INITIAL_PROB 	= 0.05
UPD_FREQUENCY	= .1
CPROB_FREQUENCY	= 1

class robot_state(object):

	def __init__(self, ID, policy, zone):
		self.ID 			= ID
		init_pos 			= None #np.array([0.0,0.0,0.0])
		self.bayes 			= compromized_state(INITIAL_PROB, policy, zone, init_pos)
		rospy.Subscriber(ID + "/noisedOdom", Odometry, self.sns_callback)
		self.c_particle_pub 	= rospy.Publisher("/" + str(self.ID) + "/c_pointList", MarkerArray, queue_size=array_size)
		self.unc_particle_pub 	= rospy.Publisher("/" + str(self.ID) + "/unc_pointList", MarkerArray, queue_size=array_size)
		self.lock 			= RLock()
		self.callbackOdom 	= None

	'''
	Updates the particle filter
	'''
	def bayes_upd(self):
		global UPD_FREQUENCY
		while True:
			with self.lock:
				c_pointList, unc_pointList = self.bayes.get_update()
			publish_particles(self.c_particle_pub, c_pointList, [1,.1,.1])		#RED
			publish_particles(self.unc_particle_pub, unc_pointList, [.1,1,.1])	#BLUE
			rospy.sleep(UPD_FREQUENCY)

	'''
	Resamples and computes probability of compromisation
	'''
	def bayes_c_prob(self):
		global UPD_FREQUENCY, CPROB_FREQUENCY
		while True:
			if self.callbackOdom is not None:
				with self.lock:
					sensor_pt	= state_to_list(self.callbackOdom)
					prob  		= self.bayes.compute_c_prob(sensor_pt)
				print "Likelyhood of being compromised is: " + str(prob)
				self.callbackOdom = None
				rospy.sleep(CPROB_FREQUENCY)
			else:
				rospy.sleep(UPD_FREQUENCY)


	'''
	Sensor info coming in
	'''
	def sns_callback(self, odom):
		self.callbackOdom = odom


'''
Visualizaiton methods
'''

def publish_particles(pub, pointList, rbg):

	markerArray = MarkerArray()

	for i in range(pointList.shape[0]):

		point 						= pointList[i]
		marker						= Marker()

		tempQuaternion				= tf.transformations.quaternion_from_euler(0, 0, point[2])

		marker.pose.position.x = point[x]
		marker.pose.position.y = point[y]
		marker.pose.position.z = 0

		marker.scale.x = 0.4
		marker.scale.y = 0.05
		marker.scale.z = 0.01

		marker.color.a = 1.0
		marker.color.r = rbg[0]
		marker.color.b = rbg[1]
		marker.color.g = rbg[2]

		marker.pose.orientation.x 	= tempQuaternion[0]
		marker.pose.orientation.y 	= tempQuaternion[1]
		marker.pose.orientation.z 	= tempQuaternion[2]
		marker.pose.orientation.w 	= tempQuaternion[3]

		marker.id 				= i
		marker.header.frame_id 	= FRAME_ID
		marker.header.stamp 	= rospy.Time.now()
		marker.action 			= marker.ADD

		markerArray.markers.append(marker)

	pub.publish(markerArray)


# def publish_positions(mean_prtcl, rcv_prtcl):
# 	mean_marker 	= get_marker(mean_prtcl, 1, "/map", [.3,.3,1])
# 	rcv_pos_marker 	= get_marker(rcv_prtcl, 2, "/map", [1,.3,.3])

# 	mean_pos.publish(mean_marker)
# 	rcv_pos.publish(rcv_pos_marker)

# def get_marker(particle, ID, frame_id, color):
# 	marker						= Marker()

# 	marker.pose.position.x = particle[x]
# 	marker.pose.position.y = particle[y]
# 	marker.pose.position.z = 0

# 	tempQuaternion				= tf.transformations.quaternion_from_euler(0, 0, particle[th])

# 	marker.scale.x = 0.4
# 	marker.scale.y = 0.05
# 	marker.scale.z = 0.01

# 	marker.color.a = 1
# 	marker.color.r = color[0]
# 	marker.color.g = color[1]
# 	marker.color.b = color[2]

# 	marker.pose.orientation.x 	= tempQuaternion[0]
# 	marker.pose.orientation.y 	= tempQuaternion[1]
# 	marker.pose.orientation.z 	= tempQuaternion[2]
# 	marker.pose.orientation.w 	= tempQuaternion[3]

# 	marker.id 				= ID
# 	marker.header.frame_id 	= frame_id
# 	marker.header.stamp 	= rospy.Time.now()
# 	marker.action 			= marker.ADD

# 	return marker

'''
Used to obtain robot ID from observations. This is post processed
'''
def observe_robots(ID_list):
	global u_list, p_list, policy, zone

	for ID in ID_list:
		c_list[ID]  	= robot_state(ID, policy, zone) #Can make diff robots have diff policies and zones here
		u_list[ID] 		= Thread(target = bayes_upd, args = (c_list[ID], ))
		u_list[ID].setDaemon(True)
		u_list[ID].start()
		p_list[ID] 		= Thread(target = bayes_c_prob, args = (c_list[ID], ))
		p_list[ID].setDaemon(True)
		p_list[ID].start()


def bayes_upd(class_inst):
	class_inst.bayes_upd()

def bayes_c_prob(class_inst):
	class_inst.bayes_c_prob()


'''
Extra methods
'''
def get_yaw(orientation):
	quaternion = (
		orientation.x,
		orientation.y,
		orientation.z,
		orientation.w)
	return tf.transformations.euler_from_quaternion(quaternion)


def state_to_list(odom_msg):
	return [odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, get_yaw(odom_msg.pose.pose.orientation)[2]]


def beh_to_list(twist_msg):
	bh_list = [twist_msg.linear.x , twist_msg.angular.z]
	return bh_list


if __name__ == '__main__':
	global c_list, u_list, p_list

	c_list 			= {}
	u_list, p_list 	= {}, {} #When multiple robots go in and out of range these lists can be used to kill the threads

	rospy.init_node('camp_comp', anonymous=True)
	#rospy.Subscriber("/secure_estimation/", custum_message_type, observe_robots)

	ID_list = ["F9DRR"]

	observe_robots(ID_list)

	# '''
	# Used as a place holder for the subscriber of a module that should discern new robots coming in and track them
	# '''
	# listener_thread 			= Thread(target = observe_robots, args = (ID_list, ))
	# listener_thread.setDaemon(True)
	# listener_thread.start()
	# '''
	# end placeholder
	# '''

	print "Map Comparator Ready!"

	rospy.spin()


	#publish_state_beh(state_beh_list, bc_interval)