#!/usr/bin/env python

import rospy
import roslib
import sys
import math
import numpy

import pudb

from time 					import sleep
from geometry_msgs.msg 		import Twist
from swarm_sim.srv 			import TeleportAbsolute, TeleportRelative, Spawn, Kill
from swarm_sim.msg 			import Pose
from threading 				import Thread, RLock
from random					import randint, random
#from obj_tr_constants		import policy,x_start,y_start,th_start


# Timing variables for use in the real-time loop as well as
# for initializing our node.
max_time_s 		= 10.0
frame_time_s 	= 0.02  # Just for kicks we will run at 100 Hz.

TURTLE_COUNT 	= 20
TURTLES 		= {}

turtle_lock 	= RLock()

d_limits				= numpy.array([[0,10],[0,10],[-numpy.pi,numpy.pi]])

r = 5

th_start = 0.0;

class turtlesim_control:
	# Just creating a single message so we don't have to waste time
	# constructing a new message over and over again.
	twist_msg = Twist()

	go_nuts = False

	time_s = 0.0

	run_lock = RLock()

	x_loc = 0.0
	y_loc = 0.0

	wolf_status = False;

	def __init__ (self):
		global max_time_s

		max_time_s = rospy.get_param("~max_time_s", 10.0)

		# Wait for the turtle's teleport service to start up, then move on to orient it.
		self.spawn = rospy.ServiceProxy('/spawn', Spawn)

		x_start,y_start,th_strt	= randint(4,6), randint(4,6), random() * 2 * math.pi - math.pi

		resp = self.spawn(x_start, y_start, th_start, "")
		self.name = resp.name
		print self.name
		rospy.wait_for_service(self.name + "/teleport_absolute")

		self.orient_turtle_service 	= rospy.ServiceProxy(self.name + "/teleport_absolute", TeleportAbsolute)
		self.control_pub 			= rospy.Publisher(self.name + "/cmd_vel", Twist, queue_size=10)


		rospy.Subscriber(self.name + "/pose", Pose, self.sns_callback)

		# The initial orientation should be based on the theta determined from the
		# initial velocities at t=0.
		v_x_t, v_y_t = get_linear_velocities_m_per_s(0.0)
		init_theta_rad = math.atan2(v_y_t, v_x_t)

		x_loc = x_start
		y_loc = y_start

		#self.orient_turtle_service(6.0, 8.0, init_theta_rad)

	def get_name(self):
		return self.name
	

	def change_state(self):
		with self.run_lock:
			self.go_nuts = not self.go_nuts


	def sns_callback(self, point):
		twist_msg = Twist()
		with self.run_lock:

			self.x_loc = point.x
			self.y_loc = point.y

			wolf_status = point.wolf


			self.twist_msg.linear.x 	= 1.5
			self.twist_msg.angular.z 	= 1.0

			# If the self is a wolf, search for the nearest sheep.
			if wolf_status:

				# print "Attempting to eat..."
				# pudb.set_trace()
				# compute the nearest potential victim...
				arc_min = 200.0*numpy.pi

				delList = []

				for  turtle_, turtle in TURTLES.iteritems():
					if turtle_ == self.get_name():
						continue

					x_dist, y_dist = 0, 0 

					with turtle.run_lock:
						x_dist = turtle.x_loc - self.x_loc
						y_dist = turtle.y_loc - self.y_loc

					norm_dist = numpy.sqrt(x_dist**2 + y_dist**2)

					
					if norm_dist < 0.5:
						# eat the sheep.
						kill = rospy.ServiceProxy('/kill', Kill)
						try:
							kill(turtle_)
							print "%s thought that %s was yummy!" % (self.get_name(), turtle_)
							delList.append(turtle_)
						except Exception:
							print "Unable to eat turtle %s!!!" % turtle_
					else:
						#Plan the approach.
						x_tan = numpy.cos(point.theta)
						y_tan = numpy.sin(point.theta)

						ip = (x_dist*x_tan + y_dist*y_tan)/norm_dist

						eta = numpy.arccos(ip)

						phi = numpy.pi/2 - eta

						#omega direction.
						omega = x_tan*y_dist - y_tan*x_dist

						omega = omega / abs(omega)

						# r inverse
						r_inv = 2 * numpy.cos(phi) / norm_dist

						theta = numpy.pi - 2*phi

						if r_inv > 0:
							arc_length = theta / r_inv
						else:
							arc_length = norm_dist

						if arc_length < arc_min:
							#find v (double that of sheep.)
							self.twist_msg.linear.x = 2.0
							self.twist_msg.angular.z = 2.0*r_inv*omega
							arc_min = arc_length

				#delete
				for k in delList: del TURTLES[k]
					

		self.control_pub.publish(self.twist_msg)


# Computes the x and y components of the linear velocity by taking the
# time derivatives of xd(t) and yd(t).
def get_linear_velocities_m_per_s(cycle_pos):
	v_x_t = ((12.0 * math.pi) / max_time_s) * math.cos(2.0 * cycle_pos)
	v_y_t = ((6.0 * math.pi) / max_time_s) * math.cos(cycle_pos)

	return v_x_t, v_y_t


# Calculates angular velocity by calculating the first time derivative of
# arctan(v_y_t/v_x_t).
def get_angular_velocity_rad_per_s(cycle_pos):
	ang_vel_rad_per_s = 3.0 * math.sin(cycle_pos) + math.sin(3.0 * cycle_pos)
	ang_vel_rad_per_s *= 4.0 * math.pi

	ang_vel_rad_per_s /= max_time_s * (math.cos(2.0 * cycle_pos) + 4.0 * math.cos(4.0 * cycle_pos) + 5.0)

	return ang_vel_rad_per_s


def main(index):
	global TURTLES
	#sleep(index*4*math.pi/TURTLE_COUNT)
	turtle = turtlesim_control()
	with turtle_lock:
		TURTLES[turtle.get_name()] = turtle
	rospy.spin()
   

if __name__ == '__main__':
	rospy.init_node("turtlesim_control", anonymous=True)
	kill = rospy.ServiceProxy('/kill', Kill)
	try:
		kill("turtle1")
	except Exception:
		print "Turtle 1 not active"

	for i in xrange(TURTLE_COUNT):
		turtle_thread 			= Thread(target = main, args = (i, ))
		turtle_thread.setDaemon(True)
		turtle_thread.start()

	# sleep(4*math.pi)
	rospy.spin()

	#add thread joining...

