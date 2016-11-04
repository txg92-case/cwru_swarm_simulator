#!/usr/bin/env python

import rospy
import roslib
import sys
import math
import pudb

from time 					import sleep
from geometry_msgs.msg 		import Twist
from swarm_sim.srv 			import TeleportAbsolute, TeleportRelative, Spawn, Kill
from swarm_sim.msg 			import Pose
from threading 				import Thread, RLock
from random					import randint, random

from obj_tr_constants		import TURTLE_COUNT, policy, d_limits, r, x ,y ,th


# Timing variables for use in the real-time loop as well as
# for initializing our node.
max_time_s 		= 10.0
frame_time_s 	= 0.02  # Just for kicks we will run at 100 Hz.

TURTLES 		= {}

turtle_lock 	= RLock()


class swarm_sim_control:
	# Just creating a single message so we don't have to waste time
	# constructing a new message over and over again.
	twist_msg = Twist()

	go_nuts = False

	time_s = 0.0

	run_lock 	= RLock()



	def __init__ (self):
		global max_time_s

		max_time_s = rospy.get_param("~max_time_s", 10.0)

		x_start,y_start,th_strt	= randint(d_limits[x][0]+2*r,d_limits[x][1]-2*r), randint(d_limits[y][0]+2*r,d_limits[y][1]-2*r), random() * 2 * math.pi - math.pi

		# Wait for the turtle's teleport service to start up, then move on to orient it.
		self.spawn = rospy.ServiceProxy('/spawn', Spawn)
		resp = self.spawn(x_start, y_start, th_strt, "")
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
		self.twist_msg = Twist()
		#self.orient_turtle_service(6.0, 8.0, init_theta_rad)

	def get_name(self):
		return self.name
	

	def change_state(self):
		with self.run_lock:
			self.go_nuts = not self.go_nuts


	def sns_callback(self, point):
		with self.run_lock:
			if self.go_nuts:
				cycle_pos = (2.0 * math.pi * self.time_s) / max_time_s

				v_x_t, v_y_t = get_linear_velocities_m_per_s(cycle_pos)
				self.twist_msg.linear.x = math.sqrt(v_x_t**2 + v_y_t**2)

				self.twist_msg.angular.z = get_angular_velocity_rad_per_s(cycle_pos)

				self.time_s += frame_time_s
				if self.time_s >= max_time_s:
					self.time_s = 0.0
			else:
				#pudb.set_trace()
				x_pol,z_pol 				= policy([point.x,point.y,point.theta],0)
				self.twist_msg.linear.x 	= x_pol
				self.twist_msg.angular.z 	= z_pol


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
	turtle = swarm_sim_control()
	with turtle_lock:
		TURTLES[turtle.get_name()] = turtle
	rospy.spin()
   

if __name__ == '__main__':
	rospy.init_node("swarm_sim_control", anonymous=True)
	kill = rospy.ServiceProxy('/swarm_sim/kill', Kill)
	to_kill = "turtle1"
	try:
		kill(to_kill)
	except Exception:
		print to_kill + " could not be killed"

	for i in range(TURTLE_COUNT):
		turtle_thread 			= Thread(target = main, args = (i, ))
		turtle_thread.setDaemon(True)
		turtle_thread.start()

	sleep(1)

	while not rospy.is_shutdown():
		input_var = raw_input('Choose a turtle to change the state of: ').strip()
		if input_var in TURTLES:
			with turtle_lock:
				TURTLES[input_var].change_state()
		else:
			print "Turtle not found!"

	rospy.spin()