#!/usr/bin/env python
import rospy
import sys
import tf
import pudb

from Particle_Filter 	import Particle_Filter
from threading 			import RLock
from numpy 				import prod, array, diag
from scipy.stats		import norm

from obj_tr_constants	import d_limits, array_size, g_func, g_res_fn,h_function, R, Q, COMP, UNCOMP

MAX_CHANGE_RATE 	= .3
class compromized_state(object):

	current_behv 	= array([0,0]) #Initialize all particles to stationary
	last_time 		= None

	def __init__(self, initial_prob, policy, zone, init_guess = None):
		self.c_prob 			= initial_prob
		self.init_prob 			= initial_prob
		self.policy				= policy 	#policy(point,c_last)
		self.zone				= zone		#zone(h_function(sensor_point), self.c_prob)
		self.comp_PF 			= Particle_Filter(d_limits, array_size, g_func, g_res_fn,h_function, R, Q, init_guess)
		self.uncomp_PF 			= Particle_Filter(d_limits, array_size, g_func, g_res_fn,h_function, R, Q, init_guess)
		self.last_time 			= rospy.Time.now()

	'''
	Compute the compromisation, then resample
	'''
	def compute_c_prob(self, sensor_point):
		sum_c_comp 		= self.sum_c(self.comp_PF.weights, self.comp_PF.point_list, sensor_point)
		sum_c_uncomp 	= self.sum_c(self.uncomp_PF.weights, self.uncomp_PF.point_list, sensor_point)

		comp_prob 		= self.zone(h_function(sensor_point), self.c_prob)
		uncomp_prob 	= self.zone(h_function(sensor_point), self.c_prob)

		# print sum_c_comp
		# print comp_prob
		# print sum_c_uncomp
		# print uncomp_prob
		c_prob_unorm	= sum_c_comp * comp_prob
		unc_prob_unorm	= sum_c_uncomp * uncomp_prob
		self.c_prob 	= c_prob_unorm / (c_prob_unorm + unc_prob_unorm)

		self.resample(sensor_point)

		return self.c_prob


	def sum_c(self, weight_list, point_list, sensor_point):
		pt_sum = 0
		for i in range(len(point_list)):
			diffs 	= g_res_fn(point_list[i],h_function(sensor_point))
			pt_sum 	= pt_sum + weight_list[i] * prod(norm(0,diag(Q)).pdf(diffs))
		return pt_sum


	'''
	Resample both particle filters
	'''
	def resample(self, sensor_point):
		self.comp_PF.resample(sensor_point)
		self.uncomp_PF.resample(sensor_point)


	'''
	update both particle filters
	'''
	def get_update(self):
		dt = self.get_duration()
		self.comp_PF.update(self.policy, COMP, dt)
		self.uncomp_PF.update(self.policy, UNCOMP, dt)
		return (self.comp_PF.point_list, self.uncomp_PF.point_list)


	def get_duration(self):
		tmp 			= rospy.Time.now()
		dur 			= tmp - self.last_time
		self.last_time 	= tmp
		return dur.to_sec()

	#Old code
	# '''
	# Bayes equation at work, adjusted to not be too volatile
	# '''
	# def update_prob(self, comp_prob, uncomp_prob):
	# 	self.c_prob =  comp_prob * self.c_prob / (comp_prob * self.c_prob + uncomp_prob * (1 - self.c_prob))
	# 	return self.get_c_prob()
	
	# # 	self.c_prob = self.adjust_prob(comp_prob, tmp)

	# # def adjust_prob(self, old_prob, new_prob):
	# # 	if math.isnan(new_prob):
	# # 		print "NaN cought"
	# # 		return old_prob

	# # 	if abs(new_prob - old_prob) > MAX_CHANGE_RATE:
	# # 		new_prob = new_prob - MAX_CHANGE_RATE if new_prob > old_prob else new_prob + MAX_CHANGE_RATE

	# # 	if new_prob < 0.0001:
	# # 		new_prob = .0001
	# # 	elif new_prob > 0.9999:
	# # 		new_prob = 0.9999

	# # 	return new_prob


	# def get_c_prob(self):
	# 	return self.c_prob

	# # '''
	# # updates the behavior to be used as control
	# # '''
	# # def upd_PF_behv(self,behv):
	# # 	with self.lock:
	# # 		self.current_behv = behv
	# # 		self.stationary = False


	# '''
	# Getter behv control
	# '''
	# def PF_behv(self):
	# 	return self.current_behv

	# '''
	# Updates the particle filter based on the sensor model input
	# '''
	# def upd_PF_sens(self, sensor_point):
	# 	self.comp_PF.upd_points(sensor_point)
	# 	self.uncomp_PF.upd_points(sensor_point)

	# '''
	# Updates the particle filter based on the motion model
	# '''
	# def get_PF_state(self):
	# 	with self.lock:
	# 		current_behv = self.current_behv
	# 	self.particle_filter.move_all(current_behv,self.get_duration())
	# 	return self.particle_filter.get_pt_list()

		
	# def get_duration(self):
	# 	tmp 			= rospy.Time.now()
	# 	dur 			= tmp - self.last_time
	# 	self.last_time 	= tmp
	# 	return dur.to_sec()
