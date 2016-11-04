#!/usr/bin/env python
import rospy
import sys
import tf
import pudb

import numpy as np
import scipy.cluster.hierarchy as hchy

from numpy					import zeros,prod,pi,diag,ones
from random					import random
from math					import hypot, pi, sin, cos, asin, acos, atan2
from scipy.stats			import norm

from obj_tr_constants		import x,y,th,v_x, v_th, data, pred_beh



class Particle_Filter(object):

	def __init__(self, d_limits, array_size, g_function, g_res_fn,h_function, R, Q, init_guess = None):

		self.g 				= g_function
		self.g_res_fn 		= g_res_fn
		self.h 				= h_function
		self.R 				= R
		self.Q 				= Q

		self.array_size 	= array_size
		self.d_limits		= d_limits

		self.weights		= ones([array_size,1])/array_size

		self.result_points	= zeros([array_size,d_limits.shape[0]])
		self.result_weights = zeros([array_size,1])

		self.rand_all(init_guess)


	def rand_all(self, init_guess):
		for i in range(self.d_limits.shape[0]):
			if i == 0:
				self.point_list = np.random.uniform(self.d_limits[i][0],self.d_limits[i][1],self.array_size).reshape((self.array_size, 1))
			else:
				self.point_list = np.hstack((self.point_list,np.random.uniform(self.d_limits[i][0],self.d_limits[i][1],self.array_size).reshape((self.array_size, 1))))
		
		if init_guess is not None and len(self.point_list) > 0:
			self.bias(init_guess)


	def bias(self, pos):
		self.point_list[-1] = pos


	'''
	Resample
	'''
	def resample(self, sensor_point):
		#pudb.set_trace()
		tot_weight = self.set_weights(sensor_point)
		self.normalize_weights(tot_weight)
		self.upd_points()
		tot_weight = self.set_weights(sensor_point)
		self.normalize_weights(tot_weight)
		#self.bias(sensor_point)
		#return self.get_mean_and_var(self.point_list)


	def set_weights(self, sensor_point):
		tot_weight = 0
		for i in range(len(self.point_list)):
			self.set_weight(self.point_list[i],sensor_point,i)
			tot_weight = tot_weight + self.weights[i]
		return tot_weight


	def set_weight(self,filter_point, sensor_point, index):
		diffs 				= self.g_res_fn(filter_point,self.h(sensor_point))
		# multiply the probabilities of each dimention
		self.weights[index] = prod(norm(0,diag(self.Q)).pdf(diffs))


	def normalize_weights(self, tot_weight):
		for i in range(len(self.point_list)):
			self.weights[i] = self.weights[i] / tot_weight


	def upd_points(self):
		sample_interval = float(1)/self.array_size
		offset 			= random() * sample_interval

		last_weight		= self.weights[0]
		index_weight	= 0

		current_weight	= 0

		#resample while while copying nodes with higher weights
		for i in range(self.array_size):
			current_weight 			= offset + i * sample_interval

			while current_weight > last_weight:
				index_weight 	+= 1
				last_weight 	+= self.weights[index_weight]

			self.result_points[i] 	= self.point_list[index_weight]

			i += 1

		self.point_list 	= self.result_points[:]

	'''
	Update
	'''
	def update(self,policy,c_last,dt):
		for i in range(self.array_size):
			self.point_list[i] = self.g(self.point_list[i], policy(self.point_list[i],c_last), dt, self.R)


	# '''
	# Extra func
	# '''
	# def get_mean_and_var(self,pointList):
	# 	mean 		= np.mean(pointList, axis=0)
	# 	mean[th] 	= self.get_polar_mean(pointList[:,th-1:th]) #TODO: uncomment
	# 	return mean

	# def get_polar_mean(self, org_list):
	# 	tot_sin 	= 0
	# 	tot_cos  	= 0
	# 	for ang in org_list:
	# 		tot_cos		=+ cos(ang)
	# 		tot_sin		=+ sin(ang)

	# 	return atan2(tot_sin,tot_cos)

	# def get_pt_list(self): 
	# 	return self.point_list


	# def circ_diff(self,u,v):
	# 	mn_val,mx_val = (u,v) if u > v else (v,u)
	# 	diff1		= mn_val - mx_val
	# 	diff2		= mx_val - mn_val + 2*pi
	# 	lesser_diff = diff1 if diff1 < diff2 else diff2
	# 	return self.rot_diff(lesser_diff)


	# def get_polar_mean_cluster(self,org_list):
	# 	method = "average"
	# 	Z = hchy.linkage(org_list, method=method, metric=self.circ_diff)
	# 	max_d = .25
	# 	lst = hchy.fcluster(Z, max_d, criterion='distance')
	# 	elm = np.bincount(lst).argmax()
	# 	ind = [i for i, x in enumerate(lst) if x==elm]
	# 	cst = org_list[[ind]]
	# 	u	= self.get_p_mean_clst(cst)
	# 	return u

	# def rot_diff(self,rot_in):
	# 	while rot_in > pi:
	# 		rot_in -= 2 * pi
	# 	while rot_in < -pi:
	# 		rot_in += 2 * pi
	# 	return rot_in

	# def get_p_mean_clst(self,cl_list):
	# 	cl_cr_list = [(j if j >=0 else j + 2*pi) for j in cl_list]
	# 	mean = np.mean(cl_cr_list)
	# 	return self.rot_diff(mean)