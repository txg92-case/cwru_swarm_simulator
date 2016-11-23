#!/usr/bin/env python
import rospy
import sys
import pudb

import numpy 	as np

from math 		import pi, atan2, sqrt
from random		import gauss

x,y,th 					= 0,1,2
v_x, v_th 				= 0,1
data, pred_beh 			= 0,1

bc_interval 			= 1

d_limits				= np.array([[0,10],[0,10],[-np.pi,np.pi]]) 																			#x,y,theta,v_x,v_r limits
array_size				= 100																												#How many particles we want

R						= np.array([[1,.001],[.001,.1]])																					#Motion model noise
Q						= np.array([[2,.01,.02],[.01,2,.02],[.01,.01,.4]])																	#Measurement noise

PROB_SWITCH 			= .3
CIRC_CENTER 			= [6,6]
r 						= 2

TURTLE_COUNT 			= 1
COMP 					= 1
UNCOMP 					= 0 

FRAME_ID 				= "map"

'''
Changes state based on the behv given
'''
def g_func(tmp_state, behv_tmp, dt, R = None):
	state 		= tmp_state[:]

	if R is not None:
		behv			= np.diag(gauss(behv_tmp, R)).copy()
	else:
		behv 			= behv_tmp

	if behv[v_th] != 0:
		state[x] 		= state[x] - (behv[v_x] / behv[v_th]) * np.sin(state[th]) + (behv[v_x] / behv[v_th]) * np.sin(state[th] + behv[v_th] * dt)
		state[y] 		= state[y] + (behv[v_x] / behv[v_th]) * np.cos(state[th]) - (behv[v_x] / behv[v_th]) * np.cos(state[th] + behv[v_th] * dt)
		state[th] 		= state[th] + behv[v_th] * dt

	else:
		state[x] 		= state[x] + np.cos(state[th]) * dt * behv[v_x]
		state[y] 		= state[y] + np.sin(state[th]) * dt * behv[v_x]
		state[th] 		= state[th]



	state[th] 			= rot_diff(state[th])

	return state

# def g_func_comp(tmp_state, behv, dt, R = None):
# 	state 		= tmp_state[:]
# 	return np.diag(gauss(state, R)).copy()

'''
Residual for motion model
'''
def g_res_fn(z, mean):

	residual 			= z[0:3] - mean

	residual[2] 		= rot_diff(residual[2])

	return residual

def rot_diff(rot_in):
	while rot_in > pi:
		rot_in -= 2 * pi
	while rot_in < -pi:
		rot_in += 2 * pi
	return rot_in

'''
Obtains the observation data from our input data
'''
def h_function(mean):
	return mean[0:3]


'''
Used for the obtaining the command vel from position
'''
def policy(point,c_last):
	global v_x, v_th, x,y,th

	behv 	= [0,0]

	# p_x 		= point[x] - CIRC_CENTER[x]
	# p_y 		= point[y] - CIRC_CENTER[y]
	# p_th 		= point[th]

	# r_x 			= (p_x/abs(p_x))*r if p_x != 0 else 0
	# r_y				= (p_y/abs(p_y))*r if p_y != 0 else 0

	# d_conv 			= 1

	if c_last is UNCOMP: #TODO: does this make sense? esentially rounding
		behv[v_x] 	= float(3)/2
		# D 		  	= sqrt(p_x**2 + p_y ** 2)
		# if D < r: 							#inside the circle
		# 	d 	= r - D
		# else:								#outside the circle
		# 	d 	= D - r

		# if d > d_conv:
		# 	pudb.set_trace()
		# 	weight 	= 1/(d+1) *.1
		# 	if D < r: 						#inside the circle
		# 		phi 	= atan2(p_x,p_y) 	#pointing away form center
		# 		behv[v_th] 	= float(1)/2 - circ_diff(p_th,phi)*weight
		# 	else: 
		# 		phi 	= atan2(p_y,p_x) 	#pointing towards center
		# 		behv[v_th] 	= float(1)/2 + circ_diff(p_th,phi)*weight
			
		# else:
		# 	phi 	= atan2(p_x,-p_y) 		#tangental to the circle	
		# 	weight 	= d/d_conv *.5
		# 	if D < r: 						#inside the circle
		# 		behv[v_th] 	= float(1)/2 - circ_diff(p_th,phi)*weight
		# 	else: 
		# 		behv[v_th] 	= float(1)/2 + circ_diff(p_th,phi)*weight
		behv[v_th] 	= float(1)
	else:
		behv[v_x] 	= gauss(0,3)
		behv[v_th] 	= gauss(0,pi)

	return behv


def circ_diff(u,v):
	mn_val,mx_val = (u,v) if u > v else (v,u)
	diff1		= mn_val - mx_val
	diff2		= mx_val - mn_val + 2*pi
	lesser_diff = diff1 if diff1 < diff2 else diff2
	return rot_diff(lesser_diff)

def rot_diff(rot_in):
	while rot_in > pi:
		rot_in -= 2 * pi
	while rot_in < -pi:
		rot_in += 2 * pi
	return rot_in

def zone(z, c):
	return c

# '''
# Used for the obtaining the compromised probability from position and last compromised state
# '''
# def zone(z, c):
# 	prob 	= 0
# 	zone1 	= [2,4]
# 	zone2 	= [1,5]#-[2,4]
# 	#zone3  = [0,inf]-[1,5] 
# 	#Good zone
# 	if (within_circ_bnds(CIRC_CENTER[x],CIRC_CENTER[y],zone1[0],zone1[1],z[0],z[1])):
# 		prob = .1
# 	#Preventive zone
# 	elif (within_circ_bnds(CIRC_CENTER[x],CIRC_CENTER[y],zone2[0],zone2[1],z[0],z[1])):
# 		prob = .5
# 	#Restricted zone
# 	else:
# 		prob = .9
# 	return prob * max(min(1, gauss(c,PROB_SWITCH)), 0)


def within_circ_bnds(x_cent,y_cent,r1,r2,z_x,z_y):
	dst_cubed = (x_cent + z_x)**2 + (y_cent + z_y)**2
	return dst_cubed >= r1**2 and dst_cubed <= r2**2