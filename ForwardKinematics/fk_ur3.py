#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab3_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def get_skew_symmetric(w,v):
	skew = np.zeros((4,4))
	skew[0] = [0,-w[2],w[1],v[0]]
	skew[1] = [w[2],0,-w[0],v[1]]
	skew[2] = [-w[1],w[0],0,v[2]]
	skew[3] = [0,0,0,0]
	return skew

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.zeros((4,4))
	M[0] = [0, -1, 0, .390]
	M[1] = [0, 0, -1, .401]
	M[2] = [1, 0, 0, .2155]
	M[3] = [0, 0, 0, 1]
	
	w1 = np.array([0,0,1])
	w2 = np.array([0,1,0])
	w3 = np.array([0,1,0])
	w4 = np.array([0,1,0])
	w5 = np.array([1,0,0])
	w6 = np.array([0,1,0])

	q1 = np.array([-.150,.150,.010])
	q2 = np.array([-.150,.270,.162])
	q3 = np.array([.094,.270,.162])
	q4 = np.array([.307, .177, .162])
	q5 = np.array([.307,.260,.162])
	q6 = np.array([.390,.260,.162])

	v1 = -np.cross(w1, q1)
	v2 = -np.cross(w2, q2)
	v3 = -np.cross(w3, q3)
	v4 = -np.cross(w4, q4)
	v5 = -np.cross(w5, q5)
	v6 = -np.cross(w6, q6)

	s1 = get_skew_symmetric(w1,v1)
	s2 = get_skew_symmetric(w2,v2)
	s3 = get_skew_symmetric(w3,v3)
	s4 = get_skew_symmetric(w4,v4)
	s5 = get_skew_symmetric(w5,v5)
	s6 = get_skew_symmetric(w6,v6)

	S = [s1, s2, s3, s4, s5, s6]

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()

	T = np.dot(T,expm(S[0]*theta1))
	T = np.dot(T,expm(S[1]*theta2))
	T = np.dot(T,expm(S[2]*theta3))
	T = np.dot(T,expm(S[3]*theta4))
	T = np.dot(T,expm(S[4]*theta5))
	T = np.dot(T,expm(S[5]*theta6))
	T = np.dot(T,M)

	# ==============================================================#
	
	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value

