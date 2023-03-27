#!/usr/bin/env python3
import rospy
import numpy as np
from robot_vision_lectures.msg import XYZarray, SphereParams
from geometry_msgs.msg import Point

points = []

def get_ball(ballPoints):
	global points
	points = ballPoints.points

def main():
	global points
	
	params = SphereParams()
	
	rospy.init_node('sphere_fit', anonymous = True)
	ballSub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, get_ball)
	spherePub = rospy.Publisher("/sphere_params", SphereParams, queue_size=10)
	
	# set the loop frequency
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		x = []
		y = []
		z = []
		eqns = []
		for point in points:
			x.append(2*point.x)
			y.append(2*point.y)
			z.append(2*point.z)
			eqns.append(point.x**2 + point.y**2 + point.z**2)
		A = np.vstack([x, y, z, np.ones(len(x))]).T
		B = np.array([eqns]).T
		#ATA = np.matmul(A.T, A)
		#ATB = np.matmul(A.T, B)
		
		#test = np.linalg.inv(ATA)
		#print(test)
		
		#P = np.matmul(ATA, ATB)
		P, c1, rank, s = np.linalg.lstsq(A,B,rcond=None)
		
		print(c1)
		params.xc = P[0]
		params.yc = P[1]
		params.zc = P[2]
		params.radius = (P[0]**2 + P[1]**2 + P[2]**2 + P[3])**(0.5)
		
		print(params)
		spherePub.publish(params)		
			
		# pause until the next iteration			
		rate.sleep()
main()
