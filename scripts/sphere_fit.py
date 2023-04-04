#!/usr/bin/env python3
# useful imports
import rospy
import numpy as np
import math
from robot_vision_lectures.msg import XYZarray, SphereParams
from geometry_msgs.msg import Point

# List of points to estimate ball
points = []

# Callback function for /xyz_cropped_ball subscriber
def get_ball(ballPoints):
	global points
	points = ballPoints.points

def main():
	global points
	# Empty SphereParams msg for storing sphere params to publish
	params = SphereParams()
	
	# Initialize node, subscriber, and publisher
	rospy.init_node('sphere_fit', anonymous = True)
	ballSub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, get_ball)
	spherePub = rospy.Publisher("/sphere_params", SphereParams, queue_size=10)
	
	# set the loop frequency
	rate = rospy.Rate(10)
	
	#Define initial conditions 
	xc_out = -0.014
	yc_out = -0.017
	zc_out = 0.52
	p_out = -0.2
	gain = 0.05
		
	# main rospy loop
	while not rospy.is_shutdown():
		# empty x, y, z, and equation lists for esitmation calculations
		x = []
		y = []
		z = []
		eqns = []
		# Looping through points and adding them to above lists for A matrix
		for point in points:
			x.append(2*point.x)
			y.append(2*point.y)
			z.append(2*point.z)
			eqns.append(point.x**2 + point.y**2 + point.z**2)
		# creating A and B matricies for estimation
		A = np.vstack([x, y, z, np.ones(len(x))]).T
		B = np.array([eqns]).T
		# using pseudo inverse in least square method to find sphere approximation
		P, c1, rank, s = np.linalg.lstsq(A,B,rcond=None)
		
		# filter x with initial x = -0.014 and gain = 0.05
		xc_out = gain*P[0][0] + (1-gain)*xc_out
		# filter y with initial y = -0.017 and gain = 0.05
		yc_out = gain*P[1][0] + (1-gain)*yc_out
		# filter z with initial z = 0.5 and gain = 0.05
		zc_out = gain*P[2][0] + (1-gain)*zc_out
		# filter 
		p_out = gain*P[3][0] + (1-gain)*p_out
		
		# Set xc, yc, and zc in SphereParams msg
		params.xc = xc_out
		params.yc = yc_out
		params.zc = zc_out
		# set radius = 0 if radius is imaginary
		# happens often if p_out is a negative number roughly around -z_out
		# results in math domain error is not caught
		if xc_out**2 + yc_out**2 + xc_out**2 + p_out < 0:
			params.radius = 0
		# set radius to calculated value is radius is not imaginary
		else:
			params.radius = math.sqrt(xc_out**2 + yc_out**2 + zc_out**2 + p_out)
		
		print(params)

		# Publishing SphereParams
		spherePub.publish(params)		
			
		# pause until the next iteration			
		rate.sleep()
main()
