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
	# Empty SphereParams msg for storing sphere to publish
	params = SphereParams()
	
	# Initialize node, subscriber, and publisher
	rospy.init_node('sphere_fit', anonymous = True)
	ballSub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, get_ball)
	spherePub = rospy.Publisher("/sphere_params", SphereParams, queue_size=10)
	
	# set the loop frequency
	rate = rospy.Rate(10)
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
		
		# Set xc, yc, zc, and radius in SphereParams msg
		params.xc = P[0]
		params.yc = P[1]
		params.zc = P[2]
		params.radius = math.sqrt(P[0]**2 + P[1]**2 + P[2]**2 + P[3])
		
		print(params)
		
		# Publishing SphereParams
		spherePub.publish(params)		
			
		# pause until the next iteration			
		rate.sleep()
main()
