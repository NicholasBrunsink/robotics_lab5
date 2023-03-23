#!/usr/bin/env python3
import rospy
from robot_vision_lectures.msg import XYZarray
from geometry_msgs.msg import Point

points = []

def get_ball(ballPoints):
	global points
	points = ballPoints.points
	print(ballPoints)
	
	
	

def main():
	global points
	rospy.init_node('sphere_fit', anonymous = True)
	ballSub = rospy.Subscriber("/xyz_cropped_ball", XYZarray, get_ball)
	
	# set the loop frequency
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		#print(points)
			
		# pause until the next iteration			
		rate.sleep()
main()
