#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import PyKDL
import math
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped

range_value = 0.0
x_value = 0.0
y_value = 0.0
yaw = 0.0

def callback(msg):
	global range_value
	range_value = msg.range

def posecallback(msg):
	global x_value, y_value, yaw
	x_value = msg.pose.pose.position.x
	y_value = msg.pose.pose.position.y
	
	ori_x = msg.pose.pose.orientation.x
	ori_y = msg.pose.pose.orientation.y
	ori_z = msg.pose.pose.orientation.z
	ori_w = msg.pose.pose.orientation.w
	rot = PyKDL.Rotation.Quaternion(ori_x, ori_y, ori_z, ori_w)
	yaw = rot.GetRPY()[2]
	#print(ori_w, ori_x, ori_y, ori_z, ori_z + ori_w, rot.GetRPY())
	

def move():
	sub = rospy.Subscriber('/sonar_1', Range, callback)
	amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, posecallback)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
	rospy.init_node('bug0', anonymous = True)
	rate = rospy.Rate(10)
	msg = Twist()
	global range_value
	rospy.Time.now()
	rospy.get_rostime()

	goal_x = 1.8
	goal_y = 0.0

	cumulative_distance = 0
	range_value_last = range_value
	time_period = 0.2
	t = 0.0
	forward = True
	while not rospy.is_shutdown():
		
		dis_to_ob = range_value			# the value of distance to obstacles

		x1 = goal_x - x_value			# calculate the angle between move direction and the target direction
		y1 = goal_y - y_value
		x1_y1_value = math.sqrt(x1**2 + y1**2)

		angle = math.acos(x1 / x1_y1_value)

		dis_to_goal = ((goal_x - x_value) ** 2 + (goal_y - y_value) ** 2) ** 0.5	# calculatr the distance bewteen current place and the target place
				
		if dis_to_ob < 0.3:			# if it is too near to obstacles, there are not enough space to turn around. so just turn left to avoid.
			msg.linear.x = 0
			msg.angular.z = 0.5
		
		elif abs(yaw - angle) > 0.05:		# when there are enough space to change the direction, turn the robo to the correct direction. (yaw == angle)
			if dis_to_goal < 0.5:		# if it is near to the goal, slow down to look for the goal.
				msg.linear.x = 0.2
			else:				# if it is still far away, just go.
				msg.linear.x = 0.5			
			if yaw > angle:			# turn the robort to the correct direction
				msg.angular.z = -0.2
			else:
				msg.angular.z = 0.2
		else:					# if the direction is correct and the space is enough, just move.
			msg.linear.x = 0.5
			
		
		if dis_to_goal < 0.05:			# if reach the goal, stop.
			msg.angular.z = 0
			msg.linear.x = 0
			break

		# view some value to test				
		#print(yaw, angle, dis_to_ob, x_value, y_value)

		pub.publish(msg)
		rate.sleep()

	rospy.spin()

if __name__ == '__main__':
	try:
		move()
	except rospy.ROSInterruptException:
		pass