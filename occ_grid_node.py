#!/usr/bin/env python

# ROS python api with lots of handy ROS functions
import rospy

# to be able to subcribe to laser scanner data
from sensor_msgs.msg import LaserScan

# to be able to publish Twist data (and move the robot)
from geometry_msgs.msg import Twist

# to be able to subscribe to the ground truth pose
from nav_msgs.msg import Odometry

import math
import numpy as np

class grid_map(object):

	def __init__(self):
	
		self.height = 10
		self.width = 10
		self.res = 4
		self.pose = [0,0,0]
		
		self.alpha = 0.5
		self.beta = 5*np.pi/360 # 5 degrees
		self.z_max = 10  # initialized on update
		
		self.l_occ = np.log(0.7/0.3)
		self.l_free = np.log(0.3/0.7)
		self.l0 = np.log(0.5/0.5)
	
		# initialize grid
		self.grid = np.ndarray( (self.height*self.res,self.width*self.res), dtype=float)
		self.grid.fill(-1) # unkown
		# self.x = 
		
	def update_pose(self, msg):
		
		self.pose[0] = msg.pose.pose.position.x
		self.pose[1] = msg.pose.pose.position.y
		self.pose[2] = msg.pose.pose.orientation.z

	def update(self, msg):
	
		self.z_max = msg.range_max
		it = np.nditer(self.grid, flags=['multi_index'])
		while not it.finished:
			self.inv_model(it.multi_index, msg)
			it.iternext()
		self.print_map_to_file()
			
	def inv_model(self, cell, msg):

		pose = self.pose
		theta = pose[2]
		
		r = math.sqrt((cell[0] - pose[0])**2 + (cell[1] - pose[1])**2)
		phi = math.atan2((cell[1] - pose[1]), (cell[0] - pose[0])) - theta
		ang_dif = []
		ang = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

		for i in range(0,len(ang)):
			ang_dif.append(abs( phi - ang[i]))

		k = np.argmin(ang_dif)

		if r > min(self.z_max, msg.ranges[k]) or abs(phi - ang[k]) > self.beta/2:
			return self.l0
		if msg.ranges[k] < self.z_max and abs(r - msg.ranges[k]) < self.alpha/2:
			return self.l_occ
		if r <= msg.ranges[k]:
			return self.l_free

	def threshold_map(self):
		pass

	def print_map_to_file(self):
		
		with open('test_map.txt', 'w') as f:
			f.write(self.grid)

			
	
	
class behaviour(object):

	def __init__(self):
		
		# register node in ROS network
        	rospy.init_node('occupancy_grid_map', anonymous=False)
        	# print message in terminal
		rospy.loginfo('Occupancy grid mapping started !')
		# subscribe to pioneer laser scanner topic
		rospy.Subscriber("robot_0/base_scan_1", LaserScan, self.laserCallback)
		# setup publisher to later on move the pioneer base
		self.pub_cmd_vel = rospy.Publisher('robot_0/cmd_vel', Twist, queue_size=1)
		
		
	def laserCallback(self, msg):
		'''
		This function gets executed everytime a laser scanner msg is received on the
		topic: /robot_0/base_scan_1
		'''
		# ============= YOUR CODE GOES HERE! =====
		# hint: msg contains the laser scanner msg
		# hint: check http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
		middle = len(msg.ranges)/2
		delta_angle = 0.3
		delta_index = int(delta_angle/msg.angle_increment)
		useful_ranges = msg.ranges[middle - delta_index : middle + delta_index]

		self.distance = min(useful_ranges)
		
	def rotate_right(self):
		'''
		Rotate the robot by a certain angle
		'''
		# create empty message of Twist type (check http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
		twist_msg = Twist()
		# liner speed
		twist_msg.linear.x = 0.0
		twist_msg.linear.y = 0.0
		twist_msg.linear.z = 0.0
		# angular speed
		twist_msg.angular.x = 0.0
		twist_msg.angular.y = 0.0
		twist_msg.angular.z = -0.3

		# publish Twist message to /robot_0/cmd_vel to move the robot
		self.pub_cmd_vel.publish(twist_msg)


	def move_forward(self):
		'''
		Move the robot forward some distance
		'''
		# create empty message of Twist type (check http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
		twist_msg = Twist()
		# linear speed
		twist_msg.linear.x = 0.5
		twist_msg.linear.y = 0.0
		twist_msg.linear.z = 0.0
		# angular speed
		twist_msg.angular.x = 0.0
		twist_msg.angular.y = 0.0
		twist_msg.angular.z = 0.0

		# publish Twist message to /robot_0/cmd_vel to move the robot
		self.pub_cmd_vel.publish(twist_msg)

def main():
	'''
	# set grid parameters
	if rospy.has_param("occupancy_rate"):
		rate = rospy.get_param("occupancy_rate")

	if rospy.has_param("grid_resolution"):
		resolution = rospy.get_param("grid_resolution")

	if rospy.has_param("grid_width"):
		width = rospy.get_param("grid_width")

	if rospy.has_param("grid_height"):
		height = rospy.get_param("grid_height")

	# fill map_msg with the parameters from launchfile
	map_msg.info.resolution = resolution
	map_msg.info.width = width
	map_msg.info.height = height
	map_msg.data = range(width*height)
	'''
	# initialize grid map
	map = grid_map()

	rospy.init_node('mbot_occ_grid_map', anonymous=False)
	# print message in terminal
	rospy.loginfo('Mbot Occupancy Grid Mapping started !')
	# subscibe to the two laser range finders
	rospy.Subscriber('/robot_0/base_scan_1', LaserScan, map.update)
	# rospy.Subscriber('/scan2')
	rospy.Subscriber('/robot_0/base_pose_ground_truth', Odometry, map.update_pose)

	while not rospy.is_shutdown():

		pass

		
		
		
		
		
		
		
		
