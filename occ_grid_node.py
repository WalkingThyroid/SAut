#!/usr/bin/env python

# ROS python api with lots of handy ROS functions
import rospy

# to be able to subcribe to laser scanner data
from sensor_msgs.msg import LaserScan

# to be able to publish Twist data (and move the robot)
from geometry_msgs.msg import Twist

# to be able to subscribe to the ground truth pose
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseWithCovarianceStamped

import math
import numpy as np
import tf

class grid_map(object):

	def __init__(self):

		self.occ_pub = rospy.Publisher("/mbot/occMap", OccupancyGrid, queue_size = 10)

		# create tf listener
		self.listener = tf.TransformListener()
	
		self.height = 720		# cells
		self.width = 720		# cells
		self.res = 0.025		# m/cell
		self.pose = [0,0,0]
		
		self.alpha = 0.1
		self.beta = 5*np.pi/360 # 5 degrees
		self.z_max = 10  	# initialized on update
		self.z_min = 0  	# initialized on update
		
		self.l_occ = np.log(0.7/0.3)
		self.l_free = np.log(0.3/0.7)
		self.l0 = np.log(0.5/0.5)

		# list to save both update message types
		self.laser_msgs = []
		self.pose_msgs = []
	
		# initialize grid
		self.grid = np.ndarray( (self.height,self.width), dtype=float)
		self.grid.fill(0) # unkown (is it right?)

		self.grid_x = np.arange(-self.width*self.res/2.0 + self.res/2.0, self.width*self.res/2.0, self.res)
		self.grid_y = np.arange(self.height*self.res/2.0 - self.res/2.0, -self.height*self.res/2.0, -self.res)
		
		'''self.grid_x = np.ndarray( (self.height,self.width), dtype=float) # deepcopy?
		it = np.nditer(self.grid_x, flags=['multi_index'])
		while not it.finished:
			self.grid_x(it.multi_index(0), it.multi_index(1)) = it.multi_index(0)*self.res + self.res * 2
			it.iternext()''' # maybe a pre-calculated matrix with coordinates

		
	def update_pose(self, msg):
		"""Updates the robot pose after receiving an odometry message"""
		
		self.pose[0] = msg.pose.pose.position.x
		self.pose[1] = msg.pose.pose.position.y

		orientation = msg.pose.pose.orientation
		(roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
		self.pose[2] = yaw

		print("Pose was updated")

	def update(self, msg):
		"""Update the map after receiving message with measurements"""

		worked = 0
		while(not worked):
			try:
				# listen to transform
				self.listener.waitForTransform('/map', '/base_laser_front_link', rospy.Time(0), rospy.Duration(1.0))
				(trans,rot) = self.listener.lookupTransform('/map', '/base_laser_front_link', rospy.Time(0))
				worked = 1
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				print("Error looking up transform!")
				worked = 0

		self.pose[0] = trans[0]
		self.pose[1] = trans[1]
		(roll, pitch, yaw) = euler_from_quaternion(rot)
		self.pose[2] = yaw

		print("Pose was updated")

		pose = self.pose
		self.z_max = msg.range_max
		self.z_min = msg.range_min

		''' This is the rectangle approach
		# get the square on which we want to iterate
		theta = self.pose[2]
		p1 = [pose[0] + self.z_max*math.cos(theta + msg.angle_min), pose[1] + self.z_max*math.sin(theta + msg.angle_min)]
		p2 = [pose[0] + self.z_max*math.cos(theta + msg.angle_max), pose[1] + self.z_max*math.sin(theta + msg.angle_max)]
		# calculate the box coordinates
		x_max = max(p1[0], p2[0], pose[0])
		x_min = min(p1[0], p2[0], pose[0])
		y_max = max(p1[1], p2[1], pose[1])
		y_min = min(p1[1], p2[1], pose[1])
		'''

		# get the square on which we want to iterate
		x_max = pose[0] + self.z_max
		x_min = pose[0] - self.z_max
		y_max = pose[1] + self.z_max
		y_min = pose[1] - self.z_max

		# get the correct indexes of the sub-grid we want to iterate on
		#idx = [pose[0] - self.z_max/self.res
		idx_max = np.argmin(np.abs(self.grid_x - x_max))
		if idx_max != self. width - 1:
			idx_max = idx_max + 1 #This is not strictly correct, it's just to be safe
		idx_min = np.argmin(np.abs(self.grid_x - x_min))
		if idx_min != 0:
			idx_min = idx_min - 1 #This is not strictly correct, it's just to be safe
		idy_min = np.argmin(np.abs(self.grid_y - y_max))
		if idy_min != 0:
			idy_min = idy_min - 1 #This is not strictly correct, it's just to be safe
		idy_max = np.argmin(np.abs(self.grid_y - y_min))
		if idy_max != self. height - 1:
			idy_max = idy_max + 1 #This is not strictly correct, it's just to be safe

		#print(idx_min)
		#print(idx_max)
		#print(idy_min)
		#print(idy_max)

		for i in range(idx_min, idx_max+1):
			for j in range(idy_min, idy_max+1):
				self.grid[i,j] += self.inv_model([i,j], msg, pose)

		# avoid overflows
		self.grid[self.grid > 50] = 50
		self.grid[self.grid < -50] = -50
			
		print("Finished one iteration")
		self.publish_map()
		#self.print_map_to_file()
			
	def inv_model(self, cell, msg, pose):
		"""Implementation of inverse model of Probabilistic Robotics"""

		#pose = self.world_to_grid(self.pose)
		#cell_pos = self.get_cell_pos(cell)
		cell_pos = [self.grid_x[cell[0]], self.grid_y[cell[1]]]
		#cell_pos.append(self.grid_x[cell[0]])
		#cell_pos.append(self.grid_y[cell[1]])
		
		#pose[2] = self.pose[2]
		theta = pose[2]
		
		r = math.sqrt((cell_pos[0] - pose[0])**2 + (cell_pos[1] - pose[1])**2)
		phi = math.atan2((cell_pos[1] - pose[1]), (cell_pos[0] - pose[0])) - theta
		

		ang = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
		'''
		for i in range(0,len(ang)):
			#ang_dif.append(math.atan2( math.sin(phi - ang[i]), math.cos(phi - ang[i]))) # maybe?
			ang_dif.append(abs( phi - ang[i])) # not sure about this, actually
		'''

		ang_dif = np.full((1, ang.size), phi) - ang

		# wrap to [-pi, pi]
		ang_dif[ang_dif > np.pi] -= 2.0 * np.pi
		ang_dif[ang_dif < -np.pi] += 2.0 * np.pi

		ang_dif = np.abs(ang_dif)

		k = np.argmin(ang_dif)
		#print(ang_dif)
		#print("r-msg.range[k] = " + str(abs(r-msg.ranges[k])) + "::::::::alpha = " + str(self.alpha/2.0))
		if abs(r - msg.ranges[k]) < 0:
			print(cell)
			print("Pose: " + str(pose))
			print("Cell position: " + str(cell_pos))
			print("r = " + str(r))
			print("phi = " + str(phi))
			print("k = " + str(k) + ":::::msg.range(k) = " + str(msg.ranges[k]))
			print("angle(k) = " + str(ang[k]))

		# discard error measurements
		if msg.ranges[k] > self.z_max or msg.ranges[k] < self.z_min:
			return self.l0

		if r > min(self.z_max, msg.ranges[k] + self.alpha/2.0) or abs(ang_dif[0,k]) > self.beta/2.0:
			#print('Out of range')
			return self.l0
		if msg.ranges[k] < self.z_max and abs(r - msg.ranges[k]) < self.alpha/2.0:
			#print('Cell' + str(cell) + ' is occupied')
			return self.l_occ
		if r <= msg.ranges[k]:
			#print('Cell' + str(cell) + ' is free')
			return self.l_free
		else:
			#print("Error:This cell is neither occupied, free or out of range")
			return self.l0

	def inv_log_odds_map(self):
		#Inverts from log odds to get the probability in the interval [0,100]
		#Does not work as intended, 0 in log odds gets mapped to 62, should be 50

		odds = np.exp(self.grid)
		self.norm_map = odds * 100 / (odds + 1)

	def print_map_to_file(self):
		"""Prints the grid map of probabilities to a text file
		Not well implemented yet"""
		
		with open('test_map.txt','wb') as f:
			for line in self.norm_map:
				np.savetxt(f, line, fmt='%.2f')

	def world_to_grid(self, world):
		#Transformation from world coordinates to grid_coordinates
		
		grid_origin = [self.width * self.res / 2.0, self.height * self.res / 2, 0]
		return np.add(grid_origin, world)

	def grid_to_world(self, grid):
		#Transformation from grid coordinates to world coordinates
		
		grid_origin = [self.width * self.res / 2.0, self.height * self.res / 2]
		return grid - grid_origin

	def get_cell_pos(self, index):
		"""Get the coordinates of the centre of the cell in the grid frame, from the index
		Origin is at the center of the grid, cell [0,0] is at the top left corner
		"""
		pos = [0,0]
		pos[0] = (-self.width * self.res /2.0) + ((index[1] + 0.5) * self.res)
		pos[1] = (self.height * self.res /2.0) - ((index[0] + 0.5) * self.res)
		return pos
		
	def publish_map(self):
		#Publish map to topic /mbot/occMap

		map_msg = OccupancyGrid()
		map_msg.header.frame_id = 'map'
		map_msg.info.resolution = self.res
		map_msg.info.width = self.width
		map_msg.info.height = self.height
		map_msg.data = range(self.width*self.height)

		# map origin
		map_msg.info.origin.position.x = - self.width * self.res / 2.0
		map_msg.info.origin.position.y = self.height * self.res / 2.0
		[x,y,z,w] = quaternion_from_euler(0,0,-np.pi/2)
		map_msg.info.origin.orientation.x = x
		map_msg.info.origin.orientation.y = y
		map_msg.info.origin.orientation.z = z
		map_msg.info.origin.orientation.w = w
		
		# get current time
		map_msg.header.stamp = rospy.Time.now()

		# might be out of order (should be row major order)
		self.inv_log_odds_map()
		self.threshold_map() # just a test
		it = np.nditer(self.norm_map, flags=['multi_index'])
		for i in range(self.width*self.height):
			map_msg.data[i] = self.norm_map[it.multi_index]
			it.iternext()

		self.occ_pub.publish(map_msg)

	def laser_callback(self, msg):
		""" Save the laser message in a list """
		if len(self.laser_msgs) > 9:
			self.laser_msgs = self.laser_msgs[1:]

		self.laser_msgs.append(msg)
	
	def pose_callback(self, msg):
		""" Save the pose message in a list """
		if len(self.pose_msgs) > 9:
			self.pose_msgs = self.pose_msgs[1:]
		self.pose_msgs.append(msg)

	def threshold_map(self):
		self.norm_map[self.norm_map > 50] = 100
		self.norm_map[self.norm_map < 50] = 0
		self.norm_map[self.norm_map == 50] = -1



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
		"""
		This function gets executed everytime a laser scanner msg is received on the
		topic: /robot_0/base_scan_1
		"""
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

	# initialize nodes
	rospy.init_node('mbot_occ_grid_map', anonymous=False)

	# print message in terminal
	rospy.loginfo('Mbot Occupancy Grid Mapping started !')

	# create grid map object
	map = grid_map()

	# subscribe to the two laser range finders
	#rospy.Subscriber('/robot_0/base_scan_1', LaserScan, map.laser_callback, queue_size=1)
	rospy.Subscriber('/scan_front', LaserScan, map.laser_callback, queue_size=None)
	# rospy.Subscriber('/scan2')
	#rospy.Subscriber('/robot_0/base_pose_ground_truth', Odometry, map.pose_callback)
	#rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, map.pose_callback)

	while not rospy.is_shutdown():

		if map.laser_msgs:
			#map.update_pose(map.pose_msgs[-1])
			map.update(map.laser_msgs[-1])

		
		
		
		
		
		
