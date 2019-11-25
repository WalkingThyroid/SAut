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

import math
import numpy as np

class grid_map(object):

	def __init__(self):

         	rospy.init_node('mbot_occ_grid_map', anonymous=False)
         	# print message in terminal
         	rospy.loginfo('Mbot Occupancy Grid Mapping started !')
         	# subscibe to the two laser range finders
        	rospy.Subscriber('/robot_0/base_scan_1', LaserScan, self.update)
        	# rospy.Subscriber('/scan2')
         	rospy.Subscriber('/robot_0/base_pose_ground_truth', Odometry, self.update_pose)
		self.occ_pub = rospy.Publisher("/mbot/occMap", OccupancyGrid, queue_size = 10)
	
		self.height = 72	# cells
		self.width = 72		# cells
		self.res = 0.25		# m/cell
		self.pose = [0,0,0]
		
		self.alpha = 0.5
		self.beta = 5*np.pi/360 # 5 degrees
		self.z_max = 10  	# initialized on update
		
		self.l_occ = np.log(0.7/0.3)
		self.l_free = np.log(0.3/0.7)
		self.l0 = np.log(0.5/0.5)
	
		# initialize grid
		self.grid = np.ndarray( (self.height,self.width), dtype=float)
		self.grid.fill(0.5) # unkown (is it right?)
		
		'''self.grid_x = np.ndarray( (self.height,self.width), dtype=float) # deepcopy?
		it = np.nditer(self.grid_x, flags=['multi_index'])
		while not it.finished:
			self.grid_x(it.multi_index(0), it.multi_index(1)) = it.multi_index(0)*self.res + self.res * 2
			it.iternext()''' # maybe a pre-calculated matrix with coordinates

		
	def update_pose(self, msg):
		'''Updates the robot pose after receiving an odometry message'''
		
		self.pose[0] = msg.pose.pose.position.x
		self.pose[1] = msg.pose.pose.position.y

		orientation = msg.pose.pose.orientation
		(roll, pitch, yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
		self.pose[2] = yaw

	def update(self, msg):
		'''Update the map after receiving message with measurements'''

		self.z_max = msg.range_max
		it = np.nditer(self.grid, flags=['multi_index'])
		while not it.finished:
			self.grid[it.multi_index] += self.inv_model(it.multi_index, msg)
			it.iternext()
		if it.finished:
			print("Finished one iteration")
		self.publish_map()
		#self.print_map_to_file()
			
	def inv_model(self, cell, msg):
		'''Implementation of inverse model of Probabilistic Robotics'''

		pose = self.world_to_grid(self.pose)
		cell_pos = self.get_cell_pos(cell)
		
		pose[2] = self.pose[2]
		theta = pose[2]
		
		r = math.sqrt((cell_pos[0] - pose[0])**2 + (cell_pos[1] - pose[1])**2)
		phi = math.atan2((cell_pos[1] - pose[1]), (cell_pos[0] - pose[0])) - theta
		

		ang_dif = []
		ang = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

		for i in range(0,len(ang)):
			#ang_dif.append(math.atan2( math.sin(phi - ang[i]), math.cos(phi - ang[i]))) # maybe?
			ang_dif.append(abs( phi - ang[i]))

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

		

		if r > min(self.z_max, msg.ranges[k] + self.alpha/2.0) or abs(phi - ang[k]) > self.beta/2.0:
			#print('Out of range')
			return self.l0
		if msg.ranges[k] < self.z_max and abs(r - msg.ranges[k]) < self.alpha/2.0:
			#print('Cell' + str(cell) + ' is occupied')
			return self.l_occ
		if r <= msg.ranges[k]:
			#print('Cell' + str(cell) + ' is free')
			return self.l_free
		else:
			print("Error:This cell is neither occupied, free or out of range")
			return self.l0

	def threshold_map(self):
		#Inverts from log odds to get the probability in the interval [0,100]
		#Does not work as intended, 0 in log odds gets mapped to 62, should be 50

		self.thres_map = np.exp(self.grid) * 100 / (np.exp(self.grid) + 1)

	def print_map_to_file(self):
		'''Prints the grid map of probabilities to a text file
		Not well implemented yet'''
		
		with open('test_map.txt','wb') as f:
    			for line in self.thres_map:
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
		'''Get the coordinates of the centre of the cell in the grid frame, from the index
		Origin is at the center of the grid, cell [0,0] is at the top left corner
		'''
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
		map_msg.info.origin.position.y = - self.height * self.res / 2.0
		
		# get current time
		map_msg.header.stamp = rospy.Time.now()

		# might be out of order (should be row major order)
		self.threshold_map()
		it = np.nditer(self.thres_map, flags=['multi_index'])
		for i in range(self.width*self.height):
			map_msg.data[i] = self.thres_map[it.multi_index]
			it.iternext()

		self.occ_pub.publish(map_msg)
	
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
	# create grid map object
	map = grid_map()

	while not rospy.is_shutdown():

		pass

		
if __name__ == "__main__":
    main()
		
		
				
		
		
		
		
		
		
