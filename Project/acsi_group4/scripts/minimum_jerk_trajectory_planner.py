#!/usr/bin/python
import numpy as np
import matplotlib

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped
from optitrack.msg import RigidBodyArray

matplotlib.use('Agg')
import matplotlib.pyplot as plt

def trajectory_planner(current_position, destination, moving_time, frequency = 1000):
	trajectory = []
	trajectory_derivative = []
	time_stamp = []
	timefreq = moving_time * frequency
	next_postion = 0
	next_derivative = 0
	current_position = [float(current_position[i]) for i in range(len(current_position))]
	destination = [float(destination[i]) for i in range(len(current_position))]

	for time in range(timefreq):
		time = float(time)
		increment = np.dot(np.subtract(destination, current_position),
					(10.*(time/timefreq)**3 - 15.*(time/timefreq)**4 + 6.*(time/timefreq)**5))
		next_postion = np.add(current_position, increment)
		trajectory.append(next_postion)

		increment_derivative = frequency * (30.*(time**2)*(1/timefreq)**3 \
							-60.*(time**3)*(1/timefreq)**4 + 30.*(time**4)*(1/timefreq)**5)
		next_derivative = np.dot(np.subtract(destination, current_position), increment_derivative)
		trajectory_derivative.append(next_derivative)
		time_stamp.append(time/frequency)

	return time_stamp, trajectory, trajectory_derivative


class Planner:
	def __init__(self):
		self.target = Pose()
		self.current_position = Pose()
		self.path = Path()
		self.path.header.frame_id = "world"

		self.rate = rospy.Rate(100)		
		
		rospy.Subscriber("target", Pose, self.target_cb)
		rospy.Subscriber("/optitrack/rigid_bodies", RigidBodyArray, self.position_cb)
		self.path_pub = rospy.Publisher("path", Path, queue_size = 10)

	def loop(self):
		while not rospy.is_shutdown():
			self.path_pub.publish(self.path)
			self.rate.sleep()

	def target_cb(self, msg):
		self.target = msg
		# Plan a minjerk trajectory based on current position and target position
		
		current_position = [self.current_position.position.x, self.current_position.position.y, self.current_position.position.z]
		destination = [self.target.position.x, self.target.position.y, self.target.position.z]
		frequency = 100
		moving_time = 3
		time, trajectory, trajectory_derivative = trajectory_planner(current_position, destination, moving_time, frequency)
		for i in range(len(trajectory)):
			ps = PoseStamped()
			ps.header.frame_id = "world"
			ps.pose.position.x = trajectory[i][0]
			ps.pose.position.y = trajectory[i][1]
			ps.pose.position.z = trajectory[i][2]
			self.path.poses.append(ps)
		

	def position_cb(self, msg):
		self.current_position = msg.bodies[0].pose


if __name__ == "__main__":

	rospy.init_node('minjerk_trajectory', anonymous=False)
	planner = Planner()
	planner.loop()


