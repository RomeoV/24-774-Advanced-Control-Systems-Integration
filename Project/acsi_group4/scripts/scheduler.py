#!/usr/bin/python
# Scheduling and assigning of each target to each robot with sequence                            

import numpy as np
import heapq
import rospy
from optitrack.msg import RigidBodyArray
from geometry_msgs.msg import Point
from acsi_group4.srv import Schedule, ScheduleRequest
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Int32

class Scheduler:
    def __init__(self):
        self.rate = rospy.Rate(100)
        # get the settings
        if rospy.has_param("/nballoons"):
            self.nballoons = rospy.get_param("/nballoons", 1)
        else:
            rospy.logfatal("did not set up number of ballons, using default 1")
            self.nballoons = 1
        if rospy.has_param("/nrobots"):
            self.nrobots = rospy.get_param("/nrobots", 0)
        else:
            rospy.logfatal("did not set up number of robots, using default 1")
            self.nballoons = 1
        
        # initialize position info
        self.balloon_positions = []
        for i in range(self.nballoons):
            p = Point()
            self.balloon_positions.append(p)
        
        self.robot_positions = []
        for i in range(self.nrobots):
            p = Point()
            self.robot_positions.append(p)

        # Subscribe to the world info
        rospy.Subscriber("/optitrack/rigid_bodies", RigidBodyArray, self.position_cb)
        rospy.Service("start_scheduling", Empty, self.scheduling_cb)
        
        # Set up the service call to each robot
        self.schedule_services = []
        for i in range(self.nrobots):
            service = rospy.ServiceProxy("/crazyflie" + str(i+1) + "/get_schedule", Schedule)
            self.schedule_services.append(service)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
    
    def position_cb(self, msg):
        # update the observation of the world, the first nrobots elements are the robots
        for i in range(len(msg.bodies)):
            if msg.bodies[i].id <= self.nrobots:
                self.robot_positions[msg.bodies[i].id - 1] = msg.bodies[i].pose.position
            elif msg.bodies[i].id <= (self.nrobots + self.nballoons):
                self.balloon_positions[msg.bodies[i].id - self.nrobots - 1] = msg.bodies[i].pose.position
        # end of extracting position info
    
    def schedule_by_id(self):
        # sort by id
        balloon_sequence = []
        for i in range(self.nrobots):
            balloon_sequence.append([])
        
        robot_id = 0
        for i in range(self.nballoons):
            balloon_sequence[robot_id].append(i)
            robot_id = robot_id + 1
            if robot_id >= self.nrobots:
                robot_id = 0
        rospy.logwarn("planned schedule is {0}".format(balloon_sequence))
        return balloon_sequence


    def schedule_by_dist(self):
        balloons_open = set(range(self.nballoons))
        robot_pos = [np.array([pos.x, pos.y, pos.z]) for pos in self.robot_positions]
        balloon_pos = [np.array([pos.x, pos.y, pos.z]) for pos in self.balloon_positions]
        robot_goals = []
        for i in range(self.nrobots):
            robot_goals.append([])

        robot_idx = 0
        while len(balloons_open) is not 0:
            dist = {} 
            for b in balloons_open:
                dist[b] = np.linalg.norm(robot_pos[robot_idx] - balloon_pos[b])

            closest_balloon = min(dist, key=dist.get)
            robot_goals[robot_idx].append(closest_balloon)
            balloons_open.remove(closest_balloon)
            robot_pos[robot_idx] = balloon_pos[closest_balloon]
            robot_idx = (robot_idx+1)%self.nrobots

        rospy.logwarn("planned schedule is {0}".format(robot_goals))
        return robot_goals
     
    def scheduling_cb(self, srv):
        rospy.logwarn("start scheduling!!!!")
        
        # call the scheduling routine over here
        balloon_sequence = self.schedule_by_dist() 

        # send service request
        for i in range(self.nrobots):
            req = ScheduleRequest()
            for j in range(len(balloon_sequence[i])):
                target_balloon = Int32()
                target_balloon.data = balloon_sequence[i][j] + 1
                req.schedule.append(target_balloon)
            #rospy.logwarn("robot {0} track balloons {1}".format(i+1, req))
            
            self.schedule_services[i](req)
        return EmptyResponse()
if __name__ == '__main__':
    rospy.init_node("task_scheduler", anonymous=False)
    rospy.logwarn("Start task scheduler node")
    scheduler = Scheduler()
    scheduler.run()
