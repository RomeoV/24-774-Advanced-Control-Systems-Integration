#!/usr/bin/python
# subscribes the information of the optitrack system and provides estimation of trajectory of the ballons

import rospy
import math
import numpy as np
from scipy.signal import butter, lfilter, freqz

from optitrack.msg import RigidBodyArray
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path

# Lowpass filter
def butter_lowpass(cutoff, fs, order):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

class BalloonPredictor:
    # Predict the balloon's trajectory based on sensing info
    def __init__(self, balloon_id):
        self.id = balloon_id
        self.vz = -2.0
        self.dt = 0.1

        # The true z value of the floor
        self.z0 = rospy.get_param("/z0", -1.4)
    def update_velocity(self, curr_position):
        # Infere about the balloon's velocity, we use a constant speed model to predit trajectory 
        self.vz = self.vz
        pass
    def path_prediction(self, curr_position):
        path = Path()
        path.header.frame_id = "world"
        
        t0 = rospy.get_time()
        position = Point()
        position.x = curr_position.x
        position.y = curr_position.y
        position.z = curr_position.z
        
        dt = .0
        t = t0
        while position.z - self.z0 >= 0.1 and dt <= 5.0:
            ps = PoseStamped()
            ps.header.frame_id = "world"
            ps.header.stamp = rospy.Time.from_sec(t)
            p = Point()
            p.x = position.x
            p.y = position.y
            p.z = position.z

            ps.pose.position = p
            path.poses.append(ps)

            # update position of next iteration
            t += self.dt
            position.z += self.vz * self.dt
        
        return path
    
    def add_points(self, p1, p2):
        p_add = Point()
        p_add.x = p1.x + p2.x
        p_add.y = p1.y + p2.z
        p_add.z = p1.z + p2.z
        return p_add

class Predictor:
    def __init__(self):
        self.rate = rospy.Rate(100)
        self.balloon_pos = []
        self.curr_t = 0
        
        self.received_states = False
        
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
        
        self.balloon_predictors = [] 
        self.balloon_prediction_pubs = []
        self.balloon_marker_pubs = []
        for i in range(self.nballoons):
            # publisher for prediction
            balloon_prediction_pub = rospy.Publisher("balloon_prediction" + str(i+1), Path, queue_size = 10)
            self.balloon_prediction_pubs.append(balloon_prediction_pub)
            balloon_visualize_pub = rospy.Publisher("balloon_position" + str(i+1), Marker, queue_size = 10)
            self.balloon_marker_pubs.append(balloon_visualize_pub)
            
            balloon_predictor = BalloonPredictor(i)
            self.balloon_predictors.append(balloon_predictor)
            self.balloon_pos.append([])
         
        # Set up subscription ot the optitrack
        rospy.Subscriber("optitrack/rigid_bodies", RigidBodyArray, self.optitrack_callback)

    def run(self):
        while not rospy.is_shutdown():
            if self.received_states:
                # publish the balloons estimation
                for i in range(self.nballoons):

                    # prediction and filtering
                    self.balloon_predictors[i].update_velocity(self.balloon_pos[i])
                    predicted_path = self.balloon_predictors[i].path_prediction(self.balloon_pos[i])
                    
                    # publish the prediced path
                    self.balloon_prediction_pubs[i].publish(predicted_path)
                    # pusblish the visualization of the balloon itself
                    self.publish_balloon_visualize(i)
            # end of prediction 
            self.rate.sleep()
    
    def publish_prediction(self, points, balloon_id):
        path = Path()
        path.header.frame_id = "world"
        for i in range(len(points)):
            ps = PoseStamped()
            ps.header.frame_id = "world"
            ps.pose.position.x = points[i].x
            ps.pose.position.y = points[i].y
            ps.pose.position.z = points[i].z
            path.poses.append(ps)
        
        self.balloon_prediction_pubs[balloon_id].publish(path)
    
    def publish_balloon_visualize(self, balloon_id):
        # published the balloon
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "baloon_visualization"
        marker.id = balloon_id
        marker.type = Marker.SPHERE
        marker.pose.position.x = self.balloon_pos[balloon_id].x
        marker.pose.position.y = self.balloon_pos[balloon_id].y
        marker.pose.position.z = self.balloon_pos[balloon_id].z
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = .4
        marker.scale.y = .4
        marker.scale.z = .4
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.5
        self.balloon_marker_pubs[balloon_id].publish(marker)

    def optitrack_callback(self, data):
        # note that id 1 is the drone
        for i in range(len(data.bodies)):
            if data.bodies[i].id > self.nrobots and data.bodies[i].id <= (self.nrobots + self.nballoons):
                self.balloon_pos[data.bodies[i].id - self.nrobots - 1] = data.bodies[i].pose.position
            
        self.received_states = True

if __name__ == '__main__':

    rospy.init_node('baloon_predictor', anonymous=False)
    rospy.logwarn("Start balloon predictor node")
    predictor = Predictor()


    predictor.run()
