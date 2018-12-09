#!/usr/bin/python
# subscribes the information of the optitrack system and provides estimation of trajectory of the ballons

import rospy
import math
import numpy as np
import pandas as pd
from scipy.optimize import curve_fit
from scipy.signal import butter, lfilter, freqz

from optitrack.msg import RigidBodyArray
from acsi_group4.msg import BalloonArray, Balloon
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
        self.dt_predict = 0.1
        self.pose_data = pd.DataFrame(columns=['time','x','y','z'])
        self.z0 = rospy.get_param("/z0", 0.) # The true z value of the floor

    def get_data_in_timewindow(self, t_start, t_end):
        data_in_timewindow = 
            self.pose_data.loc[  
                (self.pose_data.time >= t_start)
              & (self.pose_data.time <= t_end)
              & (self.pose_data.index%1 == 0)  # gives option to only use every n-th datapoint
            ] 
        return data_in_timewindow

    @staticmethod
    def position_function(t,*params):
        v_max, kappa, t0, z0 = params
        return z0 + v_max*(t0+1/kappa) - v_max*t - v_max/kappa * np.exp(-kappa*(t-t0))

    @staticmethod
    def velocity_function(t,*params):
        v_max, kappa, t0, z0 = params
        return v_max*(1-np.exp(-kappa*(t-t0)))

    def add_datapoint(self, curr_pose):
        t = curr_pose.stamp
        x,y,z = curr_pose.x, curr_pose.y, curr_pose.z
        self.pos_data.append(pd.DataFrame(np.matrix([t,x,y,z]),
            columns=['time','x','y','z']), ignore_indices=True)

    def path_prediction(self):
        path = Path()
        path.header.frame_id = "world"
        
        t0 = rospy.get_time()

        data_in_timewindow = self.get_data_in_timewindow(t0-2,t0)
        p_opt, _ = curve_fit(BalloonPredictor.position_function, data_in_timewindow.time, data_in_timewindow.z, p0=(3,2,t0,3))

        current_position = Point(*(self.pose_data.iloc[-1][1:])) # get latest datapoint

        time_pred = np.arange(t0,t0+3,self.dt_predict)
        x_pred = np.ones(time_pred.shape)*current_position.x
        y_pred = np.ones(time_pred.shape)*current_position.y
        z_pred = BalloonPredictor.position_function(time_pred,*p_opt)

        for t,x,y,z in zip(time_pred, x_pred, y_pred, z_pred):
            if z < self.z0+0.05:
                break
            ps = PoseStamped()
            ps.header.frame_id = "world"
            ps.header.stamp = rospy.Time.from_sec(t)
            ps.pose = Point(x,y,z)

            path.poses.append(ps)

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
        self.nballoons = rospy.get_param("/nballoon", 0)
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

    def run(self):
        while not rospy.is_shutdown():
            if self.received_states:
                # publish the balloons estimation
                for i in range(self.nballoons):

                    # prediction and filtering
                    self.balloon_predictors[i].add_datapoint(self.balloon_pos[i])
                    predicted_path = self.balloon_predictors[i].path_prediction()
                    
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
            if data.bodies[i].id is not 1:
                self.balloon_pos[data.bodies[i].id - 2] = data.bodies[i].pose.position
        self.received_states = True

if __name__ == '__main__':

    rospy.init_node('baloon_predictor', anonymous=False)
    rospy.logwarn("Start balloon predictor node")
    predictor = Predictor()

    rospy.Subscriber("optitrack/rigid_bodies", RigidBodyArray, predictor.optitrack_callback)
    predictor.run()
