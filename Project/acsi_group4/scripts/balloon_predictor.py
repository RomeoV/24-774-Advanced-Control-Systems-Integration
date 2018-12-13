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

    def get_data_in_timewindow(self, t_start, t_end, N_SKIP=1):
        data_in_timewindow = self.pose_data.loc[  (self.pose_data.time >= t_start)
                                                & (self.pose_data.time <= t_end)
                                                #& (self.pose_data.index.values%N_SKIP == 0)  # gives option to only use every n-th datapoint
                                               ] 
        return data_in_timewindow

    @staticmethod
    def position_function_exponential(t,*params):
        v_max, kappa, t0, z0 = params
        return z0 + v_max*(t0+1/kappa) - v_max*t - v_max/kappa * np.exp(-kappa*(t-t0))

    @staticmethod
    def position_function(t,*params):
        z0,vz = params
        return z0-vz*t

    @staticmethod
    def velocity_function(t,*params):
        v_max, kappa, t0, z0 = params
        return v_max*(1-np.exp(-kappa*(t-t0)))

    def add_datapoint(self, curr_pose):
        t = curr_pose.header.stamp.secs + 1e-9 * curr_pose.header.stamp.nsecs
        x,y,z = curr_pose.pose.position.x, curr_pose.pose.position.y, curr_pose.pose.position.z
        if len(self.pose_data) == 0:
            self.pose_data = self.pose_data.append(pd.DataFrame(np.matrix([t,x,y,z]),
                columns=['time','x','y','z']), ignore_index=True)
        elif t - self.pose_data.iloc[-1,0] > self.dt_predict/10:
            self.pose_data = self.pose_data.append(pd.DataFrame(np.matrix([t,x,y,z]),
                columns=['time','x','y','z']), ignore_index=True)

    def path_prediction(self):
        path = Path()
        path.header.frame_id = "world"
        
        t0 = rospy.get_time()

        latest_time = self.pose_data.iloc[-1,0]
        data_in_timewindow = self.get_data_in_timewindow(latest_time-0.3,latest_time)
        data_in_timewindow.to_csv('/tmp/balloon_data.csv',sep=',')
        if len(data_in_timewindow)<8:
            rospy.logwarn('Too few recorded datapoints in last 2 seconds to make trajectory prediction!')
            rospy.logwarn(str(data_in_timewindow))
            rospy.logwarn(str(self.pose_data.iloc[-1,0]))
            rospy.logwarn(str(t0))
            return Path()
        min_time = np.min(data_in_timewindow.time)
        data_in_timewindow.time -= min_time
        try:
            #p_opt, _ = curve_fit(BalloonPredictor.position_function, data_in_timewindow.time, data_in_timewindow.z, p0=(3,2,t0-min_time,3))
            p_opt, _ = curve_fit(BalloonPredictor.position_function, data_in_timewindow.time, data_in_timewindow.z, p0=(3,2))
            #rospy.logwarn('Num datapoints: %s',len(data_in_timewindow))
            #rospy.logwarn('Estimated params are: z0=%f, vz=%f', *p_opt)

            current_position = Point(*(self.pose_data.iloc[-1][1:])) # get latest datapoint

            time_pred = np.arange(t0,t0+3,self.dt_predict) - min_time
            x_pred = np.ones(time_pred.shape)*current_position.x
            y_pred = np.ones(time_pred.shape)*current_position.y
            z_pred = BalloonPredictor.position_function(time_pred,*p_opt)

            for t,x,y,z in zip(time_pred, x_pred, y_pred, z_pred):
                if z < self.z0+0.05:
                    break
                ps = PoseStamped()
                ps.header.frame_id = "world"
                ps.header.stamp = rospy.Time.from_sec(t+min_time)
                ps.pose.position = Point(x,y,z)

                path.poses.append(ps)

            return path
        except RuntimeError:
            rospy.logwarn("Curve fit didn't converge")
            return Path()
    
    def add_points(self, p1, p2):
        p_add = Point()
        p_add.x = p1.x + p2.x
        p_add.y = p1.y + p2.z
        p_add.z = p1.z + p2.z
        return p_add

class Predictor:
    def __init__(self):
        self.rate = rospy.Rate(10)
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
                    #self.balloon_predictors[i].add_datapoint(self.balloon_pos[i])
                    predicted_path = self.balloon_predictors[i].path_prediction()
                    
                    # publish the prediced path
                    if predicted_path is not Path(): # too little data
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
                stamped_pose = PoseStamped()
                stamped_pose.header = data.header
                stamped_pose.pose = data.bodies[i].pose
                self.balloon_predictors[data.bodies[i].id - self.nrobots - 1].add_datapoint(stamped_pose)

                # measure the radius of this balloon
                if not self.received_states:
                    r_sum = .0
                    for j in range(len(data.bodies[i].markers)):
                        dx = data.bodies[i].markers[j].x - data.bodies[i].pose.position.x
                        dy = data.bodies[i].markers[j].y - data.bodies[i].pose.position.y
                        dz = data.bodies[i].markers[j].z - data.bodies[i].pose.position.z
                        r_sum += np.sqrt(dx*dx + dy*dy + dz*dz)
                        pass
                    r_sum = r_sum/len(data.bodies[i].markers)
                    rospy.logwarn("radius is {0}".format(r_sum))
        self.received_states = True

if __name__ == '__main__':

    rospy.init_node('balloon_predictor', anonymous=False)
    rospy.logwarn("Start balloon predictor node")
    predictor = Predictor()

    predictor.run()
