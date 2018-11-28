#!/usr/bin/python
#subscribes the information of the optitrack system and provides estimation of trajectory of the ballons

import rospy
import math
import numpy as np
from scipy.signal import butter, lfilter, freqz



from optitrack.msg import RigidBodyArray
from acsi_group4.msg import BalloonArray, Balloon
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point

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



class Predictor:
    def __init__(self):
        self.rate = rospy.Rate(100)
        self.n_balloons = 0
        self.balloon_prev_pos = []
        self.balloon_pos = []
        self.prev_t = 0
        self.curr_t = 0

        self.visualization_pub = rospy.Publisher("balloons", MarkerArray, queue_size=10)
  
    def position_to_visual(self, points, marker_id):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "balloon_marker"
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = marker_id * 2
        
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        
        marker.scale.x = .01
        
        
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .0
        marker.color.b = .0
        

        for i in range(len(points)):
            marker.points.append(points[i])

        return marker


    def run(self):
        while not rospy.is_shutdown():
            # publish the balloons estimation
            balloons = BalloonArray()
            visual_msg = MarkerArray()
            for i in range(self.n_balloons):
                
                
                # prediction and filtering
                b = Balloon()

                b.x = self.balloon_pos[i].x
                b.y = self.balloon_pos[i].y
                b.z = self.balloon_pos[i].z
                
                # do the velocity estimation over here
                #b.vz = (self.balloon_pos[i].z - self.balloon_prev_pos[i].z)/(self.curr_t - self.prev_t)
                balloons.balloons.append(b)
                
                dt = self.curr_t-self.prev_t
                
                if len(b.z) > 3:
                    b.vz = (b.z[2]-b.z[1])/dt
                    
                b_p = Balloon()
                b_p.x = b.x[0]
                b_p.y = b.y[0]
                b_p.z = b.z[0]
                count = 1;
                
                while b_p.z >= 50.0 * 0.001:
                    balloons.balloons.append(b_p)
                    b_p.x = b.x[0]
                    b_p.y = b.y[0]
                    b_p.z = b.z+count*b.vz
                    count += 1
                    
                # Filter data
                order = 2
                fs = 100       # sample rate, Hz
                cutoff = 10 # cutoff frequency
                b, a = butter_lowpass(cutoff, fs, order)
                b_f = Balloon()
                b_f.x = b_p.x
                b_f.y = b_p.y
                b_f.z = butter_lowpass_filter(b_p.z, cutoff, fs, order)

                # generate a pseudo array storing predicted positions
                
                points = []
                for j in range(10):
                    p = Point()
                    p.x = b.x + 0.1*j
                    p.y = b.y + 0.1*j
                    p.z = b.z + 0.1*j
                    points.append(p)



                balloon_marker = self.position_to_visual(points, i)
                visual_msg.markers.append(balloon_marker)


            self.visualization_pub.publish(visual_msg)
            
            
            self.rate.sleep()
            

    def optitrack_callback(self, data):
        # note that id 1 is the drone
        self.n_balloons = len(data.bodies) - 1 
        if len(self.balloon_pos) == 0:
            for i in range(self.n_balloons):
                self.balloon_pos.append(data.bodies[i+1].pose.position)
        else:
            self.balloon_prev_pos = self.balloon_pos
            for i in range(self.n_balloons):
                self.balloon_pos[i] = data.bodies[i+1].pose.position
            self.prev_t = self.curr_t
            self.curr_t = rospy.get_time() 

if __name__ == '__main__':

    rospy.init_node('baloon_predictor', anonymous=False)
    predictor = Predictor()

    rospy.Subscriber("optitrack/rigid_bodies", RigidBodyArray, predictor.optitrack_callback)
    predictor.run()
