from functools import partial
import rospy
from optitrack.msg import *

def callback(data,filestream,starttime):
    point = data.bodies[0].pose.position
    t = (rospy.get_rostime().nsecs*1e-9+rospy.get_rostime().secs) - starttime
    filestream.write("{0},{1},{2},{3}\n".format(t,point.x,point.y,point.z))

def listener():
    with open('balloon_trajectory.csv','w') as fstream:
        rospy.init_node('balloon_recorder', anonymous=True)
        starttime  = (rospy.get_rostime().nsecs*1e-9+rospy.get_rostime().secs)
        callback_to_file = partial(callback,filestream=fstream,starttime=starttime)
        rospy.Subscriber("/optitrack/rigid_bodies", RigidBodyArray, callback_to_file)
        rospy.spin()

if __name__ == '__main__':
    listener()
