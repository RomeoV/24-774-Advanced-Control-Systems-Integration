#!/usr/bin/python
import rospy
import math
from geometry_msgs.msg import PoseStamped

def getCircle(t,T):
    x = math.cos(2*math.pi*t/T)
    y = math.sin(2*math.pi*t/T)
    return (x,y)

def getFigure8(t,T):
    x = math.cos(2*math.pi*t/T)
    y = math.sin(2*2*math.pi*t/T)
    return (x,y)
    

def talker():
    pub = rospy.Publisher('/crazyflie/goal', PoseStamped, queue_size=10)
    rospy.init_node('goal_publisher', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = rospy.Time.now()

        pose.pose.orientation.x = 0.
        pose.pose.orientation.y = 0.
        pose.pose.orientation.z = 0.
        pose.pose.orientation.w = 1.

        pose.pose.position.z = 0.5

        T = 20
        current_phase = rospy.get_time()%T
        #(x,y) = getCircle(current_phase,T)
        (x,y) = getFigure8(current_phase,T)
        pose.pose.position.x = x
        pose.pose.position.y = y

        rospy.loginfo(pose)
        pub.publish(pose)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
