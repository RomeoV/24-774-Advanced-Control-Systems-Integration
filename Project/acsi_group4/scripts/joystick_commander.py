#!/usr/bin/env python

"""
This node sends joystick command to activate services such as start, emergency, land, takeoff

 Y 
X B
 A


"""


import rospy
from sensor_msgs.msg import Joy
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty

class Controller():
    def __init__(self, use_controller, joy_topic):
        
        # Services provided by crazyflie_server
        rospy.logwarn("waiting for services from crazyflie server")
        try: 
            rospy.wait_for_service('update_params', 5.0)
            rospy.loginfo("found update_params service")
            self._update_params = rospy.ServiceProxy('update_params', UpdateParams)
            
            rospy.loginfo("waiting for emergency service")
            rospy.wait_for_service('emergency', 5.0)
            rospy.loginfo("found emergency service")
            self._emergency = rospy.ServiceProxy('emergency', Empty)

        except (rospy.ServiceException, rospy.ROSException):
            rospy.logwarn("couldn't find service from crazy flie server")

         
        # Services provided by position controller
        rospy.logwarn("waiting for services from position controller")
        try:
            rospy.loginfo("waiting for land service")
            rospy.wait_for_service('land')
            rospy.loginfo("found land service")
            self._land = rospy.ServiceProxy('land', Empty)

            rospy.loginfo("waiting for takeoff service")
            rospy.wait_for_service('takeoff')
            rospy.loginfo("found takeoff service")
            self._takeoff = rospy.ServiceProxy('takeoff', Empty)
        
        except (rospy.ServiceException, rospy.ROSException):
            rospy.logwarn("couldn't find service from position controller")
        
        # Services provided by trajectory planner
        rospy.logwarn("waiting for services from trajectroy planner")
        try:
            rospy.loginfo("waiting for catch ball service")
            rospy.wait_for_service('start_catching')
            rospy.loginfo("found start_catching service")
            self._catch = rospy.ServiceProxy('start_catching', Empty)
        
        except (rospy.ServiceException, rospy.ROSException):
            rospy.logwarn("couldn't find service from trajectory planner")
        
        
        # subscribe to the joystick at the end to make sure that all required
        # services were found
        self._buttons = None
        rospy.Subscriber(joy_topic, Joy, self._joyChanged)

    def _joyChanged(self, data):
        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != self._buttons[i]:
                if i == 0 and data.buttons[i] == 1 and self._takeoff != None:
                    self._takeoff()
                if i == 1 and data.buttons[i] == 1 and self._emergency != None:
                    self._emergency()
                if i == 2 and data.buttons[i] == 1 and self._land != None:
                    self._land()
                if i == 3 and data.buttons[i] == 1 and self._catch != None:
                    self._catch()
                if i == 4 and data.buttons[i] == 1:
                    value = int(rospy.get_param("ring/headlightEnable"))
                    if value == 0:
                        rospy.set_param("ring/headlightEnable", 1)
                    else:
                        rospy.set_param("ring/headlightEnable", 0)
                    self._update_params(["ring/headlightEnable"])
                    print(not value)

        self._buttons = data.buttons

if __name__ == '__main__':
    rospy.init_node('joystick_commander', anonymous=True)
    use_controller = rospy.get_param("~use_crazyflie_controller", False)
    joy_topic = rospy.get_param("~joy_topic", "joy")
    controller = Controller(use_controller, joy_topic)
    rospy.spin()
