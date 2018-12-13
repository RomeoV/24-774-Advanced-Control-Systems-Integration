#!/usr/bin/python
import numpy as np
import matplotlib
import pandas as pd
from scipy.interpolate import interp1d

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Point
from optitrack.msg import RigidBodyArray
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Empty as EmptyService, EmptyResponse
from acsi_group4.srv import Schedule, ScheduleResponse
from min_jerk import solve_equation, build_equation

matplotlib.use('Agg')
import matplotlib.pyplot as plt

def trajectory_planner(current_position, destination, moving_time, frequency = 1000):
    # Minjerk planner
    trajectory = []
    trajectory_derivative = []
    time_stamp = []
    timefreq = moving_time * frequency
    next_postion = 0
    next_derivative = 0
    #current_position = [float(current_position[i]) for i in range(len(current_position))]
    #destination = [float(destination[i]) for i in range(len(current_position))]

    ros_time = rospy.Time.now()
    current_time = ros_time.secs + 1e-9*ros_time.nsecs

    for time in np.arange(0,timefreq,1./frequency):
        time = float(time)
        increment = np.dot(destination-current_position,
                    (10.*(time/timefreq)**3 - 15.*(time/timefreq)**4 + 6.*(time/timefreq)**5))
        next_postion = current_position+increment
        trajectory.append(next_postion)

        increment_derivative = frequency * (30.*(time**2)*(1/timefreq)**3 \
                            -60.*(time**3)*(1/timefreq)**4 + 30.*(time**4)*(1/timefreq)**5)
        next_derivative = np.dot(np.subtract(destination, current_position), increment_derivative)
        trajectory_derivative.append(next_derivative)
        time_stamp.append(time/frequency + current_time)

    return time_stamp, trajectory, trajectory_derivative

def trajectory_planner_with_speed(current_position, destination, current_velocity, desired_velocity, moving_time, frequency = 1000):
    trajectory = []
    trajectory_derivative = []
    time_stamp = []
    timefreq = moving_time * frequency
    next_postion = 0
    next_derivative = 0
    # current_position = [float(current_position[i]) for i in range(len(current_position))]
    # destination = [float(destination[i]) for i in range(len(current_position))]
    # timefreq = float(timefreq)
    
    params_all = [solve_equation(current_position[dir], destination[dir], current_velocity[dir], desired_velocity[dir], moving_time) for dir in range(3)]
    params = [p[0] for p in params_all]
    params_vel = [p[1] for p in params_all]

    ros_time = rospy.Time.now()
    current_time = ros_time.secs + 1e-9*ros_time.nsecs

    for time in np.arange(0,moving_time,1./frequency):
        pos = np.array([0,0,0])
        pos_vel_list = np.array([build_equation(params[dir],params_vel[dir],time) for dir in range(3)])
        pos = pos_vel_list[:,0]
        vel = pos_vel_list[:,1]
        trajectory.append(pos)
        trajectory_derivative.append(vel)
        time_stamp.append(time+current_time)

    return time_stamp, trajectory, trajectory_derivative

class Planner:
    def __init__(self):
        if not rospy.has_param("/vmax"):
            rospy.logfatal("did not set up vmax, using default value 0.5")
            self.robot_vmax = 0.5
        else:
            self.robot_vmax = rospy.get_param("/vmax", 0.5)
        self.target = Point()
        self.current_position = Pose()
        self.path_as_dataframe = pd.DataFrame()
        self.path = Path()
        self.path_count = 0
        self.path.header.frame_id = "world"
        self.active = False
        self.freq = 10
        self.rate = rospy.Rate(self.freq)       
        
        # extract the digit from namespace
        ns = rospy.get_namespace()
        for i in range(len(ns)):
            if ns[i].isdigit():
               self.robot_id = int(ns[i])
               break
        
        if self.robot_id == None:
            rospy.logfatal("Did not set up crazyflie namespace with id!!")


        if rospy.has_param("/nballoons"):
            self.nballoons = rospy.get_param("/nballoons", 1)
        else:
            rospy.logfatal("did not set up nballoons, using default 1")
            self.nballoons = 1
        if rospy.has_param("/nrobots"):
            self.nrobots = rospy.get_param("/nrobots", 1)
        else:
            rospy.logfatal("did not set up nrobot, using default 1")
            self.nrobots = 1
        
        self.balloon_predictions = []
        for i in range(self.nballoons):
            # Set up the subscriber for each balloon
            cb_name = "/balloon_prediction" + str(i+1)
            _sub_func = self.make_sub_func(i)
            setattr(self, cb_name, _sub_func)
            sub_func = getattr(self, cb_name)
            rospy.Subscriber(cb_name, Path, sub_func)
            
            # Initialize the predictions with a empty path
            path = Path()
            self.balloon_predictions.append(path)
        

        self.target_balloon_id = self.robot_id
        self.target_sequence = [] 
        # Subscribe to world info
        rospy.Subscriber("/optitrack/rigid_bodies", RigidBodyArray, self.position_cb)
        
        # to plan a target currently has three interfaces, specify target, start tracking balloon one, and provide a sequence of targets
        rospy.Subscriber("target", Point, self.target_cb)
        rospy.Service("start_catching", EmptyService, self.start_catching_service)
        rospy.Service("get_schedule", Schedule, self.get_schedule_cb)
        # Send commands to robot
        self.control_target_pub = rospy.Publisher("control_target", PoseStamped, queue_size = 10)
        self.path_pub = rospy.Publisher("path", Path, queue_size = 10)
        self.balloon_target_visualization_pub = rospy.Publisher("balloon_target_visual", Marker, queue_size = 10)
        self.control_target_visualization_pub = rospy.Publisher("control_target_visual", Marker, queue_size = 10)


    def make_sub_func(self, bid):
        def _func(msg):
            self.balloon_predictions[bid] = msg
        return _func

    def loop(self):
        while not rospy.is_shutdown():
            if self.active:
                # check if we need re-plan
                # only prediction could go wrong only because we are tracking very good
                self.check_replan()
                # visualization 
                self.path_pub.publish(self.path)
                self.publish_balloon_target_visualization()
                
                if len(self.path.poses) >= 1:
                    # Interpolation of the target over here
                    ros_time = rospy.Time.now()
                    current_time = ros_time.secs + 1e-9*ros_time.nsecs
                    control_target = self.interpolatePath(current_time)
                    self.publish_control_target_visualization(control_target)
                    
                    self.control_target_pub.publish(control_target)
            self.rate.sleep()

    def interpolatePath(self, time):
        if time < self.path_as_dataframe.time.min():
            pos = self.path_as_dataframe.iloc[0,1:4]
        elif time > self.path_as_dataframe.time.max():
            pos = self.path_as_dataframe.iloc[-1,1:4]
            if time > self.path_as_dataframe.time.max() + 1.:
                rospy.logwarn("Reached target balloon {0}!".format(self.target_balloon_id))
                if len(self.target_sequence) > 0:
                    # multiple balloons to catch, catch next one
                    rospy.logwarn("robot {0} has more balloons to catch!".format(self.robot_id))
                    self.target_balloon_id = self.target_sequence[0]
                    del self.target_sequence[0]
                    self.catch_target_balloon()
                else:
                    self.active = False
                    self.path_as_dataframe = pd.DataFrame(columns=['time','x','y','z','vx','vy','vz'])
        
        else:
            pos_inter = interp1d(self.path_as_dataframe.time, self.path_as_dataframe.iloc[:,1:4], axis=0)
            pos = pos_inter(time)
            #rospy.logwarn("Interpolating target!")

        control_target = PoseStamped()
        control_target.pose.position.x = pos[0]
        control_target.pose.position.y = pos[1]
        control_target.pose.position.z = pos[2]
        control_target.pose.orientation.x = 0
        control_target.pose.orientation.y = 0
        control_target.pose.orientation.z = 0
        control_target.pose.orientation.w = 1

        return control_target

    def interpolateCurrentVelocity(self, time):
        if time < self.path_as_dataframe.time.min():
            vel = self.path_as_dataframe.iloc[0,4:7]
        elif time > self.path_as_dataframe.time.max():
            vel = self.path_as_dataframe.iloc[-1,4:7]
            vel = np.array([0,0,0])
        else:
            vel_inter = interp1d(self.path_as_dataframe.time, self.path_as_dataframe.iloc[:,4:7], axis=0)
            vel = vel_inter(time)
            #rospy.logwarn("Interpolating target!")
        return vel


    def check_replan(self):
        #pass
        # check if the original plan still meets the preditcion
        # Always replan
        #### ROMEO LOOK HERE TOMORROW
        return
        self.catch_target_balloon()
    
    
    def publish_balloon_target_visualization(self):
        # published the target on the balloon prediction trajectory
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "balloon_target_visualization"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.pose.position.x = self.target.x
        marker.pose.position.y = self.target.y
        marker.pose.position.z = self.target.z
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = .05
        marker.scale.y = .05
        marker.scale.z = .05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.balloon_target_visualization_pub.publish(marker)

    def publish_control_target_visualization(self, control_target):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "control_target_visualization"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.pose.position.x = control_target.pose.position.x
        marker.pose.position.y = control_target.pose.position.y
        marker.pose.position.z = control_target.pose.position.z
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = .05
        marker.scale.y = .05
        marker.scale.z = .05
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        self.balloon_target_visualization_pub.publish(marker)

    def target_cb(self, msg):
        self.active = True
        self.plan_trajectory(msg)
        rospy.logwarn("Received target {0}, planned path length {1}".format(msg, len(self.path.poses)))

    def position_cb(self, msg):
        # updates the position info of drone from optitrack
        for i in range(len(msg.bodies)):
            if msg.bodies[i].id == self.robot_id:
                self.current_position = msg.bodies[i].pose
    
    def get_schedule_cb(self, srv):
        # Get the target catching sequence
        #rospy.logwarn("robot {0} got schedule {1}".format(self.robot_id, srv))

        #rospy.loginfo("got target sequence {0}".format(self.target_sequence))
        self.active = True
        self.target_sequence = []
        for i in range(len(srv.schedule)):
            self.target_sequence.append(srv.schedule[i].data)
        rospy.logwarn("robot {0} got catching sequence {1}".format(self.robot_id, self.target_sequence))
        self.target_balloon_id = self.target_sequence[0]
        del self.target_sequence[0]
        self.catch_target_balloon()
        return ScheduleResponse()
    
    def plan_trajectory(self, target_point):
        self.target = target_point
        
        # Plan a Minjerk trajectory based on current position and target position
        # Clear previous path
        self.path.poses = []
        self.path_count = 0
        current_position = np.array([self.current_position.position.x, self.current_position.position.y, self.current_position.position.z])
        if len(self.path_as_dataframe) == 0: # Assume velocity zero
            current_velocity = np.array([.0, .0, .0])
        else: # Read current velocity from path
            ros_time = rospy.Time.now()
            current_time = ros_time.secs + 1e-9*ros_time.nsecs
            current_velocity = self.interpolateCurrentVelocity(current_time)
        
        destination = np.array([self.target.x, self.target.y, self.target.z])
        destination_velocity = np.array([.0, .0, 0.5])
        
        moving_time = np.linalg.norm(current_position - destination)/self.robot_vmax

        #time, trajectory, trajectory_derivative = trajectory_planner(current_position, destination, moving_time, self.freq)
        time, trajectory, trajectory_derivative = trajectory_planner_with_speed(current_position, destination, current_velocity, destination_velocity, moving_time, self.freq)

        # Save path and velocities to dataframe
        positions = np.array(trajectory)
        velocities = np.array(trajectory_derivative)
        self.path_as_dataframe = pd.DataFrame(np.array([np.array(time), 
                                    positions[:,0], positions[:,1], positions[:,2], 
                                    velocities[:,0], velocities[:,1], velocities[:,2]]).T, 
                                    columns=['time','x','y','z','vx','vy','vz'])
        # make this plan into a plan data structure
        for i in range(len(trajectory)):
            ps = PoseStamped()
            ps.header.frame_id = "world"
            ps.pose.position.x = trajectory[i][0]
            ps.pose.position.y = trajectory[i][1]
            ps.pose.position.z = trajectory[i][2]
            self.path.poses.append(ps)
        #rospy.logwarn("Received signal to track balloon , planned path length {1}".format(self.target_balloon_id, len(self.path.poses)))

        pass

    def catch_target_balloon(self):
        #rospy.logwarn("robot {0} got catching request to catch balloon {1}!!".format(self.robot_id, self.target_balloon_id))
        # back track the prediction to find the suitable point to pursuit
        current_position = np.array([self.current_position.position.x, self.current_position.position.y, self.current_position.position.z])
        current_time = rospy.Time.now()
        current_time_in_s = current_time.secs +  1e-9*current_time.nsecs
        for pred in reversed(self.balloon_predictions[self.target_balloon_id-1].poses):
            balloon_pos = np.array([pred.pose.position.x, pred.pose.position.y, pred.pose.position.z])
            balloon_time = pred.header.stamp.secs + 1e-9*pred.header.stamp.nsecs
            robot_dist = np.linalg.norm(balloon_pos - current_position)
            robot_fly_time = robot_dist/self.robot_vmax

            if balloon_time+0.1 <= robot_fly_time+current_time_in_s:
                break
        else:
            rospy.logwarn("No suitable point found for balloon catching")
            self.active = False
            return False

        self.target = pred.pose.position
        self.target.z -= 0.
        self.plan_trajectory(self.target)
        return True

    def start_catching_service(self, srv):
        self.active = True
        self.catch_target_balloon()

        return EmptyResponse()

if __name__ == "__main__":

    rospy.init_node('trajectory_planner', anonymous=False)
    planner = Planner()
    planner.loop()


