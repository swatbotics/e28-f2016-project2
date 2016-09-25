#!/usr/bin/env python

import roslib; roslib.load_manifest('project2')
import rospy
import random
import numpy
import tf

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState

# control at 100Hz
CONTROL_PERIOD = rospy.Duration(0.01)

# react to bumpers for 1 second
BUMPER_DURATION = rospy.Duration(1.0)

# radians per second squared
MAX_ANGULAR_ACCEL = 1.0

# meters per second squared
MAX_LINEAR_ACCEL = 1.0

# converts radians to radians per second
ANGULAR_GAIN = 0.1

# our controller class
class Controller:

    # called when an object of type Controller is created
    def __init__(self):

        # initialize rospy
        rospy.init_node('starter')

        # set up publisher for commanded velocity
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity',
                                           Twist)

        # start out in waiting state
        self.state = 'waiting'

        # no start pose yet
        self.start_pose = None

        # we always store the previous command from the control callback
        self.prev_vel = Twist()

        # set up subscriber for sensor state for bumpers/cliffs
        rospy.Subscriber('/mobile_base/sensors/core',
                         SensorState, self.sensor_callback)

        # TODO: set up other publishers/subscribers here if necessary

        # set up control timer at 100 Hz
        rospy.Timer(CONTROL_PERIOD, self.control_callback)

        # set up a TransformListener to get odometry information
        self.odom_listener = tf.TransformListener()

    # called whenever sensor messages are received
    def sensor_callback(self, msg):

        if msg.cliff or msg.bumper:
            rospy.loginfo('quitting because picked up/bumped')
            sys.exit(0)

    # get current pose from TransformListener and convert it into a transform2d
    def get_current_pose(self):

        try:
            ros_xform = self.odom_listener.lookupTransform(
                '/odom', '/base_footprint',
                rospy.Time(0))

        except tf.LookupException:
            return None

        xform2d = transform2d.transform2d_from_ros_transform(ros_xform)

        if self.start_pose is None:
            self.start_pose = xform2d.copy()

        return xform2d

    # called to cap max change on velocities
    def filter_cmd(self, cmd_vel):

        dt = CONTROL_PERIOD.to_sec()
        max_angular_vel_change = MAX_ANGULAR_ACCEL * dt
        max_linear_vel_change = MAX_LINEAR_ACCEL * dt

        angular_delta = numpy.clip(prev_vel.angular.z - cmd_vel.angular.z,
                                   -max_angular_vel_change,
                                   max_angular_vel_change)

        linear_delta = numpy.clip(prev_vel.linear.x - cmd_vel.linear.x,
                                  -max_linear_vel_change,
                                  max_linear_vel_change)

        filtered_vel = Twist()
        filtered_vel.angular.z = prev_vel.angular.z + angular_delta
        filtered_vel.linear.x = prev_vel.linear.x + linear_delta

        return filtered_vel

    # called to figure angular speed
    def compute_turn_vel(self, rel_pose, desired_angle):
        
        angle_error = transform2d.diff_angle_rad(rel_pose, desired_angle)
        
        cmd_vel = Twist()
        
        cmd_vel.angular.z = ANGULAR_GAIN*angle_error

        done = (angle_error < 0.001)

        return self.filter_cmd(cmd_vel), done
            
    # called 100 times per second
    def control_callback(self, timer_event=None):

        # initialize commanded vel to 0, 0
        cmd_vel = Twist()

        cur_pose = self.get_current_pose()
        if cur_pose is not None:
            rel_pose = self.start_pose.inverse() * cur_pose

        if self.state == 'waiting':
            
            if self.start_pose is None:
                rospy.loginfo('waiting for start pose')
            else:
                rospy.loginfo('ready!')
                self.state = 'turnleft'

        elif self.state == 'turnleft':
            
            cmd_vel, done = self.compute_turn_vel(rel_pose, math.pi/2)
            
            if done:
                self.start_pose = cur_pose
                self.state = 'turnright'
                rospy.loginfo('entering state' + self.state)
                
        elif self.state == 'turnright':

            cmd_vel, done = self.compute_turn_vel(rel_pose, -math.pi/2)
            
            if done:
                self.start_pose = cur_pose
                self.state = 'turnleft'
                rospy.loginfo('entering state' + self.state)
            
        self.cmd_vel_pub.publish(cmd_vel)
        self.prev_vel = cmd_vel
        
    # called by main function below (after init)
    def run(self):
        
        # timers and callbacks are already set up, so just spin.
        # if spin returns we were interrupted by Ctrl+C or shutdown
        rospy.spin()


# main function
if __name__ == '__main__':
    try:
        ctrl = Controller()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass
    
