#!/usr/bin/env python

import roslib; roslib.load_manifest('project2')
import rospy
import random
import numpy
import tf
import transform2d

from geometry_msgs.msg import Twist
from kobuki_msgs.msg import SensorState

# control at 100Hz
CONTROL_PERIOD = rospy.Duration(0.01)

# wait time between actions
WAIT_DURATION = rospy.Duration(0.5)

ANGULAR_TOL = 0.01 # rad (about 1/2 degree)
ANGULAR_RAMP_TIME = 2.0 # s
ANGULAR_RAMP_SPEED = 1.5 # rad/s
ANGULAR_MIN_SPEED = 0.2 # rad/s
ANGULAR_GAIN = 3.0 # rad/s per rad, so actually just 1/s

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
        self.reset_state('waiting')

        # wait for the very first pose of the robot (so we can check at end)
        self.very_first_pose = None

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

    def reset_state(self, state):

        self.state = state
        self.start_pose = None
        self.start_time = rospy.get_rostime()

    # called whenever sensor messages are received
    def sensor_callback(self, msg):

        if msg.cliff or msg.bumper:
            rospy.loginfo('quitting because picked up/bumped')
            rospy.signal_shutdown('safety stop')

    # get current pose from TransformListener and convert it into a transform2d
    def get_current_pose(self):

        try:
            ros_xform = self.odom_listener.lookupTransform(
                '/odom', '/base_footprint',
                rospy.Time(0))

        except tf.LookupException:
            return None

        xform2d = transform2d.transform2d_from_ros_transform(ros_xform)

        return xform2d

    # called to figure angular speed
    def compute_turn_vel(self, rel_pose, desired_angle, time):

        angle_error = desired_angle - rel_pose.theta
        done = (abs(angle_error) < ANGULAR_TOL)

        rospy.loginfo('angle_error={}'.format(angle_error))

        angular_vel = ANGULAR_GAIN * angle_error

        sign = numpy.sign(angular_vel)
        clamped_vel = max(abs(angular_vel), ANGULAR_MIN_SPEED)
        
        angular_vel = sign*clamped_vel

        effective_time = min(time, ANGULAR_RAMP_TIME)
        vmax = effective_time * ANGULAR_RAMP_SPEED/ANGULAR_RAMP_TIME
            
        angular_vel = numpy.clip(angular_vel, -vmax, vmax)

        cmd_vel = Twist()

        cmd_vel.angular.z = angular_vel

        return cmd_vel, done
            
    # called 100 times per second
    def control_callback(self, timer_event=None):

        # initialize commanded vel to 0, 0
        cmd_vel = Twist()

        cur_pose = self.get_current_pose()

        if cur_pose is not None:
            if self.start_pose is None:
                self.start_pose = cur_pose.copy()
            rel_pose = self.start_pose.inverse() * cur_pose
            rospy.loginfo('rel_pose: {}'.format(rel_pose))
        else:
            rel_pose = None

        state_duration = rospy.get_rostime() - self.start_time
            
        if self.state == 'waiting':
            
            if cur_pose is None:
                rospy.loginfo('waiting for start pose')
            else:
                rospy.loginfo('ready!')
                self.very_first_pose = cur_pose.copy()
                self.reset_state('turnleft')

        elif self.state == 'waitleft':

            if state_duration > WAIT_DURATION:
                self.reset_state('turnleft')

        elif self.state == 'turnleft':
            
            cmd_vel, done = self.compute_turn_vel(rel_pose, numpy.pi/2,
                                                  state_duration.to_sec())
            
            if done:
                self.reset_state('waitright')

        elif self.state == 'waitright':

            if state_duration > WAIT_DURATION:
                self.reset_state('turnright')
                
        elif self.state == 'turnright':

            cmd_vel, done = self.compute_turn_vel(rel_pose, -numpy.pi/2,
                                                  state_duration.to_sec())
            
            if done:
                self.reset_state('waitleft')
            
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
    
