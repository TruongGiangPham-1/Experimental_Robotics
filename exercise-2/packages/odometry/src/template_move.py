#!/usr/bin/env python3

# import required libraries
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped, WheelsCmdStamped
import math
import time
import numpy as np

# throttle and direction for  each wheel
THROTTLE_LEFT = 0.4        # 50% throttle
DIRECTION_LEFT = 1         # forward
THROTTLE_RIGHT = 0.4       # 30% throttle
DIRECTION_RIGHT = 1       # backward

TICK_TOTAL = 135

class MoveNode(DTROS):
    def __init__(self, node_name):
        #super(MoveNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # add your code here

        # initialize the DTROS parent class
        super(MoveNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        # for wheel subscriber topic ----------------------------------------------------
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        #self._left_vel_topic = f"/{self._vehicle_name}/encoder_node/encoder_velocity/vel_encoder"
        # temporary data storage
        self._ticks_left = None
        self._ticks_right = None
        self._ticks_left_init = None
        self._ticks_right_init = None
        # construct subscriber
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)

        self.sub_vel = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.vel_callback)

        # for wheel pulisher topic -------------------------------------------------------
        # url: https://docs.duckietown.com/daffy/devmanual-software/beginner/ros/wheel-control.html
        wheels_topic = f"/{self._vehicle_name}/wheels_driver_node/wheels_cmd"

        # form the message
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT
        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

        self.radius = rospy.get_param(f'/{self._vehicle_name}/kinematics_node/radius', 0.0325)  # in meter
        self.distance_between_wheels = rospy.get_param(f'/{self._vehicle_name}/kinematics_node/baseline', 0.1) # in meter


        #print(f'radiums {radius}')
        # subscribe to the left and right wheel encoder topics
        # publish to the wheels command topic

        # LEDs

        self.time_init = None
        self.robot_frame = np.zeros(3, dtype=float)
        self.robot_frame_right = np.zeros(3, dtype=float)
        self.robot_frame_left = np.zeros(3, dtype=float)
        # define other variables    
        pass
        
    def vel_callback(self, data):

        return

    def callback(self, data):
        # add your code here
        # can define one or two depending on how you want to implement      
        # log general information once at the beginning
        pass
        
    def callback_left(self, data):
        # log general information once at the beginning
        #rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        #rospy.loginfo_once(f"Left encoder type: {data.type}")
        # store data value
        if self._ticks_left == None:
            self._ticks_left_init = data.data
            self.time_start = time.time()
        self._ticks_left = data.data
        self._left_wheel_speed = (self._ticks_left - self._ticks_left_init) / (time.time() - self.time_start)

        self.delta_x_left = (2*self.radius*math.pi*(self._ticks_left - self._ticks_left_init)) / TICK_TOTAL

    def callback_right(self, data):
        # log general information once at the beginning
        #rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        #rospy.loginfo_once(f"Right encoder type: {data.type}")
        # store data value

        if self._ticks_right == None:
            self._ticks_right_init = data.data
            self.time_start = time.time()
        self._ticks_right = data.data

        self._right_wheel_speed = (self._ticks_right - self._ticks_right_init) / (time.time() - self.time_start)
        self.delta_x_right = (2*self.radius*math.pi*(self._ticks_right - self._ticks_right_init)) / TICK_TOTAL

        #self.robot_frame[2] += (self.radius* (self._vel_right)) / 2*self.distance_between_wheels

    def compute_distance_traveled(self, **kwargs):
        # add your code here

        pass
    
    
    def rotate_clockwise(self, angle):
        # add your code here
        rospy.sleep(2)  # wait for the node to initialize

        turn_msg = WheelsCmdStamped(vel_left=self._vel_left, vel_right=-self._vel_right)
        rate = rospy.Rate(20)
        print(f'numerators - {self.delta_x_left - self.delta_x_right}')

        print(f'dist {self.distance_between_wheels}')
        while not rospy.is_shutdown():
            self._publisher.publish(turn_msg)
            print(f'distance right {self.delta_x_right}  left {self.delta_x_left} ')
            self.robot_frame[2] = (np.abs(self.delta_x_left) + np.abs(self.delta_x_right)) / (self.distance_between_wheels)
            print(f"robot angle = {self.robot_frame[2]}")
            #if (self.robot_frame[2] - angle) % 2*math.pi < 0.1:
            if self.robot_frame[2] > angle:
                stop = WheelsCmdStamped(vel_left=0, vel_right=0)
                self._publisher.publish(stop)
                return
            rate.sleep()
        return

    def rotate_anticlockwise(self, angle):
        # add your code here
        rospy.sleep(1)  # wait for the node to initialize
        print(f'in anticlockwise')
        turn_msg = WheelsCmdStamped(vel_left=-self._vel_left, vel_right=self._vel_right)
        rate = rospy.Rate(20)
        init_angle = self.robot_frame[2]
        self._ticks_left_init = self._ticks_left
        self._ticks_right_init = self._ticks_right
        rospy.sleep(1)
        while not rospy.is_shutdown():
            print(f'here')
            self._publisher.publish(turn_msg)
            self.robot_frame[2] = (np.abs(self.delta_x_left) + np.abs(self.delta_x_right)) / (self.distance_between_wheels)
            print(f"robot angle = {self.robot_frame[2]}")
            #if (self.robot_frame[2] - angle) % 2*math.pi < 0.1:
            if self.robot_frame[2] > angle:
                stop = WheelsCmdStamped(vel_left=0, vel_right=0)
                self._publisher.publish(stop)
                return
            rate.sleep()

    def use_leds(self, **kwargs):
        # add your code here
        pass

    # define other functions if needed
    
    def drive_straight_n_back(self):
        rospy.sleep(2)  # wait for the node to initialize

        # add your code here
        # call the functions you have defined above for executing the movements

        # publish received tick messages every 0.05 second (20 Hz)
        is_forward = True
        rate = rospy.Rate(10)
        forward_message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
        backward_message = WheelsCmdStamped(vel_left=-self._vel_left, vel_right=-self._vel_right)
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        while not rospy.is_shutdown():
            if self._ticks_right is not None and self._ticks_left is not None:
                # start printing values when received from both encoders
                msg = f"Wheel encoder ticks [LEFT, RIGHT]: {self._ticks_left}, {self._ticks_right}"
                #print(f'wheel left, right distance {self.delta_x_left} : {self.delta_x_right}')
                if self.delta_x_left > 1.25 and self.delta_x_right > 1.25 and is_forward:
                    self._ticks_left_init = self._ticks_left

                    is_forward = False
                    self._publisher.publish(stop)
                    rate.sleep()
                if self.delta_x_left < 0 and self.delta_x_right < 0:
                    self._publisher.publish(stop)
                    break
                if is_forward:
                    self._publisher.publish(forward_message)
                else:
                    self._publisher.publish(backward_message)
                rospy.loginfo(msg)
            rate.sleep()
        pass

    """
    publish for making the bot movie
    https://docs.duckietown.com/daffy/devmanual-software/beginner/ros/wheel-control.html 
    """
    def run_publisher(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(0.1)
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
        while not rospy.is_shutdown():
            self._publisher.publish(message)
            rate.sleep()
    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # define class MoveNode
    # call the function run of class MoveNode
    # create the node
    node = MoveNode(node_name='move_node')
    # run the timer in node
    #node.drive_straight_n_back()
    node.rotate_clockwise(math.pi / 2)
    node.rotate_anticlockwise(math.pi / 2)
    # keep spinning
    rospy.spin()