#!/usr/bin/python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollowingNode:
    def __init__(self):

        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.cmd_pub = rospy.Publisher("ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=10)

        # Controller Gains
        self.Kp = 0.1 #Proportional Gain
        self.Kd = 0 #Derivative Gain

        # This is the distance from the wall you want to drive
        self.reference_cmd = 0.45

        # Desired speed you want the car to drive.
        self.speed_cmd = 1


        self.ANGLE = 40 * np.pi/180



    def laser_callback(self,msg):
        #Copy over to local variables
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        ranges = msg.ranges


        NINETY = 900
        if ranges[int(NINETY)] < self.reference_cmd:
            # TURN LEFT
            print('TURN LEFT, CURRENT DISTANCE FROM WALL IS: %f' %ranges[int(NINETY)])
            u_steer = 0.3
        elif ranges[int(NINETY)] > self.reference_cmd:
            # TURN RIGHT
            print('TURN RIGHT, DISTANCE IS: %f' %ranges[int(LOW)])
            u_steer = -0.3
        else:
            print("GO STRAIGHT")
            u_steer = 0

        #Find a way to compute the error and error rate
        error = 0
        error_rate = 0

        self.runBangController(u_steer)


    '''
    def run_position_controller(self,error,error_rate):

        p_control = self.Kp * error
        d_control = self.Kd * error_rate

        u_steer = p_control + d_control

        #You have to make sure the steering command isnt too big.
        #Saturate the control input.
        saturation_value = .3

        if u_steer > saturation_value:
            u_steer = saturation_value

        elif u_steer < -saturation_value:
            u_steer = -saturation_value

        # Print some debugging information!
        print "================================"
        print "Error:   %.2f  E_Rate:   %.2f" %(error,error_rate)
        print "P_term:  %.2f  D_term:   %.2f" %(p_control,d_control)
        print "Control: %.2f  Steering: %.2f" %(p_control+d_control,u_steer)

    '''

    def runBangController(self, u_steer):
            output_msg = AckermannDriveStamped()
            output_msg.drive.steering_angle = u_steer #This might have to be negative. Not sure.
            output_msg.drive.speed = self.speed_cmd
            output_msg.header.stamp = rospy.Time.now()
            self.cmd_pub.publish(output_msg)


if __name__ == "__main__":
    rospy.init_node("wall_following_controller")
    node = WallFollowingNode()
    rospy.spin()
