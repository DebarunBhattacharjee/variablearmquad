#Required Libraries

import rospy
import time
import numpy as np
import math
import RPi.GPIO as GPIO
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

class Get_UAV_Data(object):  #Just to get UAV data

    def __init__(self):
        self.has_data = False
         # Target attitude
        self.set_target_attitude_subscriber=rospy.Subscriber('mavros/setpoint_raw/target_attitude', AttitudeTarget, self.callback_target_attitude)

         # Target position
        self.set_target_position_subscriber=rospy.Subscriber('mavros/setpoint_raw/target_local', PositionTarget, self.callback_target_position)

         # Position and attitude feedback
        self.position_attitude_feedback_subscriber=rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.callback_position_attitude)

         # Linear and angular velocities feedback in body frame
        self.velocities_feedback_subscriber=rospy.Subscriber('mavros/local_position/velocity_body', TwistStamped, self.callback_velocities)

         # Controller gains

        self.kpx = 2
        self.kix = 0
        self.kdx = 0.001

        self.kpy = 2
        self.kiy = 0
        self.kdy = 0.001

        self.kpphi = 2
        self.kiphi = 0
        self.kdphi = 0.1

        self.kptheta = 2
        self.kitheta = 0
        self.kdtheta = 0.1

        self.kppsi = 0
        self.kipdi = 0
        self.kdpsi = 0
        # print('Controller gains defined')

         # Specifying the schematic of the sero motors for the sliding arms drone and initialization
         GPIO.setmode(GPIO.BOARD)
         self.slide_1_pin = 11
         self.slide_2_pin = 13

         GPIO.setup(self.slide_1_pin,GPIO.OUT)
         GPIO.setup(self.slide_2_pin,GPIO.OUT)

         self.pwm1 = GPIO.PWM(self.slide_1_pin,50)
         self.pwm2 = GPIO.PWM(self.slide_2_pin,50)

         self.pwm1.start(7)
         self.pwm2.start(7)

         # print('PWM signal pins configured')
         # Initilize the states
         self.q0 = 1
         self.q1 = 0
         self.q2 = 0
         self.q3 = 0

         self.phi = 0
         self.theta = 0
         self.psi = 0

         self.q0_des = 1
         self.q1_des = 0
         self.q2_des = 0
         self.q3_des = 0

         self.phi_des = 0
         self.theta_des = 0
         self.psi_des = 0

         self.x_pos = 0
         self.y_pos = 0

         self.x_des = 0
         self.y_des = 0

         self.x_vel = 0
         self.y_vel = 0
         self.z_vel = 0
         self.p_rt = 0
         self.q_rt = 0
         self.r_rt = 0



    def euler_from_quat(self,q1,q2,q3,q0):

        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        x = q1
        y = q2
        z = q3
        w = q0

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z # in radians


    # Callback function for target attitude
    def callback_target_attitude(self, msg):
        self.has_data = True
        self.q1_des = msg.orientation.x
        self.q2_des = msg.orientation.y
        self.q3_des = msg.orientation.z
        self.q0_des = msg.orientation.w
        # print('I received target attitude')



    # Callback function for target position
    def callback_target_position(self, msg):
        self.has_data = True
        self.x_des = msg.position.x
        self.y_des = msg.position.y
        self.z_des = msg.position.z
        # print('I received target position')



    # Callback function for position and attitude feedback
    def callback_position_attitude(self, msg):
        self.has_data = True
        self.x_pos = msg.pose.position.x
        self.y_pos = msg.pose.position.y
        self.z_pos = msg.pose.position.z
        self.q1 = msg.pose.orientation.x
        self.q2 = msg.pose.orientation.y
        self.q3 = msg.pose.orientation.z
        self.q0 = msg.pose.orientation.w
        # print('I received position and attitude feedback')

    # Callback function for linear and angular velocities feedback
    def callback_velocities(self,msg):
        self.has_data = True
        self.x_vel = msg.twist.linear.x
        self.y_vel = msg.twist.linear.y
        self.z_vel = msg.twist.linear.z
        self.p_rt = msg.twist.angular.x
        self.q_rt = msg.twist.angular.y
        self.r_rt = msg.twist.angular.z
        # print('I received linear and angular velocities')

    def slide_control(self):

        # Position Control using slide
        # Position difference
        e_x = self.x_des - self.x_pos
        e_y = self.y_des - self.y_pos

        del_x_slide = self.kpx*e_x - self.kdx*self.x_vel
        del_y_slide = self.kpx*e_y - self.kdx*self.y_vel

        # Attitude control using slide
        self.phi_des, self.theta_des, self.psi_des = self.euler_from_quat(self.q1_des, self.q2_des,self.q3_des,self.q0_des)
        self.phi, self.theta, self.psi = self.euler_from_quat(self.q1, self.q2,self.q3,self.q0)

        e_phi = self.phi_des - self.phi
        e_theta = self.theta_des - self.theta
        e_psi = self.psi_des - self.psi

        del_phi_slide = self.kpphi*e_phi - self.kdphi*self.p_rt
        del_theta_slide = self.kptheta*e_theta - self.kdtheta*self.q_rt
        del_psi_slide = self.kppsi*e_psi - self.kdpsi*self.r_rt

        # print('del_x_slide: ', del_x_slide)
        # print('del_y_slide: ', del_y_slide)
        # print('del_phi_slide: ', del_phi_slide)
        # print('del_theta_slide: ', del_theta_slide)
        # print('del_psi_slide: ', del_psi_slide)

        del_slide_1 = - del_y_slide - del_psi_slide + del_theta_slide
        del_slide_2 = del_x_slide - del_psi_slide + del_phi_slide

        # dc_1 = (1.0/15.0)*del_slide_1 + 7
        # dc_2 = (1.0/15.0)*del_slide_2 + 7
        # dc_3 = (1.0/15.0)*del_slide_3 + 7
        # dc_4 = (1.0/15.0)*del_slide_4 + 7

        dc_1 = 0.2*del_slide_1 + 7  #y=mx+c where c is the middle position of the servo
        dc_2 = 0.2*del_slide_2 + 7  #at the moment, it is 7.

        #KEEP CONSTRAINTS LOG UPDATED
        #Log
        #5<dc<9(prev)
        #6<dc<8 (current)
        # constraint for servo1
        if dc_1 < 6:
            dc_1 = 6
        if dc_1 > 8:
            dc_1 = 8
        #constraints for servo2
        if dc_2 < 6:
            dc_2 = 6
        if dc_2 > 8:
            dc_2 = 8
        #end of constraints

        print('dc_1: ', dc_1)
        print('dc_2: ', dc_2)

        self.pwm1.ChangeDutyCycle(dc_1)
        self.pwm2.ChangeDutyCycle(dc_2)

        # print('Error in x-direction: ', e_x)
        # print('Error in y-direction: ', e_y)

# ############### Publisher Functions #####################
#
# def uav_ten(states_ten):
#     pub = rospy.Publisher('uav_ten1_IMM', latlonalt, queue_size=10)
#     #rospy.init_node('UAV0_1_Dis_Node', anonymous=True)
#     #rate = rospy.#rate(10) # 10hz
#     #while not rospy.is_shutdown():
#     ten_message=latlonalt()
#     ten_message.latitude=states_ten[0]
#     ten_message.longitude=states_ten[2]
#     ten_message.altitude=states_ten[4]
#     #rospy.loginfo(thirty_message)
#     pub.publish(ten_message)
#     #rate.sleep()

if __name__=='__main__':
    rospy.init_node('slide_servo_control')
    uav_data = Get_UAV_Data()

    rate = rospy.Rate(50) # 10hz

    while not rospy.is_shutdown():

        uav_data.slide_control()
        #main slide control loop
        rospy.sleep(1)

        #############################################################
