#!/usr/bin/env python
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
         self.kpx = 0
         self.kix = 0
         self.kdx = 0

         self.kpy = 0
         self.kiy = 0
         self.kdy = 0

         self.kpphi = 0
         self.kiphi = 0
         self.kdphi = 0

         self.kptheta = 0
         self.kitheta = 0
         self.kdtheta = 0

         self.kppsi = 0
         self.kipdi = 0
         self.kdpsi = 0
         print('Controller gains defined')


         # Specifying the schematic of the sero motors for the tilt-rotor drone and initialization
         GPIO.setmode(GPIO.BOARD)
         self.tilt_1_pin = 11
         self.tilt_2_pin = 13
         self.tilt_3_pin = 15
         self.tilt_4_pin = 16

         GPIO.setup(tilt_1_pin,GPIO.OUT)
         GPIO.setup(tilt_2_pin,GPIO.OUT)
         GPIO.setup(tilt_3_pin,GPIO.OUT)
         GPIO.setup(tilt_4_pin,GPIO.OUT)

         self.pwm1 = GPIO.PWM(tilt_1_pin,50)
         self.pwm2 = GPIO.PWM(tilt_2_pin,50)
         self.pwm3 = GPIO.PWM(tilt_3_pin,50)
         self.pwm4 = GPIO.PWM(tilt_4_pin,50)

         print('PWM signal pins configured')
         # Initilize the states
         self.q0 = 1
         self.q1 = 0
         self.q2 = 0
         self.q3 = 0

         self.q0_des = 1
         self.q1_des = 0
         self.q2_des = 0
         self.q3_des = 0

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
        self.x = q1
        self.y = q2
        self.z = q3
        self.w = q0

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

    def tilt_control(self):
        e_x = self.x_des - self.x_pos
        e_y = self.y_des - self.y_pos

        print('Error in x-direction %f', %e_x)
        print('Error in y-direction %f', %e_y)




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
    rospy.init_node('tilt_servo_control')
    uav_data = Get_UAV_Data()

    rate = rospy.rate(50) # 10hz


    while not rospy.is_shutdown():
        uav_data.tilt_control()

        #print('Executing main while loop')
        rospy.sleep(1)


#     while not rospy.is_shutdown():
#         #initial kalman filtering
#         # uav_data.CV_Prediction(uav_data.dt, False)
#         # uav_data.CV_Update()
#
#         uav_data.IMM_Mixing()
#
#         #Kalman Filters
#         uav_data.CV_Prediction()
#     #    uav_data.CV_Update()
#         uav_data.CA_Prediction()
# #        uav_data.CA_Update()
#         uav_data.CT_Prediction()
# #        uav_data.CT_Update()
#         uav_data.Model_Prob_Update()
#
#         uav_data.State_Estimator_and_Combiner()



        #############################################################
