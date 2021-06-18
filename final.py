#!/usr/bin/env python3
import can
import rospy
import os
import time
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
import math
from can_connection import CanConnection
from numpy import interp

class Pilot:

    def __init__(self):

        self.can=CanConnection()
        self.can_log=self.can.check_connection()
        if self.can_log['status']=='S':
            pass
        else:
            rospy.logwarn(self.can_log['msg'])
        self.prev_time = rospy.get_rostime()
        self.target_angle = 0
        self.steer_mapped=0
        self.mapped = 0
        self.current_angle = 0
        rospy.Subscriber('/cmd_vel',Twist,self.callback)
        rospy.Subscriber('/steer_angle', Float32, self.callback2)
        self.a = 0
        self.steering_max = rospy.get_param('/hardware/steering/max_angle',136)
        self.steering_min = rospy.get_param('/hardware/steering/min_angle',60)
        self.center_angle = rospy.get_param('/hardware/steering/center', 101)
    def callback(self,data):

        self.throttle = data.linear.x
        self.target_angle = data.angular.z
        print(self.throttle,self.target_angle)
        self.mapped=int(interp(abs(self.throttle), [0,1], [0,3000]))
        self.steer_mapped=int(interp(abs(self.target_angle), [-1,1], [-30,30]))
        if self.target_angle>0:
            self.steer_mapped = 101-(self.steer_mapped)
        elif self.target_angle<0:
            self.steer_mapped = 101+(self.steer_mapped)
        self.prev_time = rospy.get_rostime()
        print(self.steer_mapped)

    def callback2(self,data):
        self.current_angle = data.data
        

    def time_diff(self):

        now_time = rospy.get_rostime()

        angle_time_diff = (now_time - self.prev_time).to_nsec()/900000000

        if angle_time_diff > 1:

            print("Time out!")
            self.logger(self.can.trottle_send(0))
            self.logger(self.can.steering_send(0))
        else:
            self.logger(self.can.trottle_send(self.mapped))

        

        self.angle_diff = (self.target_angle - self.current_angle)

        while(abs(self.angle_diff)>10):

            self.angle_diff = (self.target_angle - self.current_angle)

            if self.angle_diff >= 0 :
                self.a = self.current_angle +10
                if self.a > 180:
                    self.a = 180
                elif self.a < 60:
                    self.a = 60
                self.logger(self.can.steering_send(self.a))

            elif self.angle_diff <= 0:
                self.a = self.current_angle - 10
                if self.a > 180:
                    self.a = 180
                elif self.a < 60:
                    self.a = 60
                self.logger(self.can.steering_send(self.a))


    def logger(self,logs):
        if logs['status']!='S':
            rospy.logwarn(logs['msg'])
        else:
            pass

    




if __name__ == '__main__':

    rospy.init_node('Pilot')

    a = Pilot()
    a.time_diff()
    rospy.spin()
