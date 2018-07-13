#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
import tf
from spur_test.msg import ControllerMsg
from math import atan2, degrees

class MobileBase:
    def __init__(self):
        self.cmd_cb = rospy.Subscriber('cmd_vel', Twist, self.twist_cb)
        self.speed_trigger_cb = rospy.Subscriber('speed_trigger', Bool, self.speed_cb)

        self.pub_br_r = rospy.Publisher("br_rotation_joint_position_controller/command",
                                        Float64, queue_size=1)
        self.pub_bl_r = rospy.Publisher("bl_rotation_joint_position_controller/command",
                                        Float64, queue_size=1)
        self.pub_fr_r = rospy.Publisher("fr_rotation_joint_position_controller/command",
                                        Float64, queue_size=1)
        self.pub_fl_r = rospy.Publisher("fl_rotation_joint_position_controller/command",
                                        Float64, queue_size=1)
        self.pub_br_w = rospy.Publisher("br_wheel_joint_velocity_controller/command",
                                        Float64, queue_size=1)
        self.pub_bl_w = rospy.Publisher("bl_wheel_joint_velocity_controller/command",
                                        Float64, queue_size=1)
        self.pub_fr_w = rospy.Publisher("fr_wheel_joint_velocity_controller/command",
                                        Float64, queue_size=1)
        self.pub_fl_w = rospy.Publisher("fl_wheel_joint_velocity_controller/command",
                                        Float64, queue_size=1)
        self.horizontal_degree = 0
        self.br_horizontal_degree = 0
        self.bl_horizontal_degree = 0
        self.fr_horizontal_degree = 0
        self.fl_horizontal_degree = 0
        
        self.fb_r = None
        self.fb_l = None
        print "init_node"
        
    def twist_cb(self, msg):
        horizontal_degree = -atan2(msg.linear.y, msg.linear.x)
        self.angular_degree = msg.angular.z
        print horizontal_degree
        #print abs(horizontal_degree) , abs(self.angular_degree)
        if abs(horizontal_degree) != 0 or abs(self.angular_degree) != 0:
            #print self.horizontal_degree
            if horizontal_degree > 1.57:
                print 'back naname 1.57'
                self.br_horizontal_degree = horizontal_degree - 3.1415
                self.bl_horizontal_degree = horizontal_degree - 3.1415
                self.fr_horizontal_degree = horizontal_degree - 3.1415
                self.fl_horizontal_degree = horizontal_degree - 3.1415
                if self.angular_degree < 1.57 and self.angular_degree > -1.57: #fron
                    self.fr_horizontal_degree=horizontal_degree+self.angular_degree
                    self.fl_horizontal_degree=horizontal_degree+self.angular_degree
                elif angular_degree>1.57 and angular_degree<-1.57: #back
                    self.fr_horizontal_degree=horizontal_degree+self.angular_degree
                    self.fl_horizontal_degree=horizontal_degree+self.angular_degree
                    
                self.fb_l = -2
                self.fb_r = 2

            elif horizontal_degree < -1.57:
                print 'back naname'
                self.br_horizontal_degree = horizontal_degree + 3.1415
                self.bl_horizontal_degree = horizontal_degree + 3.1415
                self.fr_horizontal_degree = horizontal_degree + 3.1415
                self.fl_horizontal_degree = horizontal_degree + 3.1415
                if self.angular_degree < 1.57 and self.angular_degree > -1.57: #fron
                    self.fr_horizontal_degree=horizontal_degree+self.angular_degree
                    self.fl_horizontal_degree=horizontal_degree+self.angular_degree
                elif angular_degree>1.57 and angular_degree<-1.57: #back
                    self.fr_horizontal_degree=horizontal_degree+self.angular_degree
                    self.fl_horizontal_degree=horizontal_degree+self.angular_degree
            
                self.fb_l = -2
                self.fb_r = 2
                

#            elif horizontal_degree >= -1.57 and horizontal_degree <= 1.57: #ps3 controller stick forward
#                print 'forward'
#                self.br_horizontal_degree = horizontal_degree
#                self.bl_horizontal_degree = horizontal_degree
#                self.fr_horizontal_degree = horizontal_degree
#                self.fl_horizontal_degree = horizontal_degree
#                if self.angular_degree < 1.57 and self.angular_degree > -1.57: #fron
#                    self.fr_horizontal_degree=horizontal_degree+self.angular_degree
#                    self.fl_horizontal_degree=horizontal_degree+self.angular_degree
#                elif angular_degree > 1.57 and angular_degree < -1.57: #back
#                    self.fr_horizontal_degree=horizontal_degree+self.angular_degree
#                    self.fl_horizontal_degree=horizontal_degree+self.angular_degree

#                self.fb_l = -2
#                self.fb_r = -2

            
            elif abs(horizontal_degree) == 0 and abs(angular_degree) != 0: #only rotation
            #else:
                print 'only rotation'
                if self.angular_degree > 0.1:
                    self.br_horizontal_degree = 0.78
                    self.bl_horizontal_degree = -0.78
                    self.fr_horizontal_degree = 0.78
                    self.fl_horizontal_degree = -0.78
                
                    self.fb_l = 2
                    self.fb_r = -2
                else:
                    self.br_horizontal_degree = -0.78
                    self.bl_horizontal_degree = 0.78
                    self.fr_horizontal_degree = -0.78
                    self.fl_horizontal_degree = 0.78

                    self.fb_l = -2
                    self.fb_r = 2

            else: #ps3 controller stick forward
                print 'forward'
                self.br_horizontal_degree = horizontal_degree
                self.bl_horizontal_degree = horizontal_degree
                self.fr_horizontal_degree = horizontal_degree
                self.fl_horizontal_degree = horizontal_degree
                if self.angular_degree < 1.57 and self.angular_degree > -1.57: #fron
                    self.fr_horizontal_degree=horizontal_degree+self.angular_degree
                    self.fl_horizontal_degree=horizontal_degree+self.angular_degree
                elif angular_degree > 1.57 and angular_degree < -1.57: #back
                    self.fr_horizontal_degree=horizontal_degree+self.angular_degree
                    self.fl_horizontal_degree=horizontal_degree+self.angular_degree

                self.fb_l = -2
                self.fb_r = -2

        else:  #stop
            print "stop"
            self.fb_l = 0
            self.fb_r = 0
            
    def speed_cb(self, msg):
        print 'speed'
        if self.fb_l < 0 and self.fb_r > 0:
            self.fb_l = self.fb_l - 3
            self.fb_r = self.fb_r + 3
        else:
            self.fb_l = self.fb_l + 3
            self.fb_r = self.fb_r - 3
        
    def publish_to_motor(self):
        #print "publish motor"
        self.pub_bl_w.publish(self.fb_l)
        self.pub_br_w.publish(self.fb_r)
        self.pub_fl_w.publish(self.fb_l)
        self.pub_fr_w.publish(self.fb_r)
        
        self.pub_br_r.publish(self.br_horizontal_degree)
        self.pub_bl_r.publish(self.bl_horizontal_degree)
        self.pub_fr_r.publish(self.fr_horizontal_degree)
        self.pub_fl_r.publish(self.fl_horizontal_degree)
        
        #print 'publish_message'    

if __name__ == "__main__":
    rospy.init_node('base_controller')
    mobilebase = MobileBase()
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        mobilebase.publish_to_motor()
        rate.sleep()
    rospy.spin()
    
