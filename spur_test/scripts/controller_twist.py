#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import Joy
import tf
from math import sin, cos, tan, atan2, degrees


'''
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
'''

class Teleop:
    def __init__(self):
        self.controller_cb = rospy.Subscriber('joy', Joy, self.cont_cb)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.bool_pub = rospy.Publisher('speed_trigger', Bool, queue_size=1)
        self.pub_msg = Twist()
        self.speedmsg = Bool()
    def cont_cb(self, msg):
        trigger = msg.buttons[10]
        if trigger == 1:
            leftstick_y  = 0 if msg.axes[0]==-0 else msg.axes[0] #y
            leftstick_x  = 0 if msg.axes[1]==-0 else msg.axes[1] #x
            rightstick_y = 0 if msg.axes[2]==-0 else msg.axes[2] #y
            rightstick_x = 0 if msg.axes[3]==-0 else msg.axes[3] #x

            print leftstick_x, leftstick_y, rightstick_x, rightstick_y

            self.pub_msg.linear.x = leftstick_x
            self.pub_msg.linear.y = leftstick_y
            
            self.pub_msg.angular.z = atan2(rightstick_y, rightstick_x)

            self.pub.publish(self.pub_msg)
            if msg.buttons[8] ==1:
                self.speedmsg.data = True
                self.bool_pub.publish(self.speedmsg)
  
if __name__ == "__main__":
    rospy.init_node('controller_to_twist')
    Teleop()
    rospy.spin()
