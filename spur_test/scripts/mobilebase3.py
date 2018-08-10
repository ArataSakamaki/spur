import rospy
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool


class MobileBase:
    def __init__(self):
        self.cmd_cb = rospy.Subscriner('cmd_vel', Twist, self.twist_cb)

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

        def twist_cb(self, msg):
            x = msg.linear.x
            y = msg.linear.y
            angle = msg.angler.z

            fr_p = (x - r*angle*np.sin(angle))**2 + (y + r*angle*sin(angle))**2
            fl_p = (x - r*angle*np.sin(angle))**2 + (y + r*angle*sin(angle))**2
            br_p = (x - r*angle*np.sin(angle))**2 + (y + r*angle*sin(angle))**2
            bl_p = (x - r*angle*np.sin(angle))**2 + (y + r*angle*sin(angle))**2

            fr_v = np.arctan(y+r*angle*np.cos(angle)/x-r*angle*np.sin(angle))
            fl_v = np.arctan(y+r*angle*np.cos(angle)/x-r*angle*np.sin(angle))
            br_v = np.arctan(y+r*angle*np.cos(angle)/x-r*angle*np.sin(angle))
            bl_v = np.arctan(y+r*angle*np.cos(angle)/x-r*angle*np.sin(angle))

        def pulish_to_motor(self):
            pass

class Odometory:
    def __init__(self):


        
if __name__ == '__main__':
    rospy.init_node('base_controller')
    mobilebase = MobileBase()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        mobilebase.publish_to_motor()
        rate.sleep()








        
