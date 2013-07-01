#!/usr/bin/env python
import roslib; roslib.load_manifest('meka_arm_navigation_tutorials')
import rospy
from std_msgs.msg import Float64

def jnt_pos_cmd():
    pub0 = rospy.Publisher('/m3joint_ma10_j0_position_controller/command', Float64)
    pub1 = rospy.Publisher('/m3joint_ma10_j1_position_controller/command', Float64)
    pub2 = rospy.Publisher('/m3joint_ma10_j2_position_controller/command', Float64)
    pub3 = rospy.Publisher('/m3joint_ma10_j3_position_controller/command', Float64)
    pub4 = rospy.Publisher('/m3joint_ma10_j4_position_controller/command', Float64)
    pub5 = rospy.Publisher('/m3joint_ma10_j5_position_controller/command', Float64)
    pub6 = rospy.Publisher('/m3joint_ma10_j6_position_controller/command', Float64)
    
    rospy.init_node('jnt_pos_cmd')
    
    while not rospy.is_shutdown():        
        pub0.publish(Float64(0.5))
        pub1.publish(Float64(0.5))
        pub2.publish(Float64(0.5))
        pub3.publish(Float64(0.5))
        pub4.publish(Float64(0.5))
        pub5.publish(Float64(0.5))
        pub6.publish(Float64(0.5))
        rospy.sleep(1.0)
        

if __name__ == '__main__':
    try:
        jnt_pos_cmd()
    except rospy.ROSInterruptException: pass