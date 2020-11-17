#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16MultiArray

# function that publishes a message on the topic ,on which the 
# arduino is listening
def publish(msg):  
    pub.publish(data=msg)

# a new message has been published on the topic /joint_states
# this message is a sensor_msg/JointStatmessage type
# get the data and publish those on the connected arduino as joint angles
def callback(data):
    anglesRad = data.position
    # conversion between radiants and percentage
    # output of this is a number between 0 and 180
    joint_1 = int(((anglesRad[0]+(math.pi/2))*180)/math.pi)
    joint_2 = 180-int(((anglesRad[1]+(math.pi/2))*180)/math.pi)
    joint_3 = int(((anglesRad[2]+(math.pi/2))*180)/math.pi)
    joint_4 = int(((-anglesRad[3])*180)/(math.pi/2))
    anglesPerc = [joint_1, joint_2, joint_3, joint_4]
    print "Angles %s ", anglesPerc
    publish(anglesPerc)
    
if __name__ == '__main__':
    pub = rospy.Publisher('servo_actuate', UInt16MultiArray, queue_size=10)

    rospy.init_node('controller_interface', anonymous=True)
    # init the subscriber
    rospy.Subscriber("/joint_states", JointState, callback)
    # keep this going
    rospy.spin()