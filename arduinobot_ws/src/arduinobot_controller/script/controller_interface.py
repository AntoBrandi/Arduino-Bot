#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt8MultiArray

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
    joint_1 = int(((anglesRad[0]+1.57075)*180)/3.1415)
    joint_2 = 180-int(((anglesRad[1]+1.57075)*180)/3.1415)
    joint_3 = int(((anglesRad[2]+1.57075)*180)/3.1415)
    joint_4 = int(((-anglesRad[3])*180)/1.57075)
    anglesPerc = [joint_1, joint_2, joint_3, joint_4]
    print "Angles %s ", anglesPerc
    publish(anglesPerc)
    
if __name__ == '__main__':
    pub = rospy.Publisher('servo_actuate', UInt8MultiArray, queue_size=10)

    rospy.init_node('controller_interface', anonymous=True)
    # init the subscriber
    rospy.Subscriber("/joint_states", JointState, callback)
    # keep this going
    rospy.spin()