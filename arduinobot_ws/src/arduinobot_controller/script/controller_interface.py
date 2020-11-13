#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16MultiArray

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4']

def convert_angles(angles_deg):
    angles_rad = []
    angles_rad.append(((math.pi*angles_deg[0]) - ((math.pi/2)*180))/180)
    angles_rad.append((((180-angles_deg[1])*math.pi)-((math.pi/2)*180))/180)
    angles_rad.append(((math.pi*angles_deg[2]) - ((math.pi/2)*180))/180)
    angles_rad.append(-((math.pi/2)*angles_deg[3])/180)
    return angles_rad


def joint_states_cb(data):
    status = JointState()
    status.header.stamp = rospy.Time.now()
    status.name = JOINT_NAMES
    status.position = convert_angles(data.data)
    pub_status.publish(status)


if __name__ == '__main__':

    pub_status = rospy.Publisher('joint_states', JointState, queue_size=10)

    rospy.init_node('controller_interface', anonymous=True)

    rospy.Subscriber("arduino/joint_states", UInt16MultiArray, joint_states_cb)
    
    rospy.spin()