#!/usr/bin/env python

from time import sleep
import numpy as np
import rospy
from std_msgs.msg import Float32


currentPosition = 0.5
pub = None

def moveServo_cb(data):
    global currentPosition, pub
    targetPosition = data.data
    r = targetPosition - currentPosition
    angles = np.array( (range(190)) [0::10]) - 90
    m = ( np.sin( angles * np.pi/ 180. ) + 1 ) /2
    for mi in np.nditer(m):
        pos = currentPosition +mi*r
        print "pos: ", pos
        pub.publish(pos)
        sleep(0.05)
    currentPosition = targetPosition
    print "pos-e: ", currentPosition
    pub.publish(currentPosition)

def listener():
    global pub
    rospy.init_node('servoencoder', anonymous=True)
    rospy.Subscriber('/head/tilt/smooth', Float32, moveServo_cb)
    pub = rospy.Publisher('/head/tilt', Float32, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listener()