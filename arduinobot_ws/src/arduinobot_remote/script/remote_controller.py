#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import Bool
from pick_task import GoalPublisher

# function that gets called every time a new message is published on the topic /jarvis_controller
def jarvis_controller_callback(data):
    pub.publish(data) 

def jarvis_messenger_callback(data):
    rospy.loginfo('Remote message received: %s', data.data)  

def jarvis_voice_callback(data):
    gp = GoalPublisher()
    if data.data:
        gp.setRobotJoint()

if __name__ == '__main__':
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('remote_controller', anonymous=True)

    # Subscribe to the topic in which the voice assistant is writing
    rospy.Subscriber("jarvis_controller", JointState, jarvis_controller_callback)
    rospy.Subscriber("jarvis_messenger", String, jarvis_messenger_callback)
    rospy.Subscriber("jarvis_voice", Bool, jarvis_voice_callback)

    # Keep the subscriber running
    rospy.spin()