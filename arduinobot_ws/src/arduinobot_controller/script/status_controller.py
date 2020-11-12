#!/usr/bin/env python
import rospy
from control_msgs.msg import FollowJointTrajectoryActionFeedback, GripperCommandActionFeedback
from sensor_msgs.msg import JointState

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
last_status = [0.0, 0.0, 0.0, 0.0]

def trajectory_feedback_cb(data):
    rospy.loginfo(rospy.get_caller_id() + "New arm feedback received")
    last_status[0] =  data.feedback.actual.positions[0]
    last_status[1] =  data.feedback.actual.positions[1]
    last_status[2] =  data.feedback.actual.positions[2]

def gripper_controller_cb(data):
    rospy.loginfo(rospy.get_caller_id() + "New gripper feedback received")
    last_status[3] = data.feedback.position


if __name__ == '__main__':

    pub_status = rospy.Publisher('joint_status', JointState, queue_size=10)

    rospy.init_node('status_controller', anonymous=True)
    rate = rospy.Rate(10)

    rospy.Subscriber("trajectory_controller/feedback", FollowJointTrajectoryActionFeedback, trajectory_feedback_cb)
    rospy.Subscriber("gripper_controller/feedback", GripperCommandActionFeedback, gripper_controller_cb)

    while not rospy.is_shutdown():
        status = JointState()
        status.header.stamp = rospy.Time.now()
        status.name = JOINT_NAMES
        status.position = last_status
        pub_status.publish(status)
        rate.sleep()

    
    rospy.spin()