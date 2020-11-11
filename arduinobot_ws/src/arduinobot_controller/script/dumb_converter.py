#!/usr/bin/env python
import rospy
from control_msgs.msg import FollowJointTrajectoryActionGoal
from moveit_msgs.msg import ExecuteTrajectoryActionGoal

def callback(data):
    msg = FollowJointTrajectoryActionGoal()
    msg.header = data.header
    msg.goal_id = data.goal_id
    msg.goal.trajectory = data.goal.trajectory.joint_trajectory
    pub.publish(msg)
    
if __name__ == '__main__':
    pub = rospy.Publisher('/arduinobot_controller/goal', FollowJointTrajectoryActionGoal, queue_size=10)

    rospy.init_node('dumb_converter', anonymous=True)
    # init the subscriber
    rospy.Subscriber("/execute_trajectory/goal", ExecuteTrajectoryActionGoal, callback)
    # keep this going
    rospy.spin()