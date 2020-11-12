#! /usr/bin/env python
import rospy
import actionlib
import control_msgs.msg
from std_msgs.msg import UInt8MultiArray

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3']


class TrajectoryControllerAction(object):
    # create messages that are used to publish feedback/result
    _feedback = control_msgs.msg.FollowJointTrajectoryFeedback()
    _result = control_msgs.msg.FollowJointTrajectoryResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, control_msgs.msg.FollowJointTrajectoryAction, execute_cb=self.goal_cb, auto_start = False)
        self._as.start()
        self.old_joint_angles = [0.0,0.0,0.0]
        self.execute(self.old_joint_angles, JOINT_NAMES)
      
    # This function is called when the action server receives a new goal
    def goal_cb(self, goal):
        success = True
        rospy.loginfo('%s: Goal Received' % self._action_name)
        joint_names = goal.trajectory.joint_names
        
        for i, point in enumerate(goal.trajectory.points):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            delay = point.time_from_start - goal.trajectory.points[i-1].time_from_start if i>0 else 0
            rospy.sleep(delay)
            self.execute(point.positions, joint_names)

            self._feedback.joint_names = JOINT_NAMES
            self._feedback.actual = point
            self._feedback.desired = point
            self._as.publish_feedback(self._feedback)

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)


    # This function receives an array of joint angles in radians and
    # convert those in degrees in order to be appliend to the servo motors
    # by the Arduino controller
    def convert_angles(self, angles, joint_names):
        # conversion between radiants and percentage
        # output of this is a number between 0 and 180
        current_angles = []
        for i, joint_name in enumerate(JOINT_NAMES):
            if joint_name in joint_names:

                if i == 0:
                    current_angles.append(self.convert_base_angle(angles[i]))
                elif i == 1:
                    current_angles.append(self.convert_shoulder_angle(angles[i]))
                else:
                    current_angles.append(self.convert_elbow_angle(angles[i]))

                self.old_joint_angles[i] = current_angles[i]
            else:
                current_angles.append(self.old_joint_angles[i])

        return current_angles           
        

    def execute(self, angles, joint_names):
        rospy.loginfo('Angles Radians : %s' % str(angles))
        angles_deg = self.convert_angles(angles, joint_names)
        rospy.loginfo('Angles Degrees : %s' % str(angles_deg))
        pub.publish(data=angles_deg)
    
    def convert_base_angle(self, angle_rad):
        return int(((angle_rad+1.57075)*180)/3.1415)
    
    def convert_shoulder_angle(self, angle_rad):
        return 180-int(((angle_rad+1.57075)*180)/3.1415)

    def convert_elbow_angle(self, angle_rad):
        return int(((angle_rad+1.57075)*180)/3.1415)
        

if __name__ == '__main__':
    # Publish the converted joint angles to the arduino subscriber node
    pub = rospy.Publisher('arduino/arm_actuate', UInt8MultiArray, queue_size=10)

    rospy.init_node('trajectory_controller')

    # Init the FollowJointTrajectory action server that will receive a trajectory for each joint and will
    # execute it in the real robot
    server = TrajectoryControllerAction(rospy.get_name())
    rospy.spin()
