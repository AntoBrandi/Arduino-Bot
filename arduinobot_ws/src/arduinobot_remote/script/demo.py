#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
import tf

ARM_GROUP_NAME = 'arduinobot_arm'
GRIPPER_GROUP_NAME = 'arduinobot_hand'

class MoveGroupPythonIntefaceTutorial(object):

  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)


    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()
    self.group = moveit_commander.MoveGroupCommander(ARM_GROUP_NAME)
    self.gripper_group = moveit_commander.MoveGroupCommander(GRIPPER_GROUP_NAME)
    self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    self.planning_frame = self.group.get_planning_frame()
    self.eef_link = self.group.get_end_effector_link()
    self.group_names = self.robot.get_group_names()
    self.box_name = "box"


  def go_to_joint_state(self, joint_goal, gripper_goal):
    self.gripper_group.go(gripper_goal, wait=True)
    self.group.go(joint_goal, wait=True)
    
    self.group.stop()


  def display_trajectory(self, plan):

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    self.display_trajectory_publisher.publish(display_trajectory)


  def execute_plan(self, plan):
    self.group.execute(plan, wait=True)


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = self.scene.get_attached_objects([self.box_name])
      is_attached = len(attached_objects.keys()) > 0
      is_known = self.box_name in self.scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      rospy.sleep(0.1)
      seconds = rospy.get_time()

    return False


  def add_box(self, timeout=4):

    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.position.x = 0.9
    box_pose.pose.position.y = 0.16
    box_pose.pose.position.z = 0.7
    roll = -0.3
    pitch = 0.0
    yaw = 1.57
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    box_pose.pose.orientation.x = quaternion[0]
    box_pose.pose.orientation.y = quaternion[1]
    box_pose.pose.orientation.z = quaternion[2]
    box_pose.pose.orientation.w = quaternion[3]
    self.scene.add_cylinder(self.box_name, box_pose, height=1, radius=0.03)

    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
    grasping_group = 'arduinobot_hand'
    touch_links = self.robot.get_link_names(group=grasping_group)
    aco = AttachedCollisionObject()
    co = CollisionObject()
    co.id = self.box_name
    aco.object = co
    aco.touch_links = touch_links
    aco.link_name = self.eef_link
    self.scene.attach_object(aco)

    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, timeout=4):
    self.scene.remove_attached_object(self.eef_link, name=self.box_name)

    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_box(self, timeout=4):
    self.scene.remove_world_object(self.box_name)

    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


if __name__ == '__main__':

    tutorial = MoveGroupPythonIntefaceTutorial()
    # start from home position
    rospy.loginfo("Going to the home pose")
    tutorial.go_to_joint_state([0.0,0.0,0.0], [-0.0, 0.0])
    rospy.loginfo("Displaying the box to grasp")
    tutorial.add_box()

    # go grab the box
    rospy.loginfo("Going to grasp the object")
    # pose 1 - close to the object and open the gripper
    tutorial.go_to_joint_state([-1.560293703551582, 0.8881149810325795, -0.8972658813469394], [ -0.6421534804488358,  0.6421534804488358])
    # pose 2 - close the gripper
    tutorial.go_to_joint_state([-1.560293703551582, 0.8881149810325795, -0.8972658813469394], [ -0.5,  0.5])

    rospy.loginfo("Grasping")
    tutorial.attach_box()

    # pose 3 - close the gripper with the object
    tutorial.go_to_joint_state([-1.560293703551582, 0.8881149810325795, -0.8972658813469394], [ -0.05,  0.05])

    # pose 4 - bring the object to the home
    rospy.loginfo("Bringing the object to me")
    tutorial.go_to_joint_state([0.0,0.0,0.0], [-0.05, 0.05])

    # pose 5 - open the gripper and deliver the object
    rospy.loginfo("Object delivered")
    tutorial.go_to_joint_state([0.0,0.0,0.0], [-0.1, 0.1])
    tutorial.detach_box()
    tutorial.go_to_joint_state([0.0,0.0,0.0], [ -0.6421534804488358,  0.6421534804488358])
    tutorial.detach_box()
    #tutorial.remove_box()
