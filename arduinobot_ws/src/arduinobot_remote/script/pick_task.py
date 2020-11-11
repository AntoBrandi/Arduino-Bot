#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

class GoalPublisher:
    # Set list of possible poses of the robot
    # POSE A
    pose_a =  geometry_msgs.msg.Pose()
    
    # POSE B
    pose_b =  geometry_msgs.msg.Pose()

    # Set a list of possible joint configuration of the robot
    # JOINT A
    joint_a = [1.49921669110568, 0, -0.39521235582288017]
    joint_a_gripper = [-1, 1]

    # JOINT B
    joint_b = [1.49921669110568, -0.5978450819800801, -0.39521235582288017]
    joint_b_gripper = [-1, 1]

    # JOINT C 
    joint_c = [1.49921669110568, -0.5978450819800801, -0.39521235582288017]
    joint_c_gripper = [0, 0]

    # JOINT D 
    joint_d = [1.49921669110568, -0.5978450819800801, 0.8]
    joint_d_gripper = [0, 0]

    # JOINT D
    joint_e = [1.49921669110568, 0, 0.8]
    joint_e_gripper = [0, 0]

    # JOINT E
    joint_f = [-1.1451105222372, -0.5978450819800801, -0.07099999397135992]
    joint_f_gripper = [0, 0]

    # select which pose to apply
    counter = 0

    def __init__(self):
        # initialize the ROS interface with the robot via moveit
        moveit_commander.roscpp_initialize(sys.argv)
        #rospy.init_node('voice_command_handler', anonymous=True)

        # create a robot commander object that will be the interface with the robot
        self.robot = moveit_commander.RobotCommander()

        # create a move group commander object that will be the interface with the robot joints
        self.group_name = 'arduinobot_arm'
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        # create a move group commander object for the gripper
        self.group_name_gripper = 'arduinobot_hand'
        self.move_group_gripper = moveit_commander.MoveGroupCommander(self.group_name_gripper)
        self.group_gripper = moveit_commander.MoveGroupCommander(self.group_name_gripper)

        # create a display trajectory object that will publish the trajectory to rviz
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        # Init pose a
        self.pose_a.position.x =  -0.602333444161
        self.pose_a.position.y = -0.122612667985
        self.pose_a.position.z = 1.47456023426
        self.pose_a.orientation.x = 0.483593065819
        self.pose_a.orientation.y = -0.395069335888
        self.pose_a.orientation.z = -0.494150054591
        self.pose_a.orientation.w = 0.604874937554
        # Init pose b
        self.pose_b.position.x =  0.367931516257
        self.pose_b.position.y = 1.37062909231
        self.pose_b.position.z = 1.30534995692
        self.pose_b.orientation.x = 0.138424625668
        self.pose_b.orientation.y = -0.018256180886
        self.pose_b.orientation.z = -0.129472347056
        self.pose_b.orientation.w = 0.981703746665


    def getRobotInfo(self):
        # We can get the name of the reference frame for this robot:
        planning_frame = self.group.get_planning_frame()
        print("============ Reference frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.group.get_end_effector_link()
        print("============ End effector: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Robot Groups:", self.robot.get_group_names())

        # Current robot pose
        print("============ Printing robot pose")
        print(self.move_group.get_current_pose().pose)

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("")


    def setRobotPose(self):
        # Plan the motion of the robot to a secific pose in the base system of reference
        # The counter is a Even number
        if(self.counter%2==0):
            pose_goal = self.pose_a
        # The counter is a Odd number
        else:
            pose_goal = self.pose_b
        

        # assign and execute the pose to the robot
        self.move_group.set_pose_target(pose_goal)
        plan = self.move_group.go(wait=True)

        # once the mouvement has been completed, stop the robot
        self.move_group.stop()

        # clear the crrent target that was assigned and completed by the robot
        self.move_group.clear_pose_targets()

        # increase the counter to move into another location the next time
        self.counter += 1


    def setRobotJoint(self):
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal_gripper = self.move_group_gripper.get_current_joint_values()

        # 1
        joint_goal[:3] = self.joint_a
        joint_goal_gripper[:2] = self.joint_a_gripper
        # go to the target position in the joint space
        self.move_group_gripper.go(joint_goal_gripper, wait=True)
        self.move_group.go(joint_goal, wait=True)
        # once the mouvement has been completed, stop the robot
        self.move_group.stop()
        self.move_group_gripper.stop()

        # 2
        joint_goal[:3] = self.joint_b
        joint_goal_gripper[:2] = self.joint_b_gripper
        # go to the target position in the joint space
        self.move_group.go(joint_goal, wait=True)
        self.move_group_gripper.go(joint_goal_gripper, wait=True)
        # once the mouvement has been completed, stop the robot
        self.move_group.stop()
        self.move_group_gripper.stop()

        # 3
        joint_goal[:3] = self.joint_c
        joint_goal_gripper[:2] = self.joint_c_gripper
        # go to the target position in the joint space
        self.move_group.go(joint_goal, wait=True)
        self.move_group_gripper.go(joint_goal_gripper, wait=True)
        # once the mouvement has been completed, stop the robot
        self.move_group.stop()
        self.move_group_gripper.stop()

        # 4
        joint_goal[:3] = self.joint_d
        joint_goal_gripper[:2] = self.joint_d_gripper
        # go to the target position in the joint space
        self.move_group.go(joint_goal, wait=True)
        self.move_group_gripper.go(joint_goal_gripper, wait=True)
        # once the mouvement has been completed, stop the robot
        self.move_group.stop()
        self.move_group_gripper.stop()

        # 5
        joint_goal[:3] = self.joint_e
        joint_goal_gripper[:2] = self.joint_e_gripper
        # go to the target position in the joint space
        self.move_group.go(joint_goal, wait=True)
        self.move_group_gripper.go(joint_goal_gripper, wait=True)
        # once the mouvement has been completed, stop the robot
        self.move_group.stop()
        self.move_group_gripper.stop()

        # 6
        joint_goal[:3] = self.joint_f
        joint_goal_gripper[:2] = self.joint_f_gripper
        # go to the target position in the joint space
        self.move_group.go(joint_goal, wait=True)
        self.move_group_gripper.go(joint_goal_gripper, wait=True)
        # once the mouvement has been completed, stop the robot
        self.move_group.stop()
        self.move_group_gripper.stop()
