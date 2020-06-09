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

    # select which pose to apply
    counter = 0

    def __init__(self):
        # initialize the ROS interface with the robot via moveit
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('voice_command_handler', anonymous=True)

        # create a robot commander object that will be the interface with the robot
        self.robot = moveit_commander.RobotCommander()

        # create a move group commander object that will be the interface with the robot joints
        self.group_name = 'arduinobot_arm'
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

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
        joint_goal[0] = -1.1655308744855999  #1.38826979362584
        joint_goal[1] = -0.4561592533027199
        joint_goal[2] = 0.050579641722960034
        print(joint_goal)

        # go to the target position in the joint space
        self.move_group.go(joint_goal, wait=True)

        # once the mouvement has been completed, stop the robot
        self.move_group.stop()
