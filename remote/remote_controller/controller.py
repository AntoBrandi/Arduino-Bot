import roslibpy
import math

CONTROLLER_TOPIC = "jarvis_controller"
CONTROLLER_TYPE = "sensor_msgs/JointState"


def degree_to_radians(angle):
    return angle*math.pi/180


class Controller:

    def __init__(self, ros):
        self.ros = ros
        self.angles = []
        while True:
            try:
                print("Set an angle for the base (0-180)")
                base = int(input())
                if 0 <= base <= 180:
                    self.angles.append(degree_to_radians(base-90))
                else:
                    raise ValueError
                print("Set an angle for the shoulder (0-180)")
                shoulder = int(input())
                if 0 <= shoulder <= 180:
                    self.angles.append(degree_to_radians(90-shoulder))
                else:
                    raise ValueError
                print("Set an angle for the elbow (0-180)")
                elbow = int(input())
                if 0 <= elbow <= 180:
                    self.angles.append(degree_to_radians(elbow-90))
                else:
                    raise ValueError
                print("Set an angle for the gripper (0-180)")
                gripper = int(input())
                if 0 <= gripper <= 180:
                    self.angles.append(-degree_to_radians(gripper/2))
                else:
                    raise ValueError
                self.compose_msg()
            except ValueError:
                print("Invalid Input")

    def compose_msg(self):
        msg = dict(position=self.angles)
        self.angles = []
        self.publish_msg(msg)

    def publish_msg(self, msg):
        self.ros.publish(CONTROLLER_TOPIC, msg)
