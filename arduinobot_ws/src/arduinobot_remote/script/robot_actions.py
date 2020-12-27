from task import Task
import random


class Dance():

    def __init__(self):
        self.task = Task()

    def run(self):
        i = 0
        while i<10:
            rand_base = round(random.uniform(-1.57,1.57),2)
            rand_shoulder = round(random.uniform(-1.57,1.57),2)
            rand_elbow = round(random.uniform(-1.57,1.57),2)
            rand_gripper = round(random.uniform(-1.57,0.0),2)
            self.task.add_position([rand_base, rand_shoulder, rand_elbow, rand_gripper, -rand_gripper])
            i +=1
        self.task.set_speed(1)
        self.task.set_acceleration(1)
        self.task.execute()


class Pick():

    def __init__(self):
        self.task = Task()

    def run(self):
        self.task.add_position([1.5, 0.0, -0.4, -1.0, 1.0])
        self.task.add_position([1.5, -0.6, -0.4, -1.0, 1.0])
        self.task.add_position([1.5, -0.6, -0.4, 0.0, 0.0])
        self.task.add_position([1.5, -0.6, 0.8, 0.0, 0.0])
        self.task.add_position([1.5, 0.0, 0.8, 0.0, 0.0])
        self.task.add_position([-1.14, -0.6, -0.07, 0.0, 0.0])
        self.task.set_speed(0.5)
        self.task.set_acceleration(0.1)
        self.task.execute()


class Wake():

    def __init__(self):
        self.task = Task()

    def run(self):
        self.task.add_position([0.0,0.0,0.0,-0.7, 0.7])
        self.task.set_speed(0.7)
        self.task.set_acceleration(0.1)
        self.task.execute()


class Sleep():

    def __init__(self): 
        self.task = Task()
        
    def run(self):
        self.task.add_position([-1.57,0.0,-1.0,0.0, 0.0])
        self.task.set_speed(0.7)
        self.task.set_acceleration(0.1)
        self.task.execute()