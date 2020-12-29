#!/usr/bin/env python
from flask import Flask, render_template
from flask_ask import Ask, statement, question, session
from arduinobot_remote.msg import ArduinobotTaskAction, ArduinobotTaskGoal
import rospy
import threading
import actionlib


"""
  arduinobot - alexa_interface

  This script implements a flask web server.
  The web server exposes APIs to the Amazon Alexa assistant
  in order to control the robot with the voice

  Copyright (c) 2020 Antonio Brandi.  All right reserved.
"""


threading.Thread(target=lambda: rospy.init_node('alexa_interface', disable_signals=True)).start()
client = actionlib.SimpleActionClient('task_server', ArduinobotTaskAction)

app = Flask(__name__)
ask = Ask(app, "/")


@ask.launch
def launch():
    # Function that gets called when the skill is activated
    goal = ArduinobotTaskGoal(task_number=0)
    client.send_goal(goal)
    launch_msg = render_template('online')
    return question(launch_msg)


@ask.intent("DanceIntent")
def dance():
    # Function that is called when the Dance Intent is activated
    client.wait_for_server()
    goal = ArduinobotTaskGoal(task_number=1)
    client.send_goal(goal)
    dance_msg = render_template('dance')
    return statement(dance_msg)


@ask.intent("PickIntent")
def pick():
    # Function that is called when the Pick Intent is activated
    client.wait_for_server()
    goal = ArduinobotTaskGoal(task_number=2)
    client.send_goal(goal)
    pick_msg = render_template('pick')
    return statement(pick_msg)


@ask.intent("SleepIntent")
def sleep():
    # Function that is called when the Sleep Intent is activated
    client.wait_for_server()
    goal = ArduinobotTaskGoal(task_number=3)
    client.send_goal(goal)
    sleep_msg = render_template('sleep')
    return statement(sleep_msg)


@ask.intent("WakeIntent")
def sleep():
    # Function that is called when the Wake Intent is activated
    client.wait_for_server()
    goal = ArduinobotTaskGoal(task_number=0)
    client.send_goal(goal)
    wake_msg = render_template('wake')
    return statement(wake_msg)


@ask.intent("AMAZON.FallbackIntent")
def fallback():
    fallback_msg = render_template('fallback')
    return statement(fallback_msg)


if __name__ == '__main__':
    app.run()