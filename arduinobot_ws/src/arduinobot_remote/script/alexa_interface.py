#!/usr/bin/env python
import logging
from flask import Flask, render_template
from flask_ask import Ask, statement, question, session
import rospy
import threading
from std_msgs.msg import String
from robot_actions import Dance, Pick, Wake, Sleep


"""
  arduinobot - alexa_interface

  This script implements a flask web server.
  The web server exposes APIs to the Amazon Alexa assistant
  in order to control the robot with the voice

  Copyright (c) 2020 Antonio Brandi.  All right reserved.
"""


threading.Thread(target=lambda: rospy.init_node('alexa_interface', disable_signals=True, anonymous=True)).start()

app = Flask(__name__)
ask = Ask(app, "/")
logging.getLogger("flask_ask").setLevel(logging.DEBUG)


@ask.launch
def launch():
    # Function that gets called when the skill is activated
    if not rospy.is_shutdown(): 
        launch_msg = render_template('online')
        wake = Wake()
        threading.Thread(target = wake.run).start()
        return question(launch_msg)
    else:
        return question(render_template('offline'))
    

@ask.intent("DanceIntent")
def dance():
    # Function that is called when the Dance Intent is activated
    if not rospy.is_shutdown(): 
        dance_msg = render_template('dance')
        dance = Dance()
        threading.Thread(target = dance.run).start()
        return statement(dance_msg)
    else:
        return question(render_template('offline'))


@ask.intent("PickIntent")
def pick():
    # Function that is called when the Pick Intent is activated
    if not rospy.is_shutdown():
        pick_msg = render_template('pick') 
        pick = Pick()
        threading.Thread(target = pick.run).start()
        return statement(pick_msg)
    else:
        return question(render_template('offline'))


@ask.intent("SleepIntent")
def sleep():
    # Function that is called when the Sleep Intent is activated
    if not rospy.is_shutdown(): 
        sleep_msg = render_template('sleep')
        sleep = Sleep()
        threading.Thread(target = sleep.run).start()
        return statement(sleep_msg)
    else:
        return question(render_template('offline'))


@ask.intent("WakeIntent")
def sleep():
    # Function that is called when the Wake Intent is activated
    if not rospy.is_shutdown(): 
        wake_msg = render_template('wake')
        wake = Wake()
        threading.Thread(target = wake.run).start()
        return statement(wake_msg)
    else:
        return question(render_template('offline'))


if __name__ == '__main__':
    app.run(debug=True)