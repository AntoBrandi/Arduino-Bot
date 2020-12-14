import logging
from flask import Flask, render_template
from flask_ask import Ask, statement, question, session
import rospy
import threading
from std_msgs.msg import String


threading.Thread(target=lambda: rospy.init_node('alexa_interface', disable_signals=True, anonymous=True)).start()
pub = rospy.Publisher('alexa_interface', String, queue_size=1)

app = Flask(__name__)
ask = Ask(app, "/")
logging.getLogger("flask_ask").setLevel(logging.DEBUG)


@ask.launch
def launch():
    # function that gets called when the skill is activated
    if not rospy.is_shutdown(): 
        launch_msg = render_template('online')
    else:
        launch_msg = render_template('offline')
    return question(welcome_msg)


@ask.intent("DanceIntent")
def dance():
    # Function that is called when the Dance Intent is activated
    pub.publish("Dance")
    dance_msg = render_template('dance')
    return statement(dance_msg)


@ask.intent("PickIntent")
def pick():
    # Function that is called when the Pick Intent is activated
    pub.publish("Pick")
    pick_msg = render_template('pick')
    return statement(pick_msg)


if __name__ == '__main__':
    app.run(debug=True)