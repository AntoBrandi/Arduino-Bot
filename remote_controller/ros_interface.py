import roslibpy
import messenger
import controller
import voice_assistant

ROBOT_IP = "192.168.0.166"
ROBOT_PORT = 9090


class RosInterface:

    def __init__(self):
        self.client = roslibpy.Ros(host=ROBOT_IP, port=ROBOT_PORT)
        self.messenger_publisher = roslibpy.Topic(self.client, messenger.MESSENGER_TOPIC, messenger.MESSENGER_TYPE)
        self.controller_publisher = roslibpy.Topic(self.client, controller.CONTROLLER_TOPIC, controller.CONTROLLER_TYPE)
        self.voice_publisher = roslibpy.Topic(self.client, voice_assistant.VOICE_TOPIC, voice_assistant.VOICE_TYPE)

    def connect(self):
        try:
            self.client.run()
            self.messenger_publisher.advertise()
            self.controller_publisher.advertise()
            self.voice_publisher.advertise()
            print('Connected')
        except Exception as e:
            print('WebSocket Error: ' + str(e))

    def publish(self, topic, msg):
        if self.isConnected():
            if topic == messenger.MESSENGER_TOPIC:
                self.messenger_publisher.publish(msg)
            elif topic == controller.CONTROLLER_TOPIC:
                self.controller_publisher.publish(msg)
            elif topic == voice_assistant.VOICE_TOPIC:
                self.voice_publisher.publish(msg)
            else:
                print('Invalid topic')
            print('Sending message...')
        else:
            print('The client is not connected to the server')

    def isConnected(self):
        return self.client.is_connected

    def closeConnection(self):
        if self.isConnected():
            self.client.terminate()
            self.messenger_publisher.unadvertise()
            self.controller_publisher.unadvertise()
        else:
            print('Connection was already closed')
