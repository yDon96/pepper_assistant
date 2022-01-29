#!/usr/bin/env python
from naoqi import ALProxy
from optparse import OptionParser
from pepper_nodes.srv import *
import rospy

class Text2SpeechNode:

    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.tts = ALProxy("ALTextToSpeech", ip, port)
        

    def say(self, msg):
        try:
            self.tts.say(msg.speech)
        except:
            self.tts = ALProxy("ALTextToSpeech", self.ip, self.port)
            self.tts.say(msg.speech)
        return "ACK"
    
    def start(self,service_name):
        rospy.init_node(service_name)
        rospy.Service(service_name, Tts, self.say)

        rospy.spin()

if __name__ == "__main__":
    parser = OptionParser()
    parser.add_option("--ip", dest="ip", default="10.0.1.207")
    parser.add_option("--port", dest="port", default=9559)
    (options, args) = parser.parse_args()

    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(REF_PATH,'config.yml')) as file:
        config = yaml.full_load(file)

    service_name = config['nodes']['pepperTts']

    try:
        ttsnode = Text2SpeechNode(options.ip, int(options.port))
        ttsnode.start(service_name)
    except rospy.ROSInterruptException:
        pass