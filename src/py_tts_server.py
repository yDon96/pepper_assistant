#!/usr/bin/env python
import pyttsx3
import yaml
import os
from ros_pepper_pkg.srv import *
import rospy

def say(self, msg):
    engine.say(msg)
    engine.runAndWait()
    return "ACK"

def main():
    """
    Main function of the service.
    """
    rospy.logdebug('Tts server READY.')
    rospy.spin()

def init_node(service_name):
    """
    Init the node.

    Parameters
    ----------
    service_name 
        Name assigned to the noder
    """
    rospy.init_node(service_name)
    rospy.Service(service_name, Tts, self.say)
    tts_engine = pyttsx3.init()

    return tts_engine

if __name__ == "__main__":

    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(REF_PATH,'config.yml')) as file:
        config = yaml.full_load(file)

    service_name = config['nodes']['pyTts']
    speech_rate = config['tts']['speechRate']

    tts_engine = init_node(service_name)
    tts_engine.setProperty('rate', speech_rate)
    main()
