#!/usr/bin/env python3
import yaml
import os
import rospy
from ros_pepper_pkg.srv import Dialogue, DialogueResponse

class TerminalInterface:
    '''Class implementing a terminal i/o interface. 

    Methods
    - get_input(self): return a string read from the terminal
    - print_output(self, text): prints the text on the terminal

    '''

    def get_input(self):
        return input("[IN]:  ") 

    def print_output(self,text):
        print("[OUT]:",text)

class VoiceTerminalInterface:
    '''Class implementing a voice input/terminal output interface. 

    Methods
    - get_input(self): return a string read from the voice data topic
    - print_output(self, text): prints the text on the terminal

    '''
    def __init__(self, rospy, topic):
        self.rospy = rospy
        self.topic = topic

    def get_input(self):
        result = rospy.wait_for_message(self.topic, String)
        print(f"[IN]:{result}") 
        return result

    def print_output(self,text):
        print("[OUT]:",text)

def main(service, interface):
    while not rospy.is_shutdown():
        message = interface.get_input()
        if message == 'exit': 
            break
        try:
            bot_answer = service(message)
            interface.print_output(bot_answer.answer)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

def init_node(config):
    node_name = config['nodes']['nluInterface']
    service_name = config['nodes']['nluServer']
    rospy.init_node(node_name)
    rospy.wait_for_service(service_name)
    dialogue_service = rospy.ServiceProxy(service_name, Dialogue)
    return dialogue_service

if __name__ == '__main__':

    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(REF_PATH,'config.yml')) as file:
        config = yaml.full_load(file)

    dialogue_service = init_node(config)
    terminal = TerminalInterface()

    try: 
        main(dialogue_service, terminal)
    except rospy.ROSInterruptException:
        pass