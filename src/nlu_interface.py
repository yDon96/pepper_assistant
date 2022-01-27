#!/usr/bin/env python3
import yaml
import os
import rospy
from ros_pepper_pkg.srv import Dialogue, DialogueResponse
from utils.interface import TerminalInterface, VoiceTerminalInterface

def get_interface(config, interface_type):
    result = None
    if interface_type == config['interfaceType']['terminal']:
        result = TerminalInterface()
    elif interface_type == config['interfaceType']['voiceTerminal']:
        result = VoiceTerminalInterface(rospy, config['topics']['voiceText'])
    return result

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

def init_node(node_name, service_name):
    rospy.init_node(node_name)
    rospy.wait_for_service(service_name)
    dialogue_service = rospy.ServiceProxy(service_name, Dialogue)
    return dialogue_service

if __name__ == '__main__':

    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(REF_PATH,'config.yml')) as file:
        config = yaml.full_load(file)

    interface_type = rospy.get_param('interface_type')
    node_name = config['nodes']['nluInterface']
    service_name = config['nodes']['nluServer']
    dialogue_service = init_node(node_name, service_name)
    interface = get_interface(config, interface_type)

    try: 
        main(dialogue_service, interface)
    except rospy.ROSInterruptException:
        pass