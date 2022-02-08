#!/usr/bin/env python3
import yaml
import os
import rospy
from ros_pepper_pkg.srv import Dialogue, DialogueResponse
from utils.interface import *
from std_msgs.msg import String


def get_interface(config, interface_type):
    """
    Generate the input/output interface to be used.

    Parameters
    ----------
    config 
        Configuration dictionary
    interface_type
        Name of the interface to inject
    """
    result = None
    if interface_type == config['interfaceType']['terminal']:
        result = TerminalInterface()
    elif interface_type == config['interfaceType']['voiceHtml']:
        voice_publisher = rospy.Publisher(config['topics']['outputText'], String, queue_size=10)
        html_publisher = rospy.Publisher(config['topics']['htmlData'], String, queue_size=10)
        result = VoiceHtmlInterface(rospy, voice_publisher, html_publisher, config['topics']['voiceText'])
    elif interface_type == config['interfaceType']['terminalHtml']:
        voice_publisher = rospy.Publisher(config['topics']['outputText'], String, queue_size=10)
        html_publisher = rospy.Publisher(config['topics']['htmlData'], String, queue_size=10)
        result = TerminalHtmlInterface(rospy, voice_publisher, html_publisher, config['topics']['voiceText'])
    elif interface_type == config['interfaceType']['voiceTerminal']:
        result = VoiceTerminalInterface(rospy, config['topics']['voiceText'])
    elif interface_type == config['interfaceType']['voice']:
        publisher = rospy.Publisher(config['topics']['outputText'], String, queue_size=10)
        result = VoiceInterface(rospy, publisher, config['topics']['voiceText'])
    elif interface_type == config['interfaceType']['voiceIdentity']:
        publisher = rospy.Publisher(config['topics']['outputText'], String, queue_size=10)
        result = VoiceIdentityInterface(rospy, publisher, config['topics']['voiceText'], config['topics']['identity'])

    return result

def main(service, interface):
    """
    Main function of the node.

    Parameters
    ----------
    service 
        Service to call
    interface
        Interface that handle input/output
    """
    while not rospy.is_shutdown():
        message, identity = interface.get_input()
        if message == 'exit': 
            break
        try:
            bot_answer = service(message)
            interface.print_output(bot_answer)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

def init_node(node_name, service_name):
    """
    Init the node.

    Parameters
    ----------
    node_name 
        Name assigned to the node
    service_name
        Name of the service to call
    """
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