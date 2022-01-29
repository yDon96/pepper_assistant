#!/usr/bin/env python3
import yaml
import os
import rospy
from ros_pepper_pkg.srv import *
from std_msgs.msg import String

def callback(message, tts_service):
    try:
        bot_answer = tts_service(message.data)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


def main():
    """
    Main function of the interface.
    """
    rospy.logdebug('Tts interface READY.')
    rospy.spin()

def init_node(node_name, service_name, topic):
    """
    Init the node.

    Parameters
    ----------
    node_name 
        Name assigned to the node
    service_name
        Name of the service to call
    topic
        topic on which the messages for the tts are published
    """
    rospy.init_node(node_name)
    rospy.wait_for_service(service_name)
    tts_service = rospy.ServiceProxy(service_name, Tts)
    rospy.Subscriber(topic, String, lambda message : callback(message, tts_service))

if __name__ == '__main__':

    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(REF_PATH,'config.yml')) as file:
        config = yaml.full_load(file)

    service_tag = rospy.get_param('service_tag')
    node_name = config['nodes']['ttsInterface']
    service_name = config['nodes'][service_tag]
    input_topic = config['topics']['outputText']
    init_node(node_name, service_name, input_topic)
    main()