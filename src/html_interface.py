#!/usr/bin/env python3
import yaml
import os
import rospy
from ros_pepper_pkg.srv import *
from std_msgs.msg import String
from utils.html_generator import HtmlGenerator
import json

def get_value_from_json(json_message):
    # string obtained by json.dumps
    results = json.loads(json_message)
    return results["text"], results["products"]

def callback(message, html_generator, html_service):
    """
    Topic callback.

    Parameters
    ----------
    message 
        Message for the service
    html_service
        Service to call
    """
    print(message)
    text, product_list = get_value_from_json(message.data)
    html_generator.generate_html(text, product_list)
    try:
        html_answer = html_service()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def main():
    """
    Main function of the interface.
    """
    rospy.logdebug('Html interface READY.')
    rospy.spin()

def init_node(node_name, service_name, topic, path_template, path_output):
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
    html_service = rospy.ServiceProxy(service_name, Html)
    html_generator = HtmlGenerator(path_template,path_output)
    rospy.Subscriber(topic, String, lambda message : callback(message, html_generator, html_service))

if __name__ == '__main__':

    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(REF_PATH,'config.yml')) as file:
        config = yaml.full_load(file)

    service_tag = rospy.get_param('html_service_tag')
    node_name = config['nodes']['htmlInterface']
    service_name = config['nodes'][service_tag]
    input_topic = config['topics']['htmlData']
    path_template = os.path.join(REF_PATH,config['html']['template'])
    path_output = os.path.join(REF_PATH,config['html']['output'])

    init_node(node_name, service_name, input_topic, path_template, path_output)
    main()