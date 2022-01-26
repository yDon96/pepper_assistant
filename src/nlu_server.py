#!/usr/bin/env python3
from ros_pepper_pkg.srv import Dialogue, DialogueResponse
import rospy
import requests
import yaml
import os

def get_post_message(text):
    return { "sender": 'bot', "message": text }

def get_dialogue_response_from(rest_response):
    result = DialogueResponse()
    result.answer = ""
    for i in rest_response.json():
        result.answer += i['text'] + ' ' if 'text' in i else ''

    return result

def handle_service(req, server_url):

    message = get_post_message(req.input_text )
    rest_response = requests.post(server_url, json=message)
    dialogue_response = get_dialogue_response_from(rest_response)

    return dialogue_response

def main():
    rospy.logdebug('Dialogue server READY.')
    rospy.spin()

def init_node(config):
    service_name = config['nodes']['nluServer']
    server_url = config['nlu']['serverUrl']
    rospy.init_node(service_name)
    service = rospy.Service(service_name, 
                            Dialogue, 
                            lambda dialogue_request : handle_service(dialogue_request, server_url))

if __name__ == '__main__':

    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(REF_PATH,'config.yml')) as file:
        config = yaml.full_load(file)

    init_node(config)

    try: 
        main()
    except rospy.ROSInterruptException as e:
        pass
