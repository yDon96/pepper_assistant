#!/usr/bin/env python3
from urllib import response
from ros_pepper_pkg.srv import Dialogue, DialogueResponse
import rospy
import requests
import yaml
import os
import json

def get_post_message(text, identity):
    """
    Generate json of the post request.

    Parameters
    ----------
    text 
        String to send as message
    """
    return { "sender": 'bot', "message": f'{identity}: {text}' }

def get_text_from(rest_response):
    result = ""
    for i in rest_response.json():
        result += i['text'] + ' ' if 'text' in i else ''
    return result

def get_products_from(rest_response):
    result = ""
    list=[]
    for i in rest_response.json():
        if 'custom' in i:
            characters = "'[]"
            records = i['custom']['product']
            for x in range(len(characters)):
                records = records.replace(characters[x],"")
            if "None" not in records:
                record=records.split(",")
                j=0
                while j<len(record):
                    list.append(record[j]+","+record[j+1])
                    j+=2
    return list

def get_dialogue_response_from(rest_response):
    """
    Generate the response of the service.

    Parameters
    ----------
    rest_response 
        Response of rest call
    """
    result = DialogueResponse()
    result.answer = get_text_from(rest_response)
    products = get_products_from(rest_response)

    data = {"text": result.answer, "products": products}
    result.json = json.dumps(data)     

    return result

def handle_service(dialogue_request, server_url):
    """
    Handle the request send to the service.

    Parameters
    ----------
    dialogue_request 
        Request send to the service
    server_url
        Url of nlu server
    """
    message = get_post_message(dialogue_request.input_text,dialogue_request.identity)
    rest_response = requests.post(server_url, json=message)
    dialogue_response = get_dialogue_response_from(rest_response)

    return dialogue_response

def main():
    """
    Main function of the service.
    """
    rospy.logdebug('Dialogue server READY.')
    rospy.spin()

def init_node(service_name, server_url):
    """
    Init the node.

    Parameters
    ----------
    service_name 
        Name assigned to the node
    server_url
        Url of nlu server
    """
    rospy.init_node(service_name)
    service = rospy.Service(service_name, 
                            Dialogue, 
                            lambda dialogue_request : handle_service(dialogue_request, server_url))

if __name__ == '__main__':

    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(REF_PATH,'config.yml')) as file:
        config = yaml.full_load(file)

    service_name = config['nodes']['nluServer']
    server_url = config['nlu']['serverUrl']

    init_node(service_name, server_url)

    try: 
        main()
    except rospy.ROSInterruptException as e:
        pass
