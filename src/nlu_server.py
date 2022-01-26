#!/usr/bin/env python3
from pepper_assistant.srv import Dialogue, DialogueResponse
import subprocess
import rospy
import requests
import yaml


def handle_service(req):
    input_text = req.input_text   

    # Get answer        
    get_answer_url = 'http://localhost:5002/webhooks/rest/webhook'
    message = {
        "sender": 'bot',
        "message": input_text
    }

    r = requests.post(get_answer_url, json=message)
    response = DialogueResponse()
    response.answer = ""
    for i in r.json():
        response.answer += i['text'] + ' ' if 'text' in i else ''

    return response

def main():

    # Server Initialization
    rospy.init_node('dialogue_service')

    s = rospy.Service('dialogue_server',
                        Dialogue, handle_service)

    rospy.logdebug('Dialogue server READY.')
    rospy.spin()


if __name__ == '__main__':

    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(REF_PATH,'config.yml')) as file:
        config = yaml.full_load(file)

    nlu_folder_path = config['nlu']['path']
    subprocess.Popen(['sh', './scripts/rasa_action.sh', nlu_folder_path]).wait()
    subprocess.Popen(['sh', './scripts/rasa_server.sh', nlu_folder_path]).wait()
    try: 
        main()
    except rospy.ROSInterruptException as e:
        pass
