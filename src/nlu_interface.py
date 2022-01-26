#!/usr/bin/env python3

import rospy
from rasa_ros.srv import Dialogue, DialogueResponse

class TerminalInterface:
    '''Class implementing a terminal i/o interface. 

    Methods
    - get_text(self): return a string read from the terminal
    - set_text(self, text): prints the text on the terminal

    '''

    def get_text(self):
        return input("[IN]:  ") 

    def set_text(self,text):
        print("[OUT]:",text)

def main():
    rospy.init_node('writing')
    rospy.wait_for_service('dialogue_server')
    dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)

    terminal = TerminalInterface()

    while not rospy.is_shutdown():
        message = terminal.get_text()
        if message == 'exit': 
            break
        try:
            bot_answer = dialogue_service(message)
            terminal.set_text(bot_answer.answer)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

if __name__ == '__main__':
    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(REF_PATH,'config.yml')) as file:
        config = yaml.full_load(file)

    nlu_folder_path = config['nlu']['path']
    subprocess.Popen(['sh', './scripts/rasa_action.sh', nlu_folder_path]).wait()
    
    try: 
        main()
    except rospy.ROSInterruptException:
        pass