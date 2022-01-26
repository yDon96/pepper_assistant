#!/usr/bin/env python3

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

def main(service, interface):
    while not rospy.is_shutdown():
        message = terminal.get_input()
        if message == 'exit': 
            break
        try:
            bot_answer = service(message)
            terminal.print_output(bot_answer.answer)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

def init_node():
    rospy.init_node('writing')
    rospy.wait_for_service('dialogue_server')
    dialogue_service = rospy.ServiceProxy('dialogue_server', Dialogue)
    return dialogue_service

if __name__ == '__main__':

    dialogue_service = init_node()
    terminal = TerminalInterface()

    try: 
        main(dialogue_service, terminal)
    except rospy.ROSInterruptException:
        pass