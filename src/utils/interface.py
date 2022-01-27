import rospy
from std_msgs.msg import String

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
        print(f"[IN]:{result.data}") 
        return result.data

    def print_output(self,text):
        print("[OUT]:",text)