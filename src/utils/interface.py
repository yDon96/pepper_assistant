import rospy
from std_msgs.msg import String

class TerminalInterface:
    '''Class implementing a terminal i/o interface. 

    Methods
    - get_input(self): return a string read from the terminal
    - print_output(self, text): prints the text on the terminal

    '''

    def get_input(self):
        return input("[IN]:  "), 'Rossi'

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
        return result.data, 'Rossi'

    def print_output(self,text):
        print("[OUT]:",text)

class VoiceInterface:
    '''Class implementing a voice input/terminal output interface. 

    Methods
    - get_input(self): return a string read from the voice data topic
    - print_output(self, text): prints the text on the terminal and publish text 

    '''
    def __init__(self, rospy, publisher, input_topic):
        self.rospy = rospy
        self.input_topic = input_topic
        self.publisher = publisher

    def get_input(self):
        result = rospy.wait_for_message(self.input_topic, String)
        print(f"[IN]:{result.data}") 
        return result.data, 'Rossi'

    def print_output(self,text):
        self.publisher.publish(text)
        print("[OUT]:",text)

class VoiceIdentityInterface:
    '''Class implementing a voice input/terminal output interface. 

    Methods
    - get_input(self): return a string read from the voice data topic
    - print_output(self, text): prints the text on the terminal and publish text 

    '''
    def __init__(self, rospy, publisher, input_topic, identification_topic):
        self.rospy = rospy
        self.input_topic = input_topic
        self.publisher = publisher
        self.identification_topic = identification_topic

    def get_input(self):
        message = rospy.wait_for_message(self.input_topic, String)
        identity = rospy.wait_for_message(self.identification_topic, String)
        print(f"[IN]:{identity.data}: {message.data}") 
        return message.data, identity.data

    def print_output(self,text):
        self.publisher.publish(text)
        print("[OUT]:",text)