#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray, String
import numpy as np
import yaml
from speech_recognition import AudioData
import speech_recognition as sr
from speech_recognition import UnknownValueError, RequestError

# this is called from the background thread
def callback(audio, recognizer, data_publisher, text_publisher, config):
    data = np.array(audio.data,dtype=np.int16)
    audio_data = AudioData(data.tobytes(), config['settings']['sampleRate'], 2)

    try:
        spoken_text= recognizer.recognize_google(audio_data, language=config['settings']['language'])
        print("Google Speech Recognition says: " + spoken_text)
        data_publisher.publish(audio) # Publish audio only if it contains words
        text_publisher.publish(spoken_text)
    except UnknownValueError:
        print("Google Speech Recognition unknown value")
    except RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))


def init_node(config):
    rospy.init_node(config['nodes']['asr'], anonymous=True)
    data_publisher = rospy.Publisher(config['topics']['voiceData'], Int16MultiArray, queue_size=10)
    text_publisher = rospy.Publisher(config['topics']['voiceText'], String, queue_size=10)

    return data_publisher, text_publisher

def listener(data_publisher, text_publisher, config):
    # Initialize a Recognizer
    recognizer = sr.Recognizer()
    rospy.Subscriber(config['topics']['microphone'], Int16MultiArray, lambda audio : callback(audio, 
                                                                                recognizer,
                                                                                data_publisher, 
                                                                                text_publisher,
                                                                                config))
    rospy.spin()

if __name__ == '__main__':
    with open('config.yml') as file:
        config = yaml.full_load(file)

    data_publisher, text_publisher = init_node(config)
    listener(data_publisher, 
                text_publisher, 
                config)