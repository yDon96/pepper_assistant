#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray
import numpy as np
import yaml
import time
import speech_recognition as sr
import os
from std_msgs.msg import Bool


stop_listening = None

def callback_microphone(set_on, recognizer, microphone, publisher):
    if set_on:
        print("Setting ON Mic")
        print("Recording...")
        stop = recognizer.listen_in_background(microphone, 
                                                        lambda recognizer, audio : callback(recognizer, 
                                                                                                audio,
                                                                                                publisher))
        stop_listening = stop
    else:
        print("Setting OFF mic")
        stop_listening(wait_for_stop=False)

def callback(recognizer, audio, pub):
    """
    Callback called each time the recognizer find audio.

    Parameters
    ----------
    recognizer
        SpeechRecognition recognizer
    audio 
        Audio source
    publisher
        Rospy publisher
    """
    data = np.frombuffer(audio.get_raw_data(), dtype=np.int16)
    data_to_send = Int16MultiArray()
    data_to_send.data = data
    
    pub.publish(data_to_send)

def calibrate_noise(microphone, recognizer, calibration_time):
    """
    Calibration within the environment.
    It only need to calibrate once, before it start listening.

    Parameters
    ----------
    microphone 
        Audio source
    recognizer
        SpeechRecognition recognizer
    calibration_time
        Time required to calibrate the environment
    """
    print("Calibrating...")
    with microphone as source:
        recognizer.adjust_for_ambient_noise(source, duration=calibration_time)  
    print("Calibration finished")


def record(calibration_time, sample_rate, chunk_size, publisher, mic_status_topic):
    """
    Start new recording session.

    Parameters
    ----------
    calibration_time 
        Time required to calibrate the environment
    sample_rate
        The number of samples of audio recorded every second.
    chunk_size
        The chunk-size field is a string of hex digits indicating the size of the chunk
    publisher
        Rospy publisher 
    """
    # Initialize a Recognizer
    recognizer = sr.Recognizer()

    # Audio source
    microphone = sr.Microphone(device_index=None,
                                sample_rate=sample_rate,
                                chunk_size=chunk_size)

    calibrate_noise(microphone, recognizer,calibration_time)
    # start listening in the background
    # `stop_listening` is now a function that, when called, stops background listening
    print("Recording...")
    stop = recognizer.listen_in_background(microphone, 
                                                        lambda recognizer, audio : callback(recognizer, 
                                                                                                audio,
                                                                                                publisher)) 
    rospy.Subscriber(mic_status_topic, Bool, lambda status : callback_microphone(status.data, 
                                                                                    recognizer,
                                                                                    microphone, 
                                                                                    publisher))
    stop_listening = stop
    rospy.spin()

def init_node(node_name, publish_topic):
    """
    Init the node.

    Parameters
    ----------
    node_name 
        Name assigned to the node
    publish_topic
        Name of the publisher topic
    """
    rospy.init_node(node_name, anonymous=True)
    publisher = rospy.Publisher(publish_topic, Int16MultiArray, queue_size=10)
    return publisher
    

if __name__ == '__main__':
    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(REF_PATH,'config.yml')) as file:
        config = yaml.full_load(file)

    calibration_time = config['settings']['calibrationTime']
    sample_rate = config['settings']['sampleRate']
    chunk_size = config['settings']['chunkSize']
    node_name = config['nodes']['voiceRecorder']
    publish_topic = config['topics']['microphone']
    mic_status_topic = config['topics']['micStatus']

    publisher = init_node(node_name, publish_topic)
    record(calibration_time, sample_rate, chunk_size, publisher, mic_status_topic)
