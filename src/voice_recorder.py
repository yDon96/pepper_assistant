 #!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray
import numpy as np
import yaml
import time
import speech_recognition as sr

def callback(recognizer, audio, pub):
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


def record(calibration_time, sample_rate, chunk_size, publisher):
    """
    Start new recording session.
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
    stop_listening = recognizer.listen_in_background(microphone, 
                                                        lambda recognizer, audio : callback(recognizer, 
                                                                                                audio, 
                                                                                                publisher))

    rospy.spin()

def init_node(config):
    rospy.init_node(config['nodes']['voiceRecorder'], anonymous=True)
    publisher = rospy.Publisher(config['topics']['microphone'], Int16MultiArray, queue_size=10)
    record(config['settings']['calibrationTime'],
            config['settings']['sampleRate'],
            config['settings']['chunkSize'],
            publisher)

if __name__ == '__main__':
    with open('config.yml') as file:
        config = yaml.full_load(file)

    init_node(config)
