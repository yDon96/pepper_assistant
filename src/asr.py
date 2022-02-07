#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray, String, Bool
import numpy as np
import yaml
from speech_recognition import AudioData
import speech_recognition as sr
from speech_recognition import UnknownValueError, RequestError
import os
    


class PublisherProvider:

    def __init__(self, data_topic, text_topic, identity_data_topic, identity_text_topic):
        self.data_publisher = rospy.Publisher(data_topic, Int16MultiArray, queue_size=10)
        self.text_publisher = rospy.Publisher(text_topic, String, queue_size=10)
        self.identity_data_publisher = rospy.Publisher(identity_data_topic, Int16MultiArray, queue_size=1)
        self.identity_text_publisher = rospy.Publisher(identity_text_topic, String, queue_size=1)
        self.is_identity = False
        

    def get_publishers(self):
        if not self.is_identity:
            return self.data_publisher, self.text_publisher
        else:
            return self.identity_data_publisher, self.identity_text_publisher

    def change_publisher(self, is_identity):
        self.is_identity = is_identity.data

def callback(audio, recognizer, publisher_provider, mic_status_publisher, sample_rate, language):
    """
    Callback called each time there is a new record.

    Parameters
    ----------
    audio 
        Audio source
    recognizer
        SpeechRecognition recognizer
    data_publisher
        Rospy publisher
    text_publisher
        Rospy publisher
    sample_rate
        The number of samples of audio recorded every second.
    language
        The language used by google to transcribe the audio
    """
        
    data = np.array(audio.data,dtype=np.int16)
    audio_data = AudioData(data.tobytes(), sample_rate, 2)

    try:
        spoken_text= recognizer.recognize_google(audio_data, language=language)
        print("Google Speech Recognition says: " + spoken_text)
        data_publisher, text_publisher = publisher_provider.get_publishers()
        data_publisher.publish(audio) # Publish audio only if it contains words
        text_publisher.publish(spoken_text)
    except UnknownValueError:
        print("Google Speech Recognition unknown value")
        mic_status_publisher.publish(True)
    except RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))
        mic_status_publisher.publish(True)

def init_node(node_name, data_topic, text_topic, identity_data_topic, identity_text_topic, mic_status_topic):
    rospy.init_node(node_name, anonymous=True)
    publisher_provider = PublisherProvider(data_topic, text_topic, identity_data_topic, identity_text_topic)
    mic_status_publisher = rospy.Publisher(mic_status_topic, Bool, queue_size=1)

    return publisher_provider, mic_status_publisher

def listener(publisher_provider, mic_status_publisher, microphone_topic, sample_topic, sample_rate, language):
    """
    Start follow the audio recording.

    Parameters
    ----------
    data_publisher
        Rospy publisher
    text_publisher
        Rospy publisher
    microphone_topic
        The subscribe topic
    sample_rate
        The number of samples of audio recorded every second.
    language
        The language used by google to transcribe the audio
    """
    # Initialize a Recognizer
    recognizer = sr.Recognizer()
    rospy.Subscriber(sample_topic, Bool, publisher_provider.change_publisher)
    rospy.Subscriber(microphone_topic, Int16MultiArray, lambda audio : callback(audio, 
                                                                                recognizer,
                                                                                publisher_provider,
                                                                                mic_status_publisher,
                                                                                sample_rate,
                                                                                language))
    rospy.spin()

if __name__ == '__main__':
    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(REF_PATH,'config.yml')) as file:
        config = yaml.full_load(file)

    node_name = config['nodes']['asr']
    data_topic = config['topics']['voiceData']
    text_topic = config['topics']['voiceText']
    mic_status_topic = config['topics']['micStatus']
    microphone_topic = config['topics']['microphone']
    sample_rate = config['settings']['sampleRate']
    language = config['settings']['language']
    identity_data_topic = config['topics']['identityData']
    identity_text_topic = config['topics']['identityText']
    sample_topic = config['topics']['sample']

    publisher_provider, mic_status_publisher = init_node(node_name, 
                                                            data_topic, 
                                                            text_topic, 
                                                            identity_data_topic, 
                                                            identity_text_topic, 
                                                            mic_status_topic)
    listener(publisher_provider,
                mic_status_publisher, 
                microphone_topic,
                sample_topic,
                sample_rate, 
                language)