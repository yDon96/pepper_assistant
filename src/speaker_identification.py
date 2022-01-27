#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray
import numpy as np
import pickle
import os
import yaml

from utils.audio import get_mfcc
from utils.model import get_deep_speaker
from utils.utils import batch_cosine_similarity, dist2id

n_embs = 0
X = []
y = []

def get_model(path):
    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    return get_deep_speaker(os.path.join(REF_PATH, path))

def callback(audio, sample_rate, num_fbanks, speaker_model, identification_threshold):
    """
    Callback called each time there is a new record.

    Parameters
    ----------
    audio 
        Audio source
    sample_rate
        The number of samples of audio recorded every second.
    num_fbanks
        Number of filter banks to apply
    speaker_model
        Deep model for speaker recognition
    identification_threshold
        The min value to assign a correct prediction
    """
    audio_data = np.array(audio.data)

    # to float32
    audio_data = audio_data.astype(np.float32, order='C') / 32768.0

    # Processing
    ukn = get_mfcc(audio_data, sample_rate, num_fbanks)

    # Prediction
    ukn = speaker_model.predict(np.expand_dims(ukn, 0))

    if len(X) > 0:
        # Distance between the sample and the support set
        emb_voice = np.repeat(ukn, len(X), 0)

        cos_dist = batch_cosine_similarity(np.array(X), emb_voice)
        
        # Matching
        id_label = dist2id(cos_dist, y, identification_threshold, mode='avg')
    
    if len(X) == 0 or id_label is None:
        c = input("Voce non conosciuta. Vuoi inserire un nuovo campione? (S/N):")
        if c.lower() == 's':
            name = input("Inserisci il nome dello speaker:").lower()
            X.append(ukn[0])
            y.append(name)
    else:
        print("Ha parlato:", id_label)

def init_node(node_name):
    """
    Init the node.

    Parameters
    ----------
    node_name 
        Name assigned to the node
    """
    rospy.init_node(node_name, anonymous=True)

def listener(sample_rate, num_fbanks, model_path, identification_threshold, data_topic):
    """
    Main function of the node.

    Parameters
    ----------
    sample_rate
        The number of samples of audio recorded every second.
    num_fbanks
        Number of filter banks to apply
    model_path
        Path to deep model
    identification_threshold
        The min value to assign a correct prediction
    data_topic
        Topic in which is published audio data
    """
    speaker_model = get_model(model_path)
    rospy.Subscriber(data_topic, Int16MultiArray, lambda audio : callback(audio, sample_rate, num_fbanks, speaker_model, identification_threshold))
    rospy.spin()
        
if __name__ == '__main__':
    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(REF_PATH,'config.yml')) as file:
        config = yaml.full_load(file)
    
    node_name = config['nodes']['reidentification']
    sample_rate = config['settings']['sampleRate']
    num_fbanks = config['settings']['numFbanks']
    model_path = config['models']['defaults']
    identification_threshold = config['settings']['identificationThreshold']
    data_topic = config['topics']['voiceData']

    init_node(node_name)
    listener(sample_rate, 
                num_fbanks, 
                model_path, 
                identification_threshold, 
                data_topic)