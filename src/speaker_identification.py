#!/usr/bin/python3
from tensorflow.python.ops.gen_logging_ops import Print
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

def callback(audio, sample_rate, speaker_model, identification_threshold):
    audio_data = np.array(audio.data)

    # to float32
    audio_data = audio_data.astype(np.float32, order='C') / 32768.0

    # Processing
    ukn = get_mfcc(audio_data, sample_rate)

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

def init_node(config):
    rospy.init_node(config['nodes']['reidentification'], anonymous=True)

def listener(config):
    sample_rate = config['settings']['sampleRate']
    model_path = config['models']['defaults']
    identification_threshold = config['settings']['identificationThreshold']
    speaker_model = get_model(model_path)
    rospy.Subscriber(config['topics']['voiceData'], Int16MultiArray, lambda audio : callback(audio, sample_rate, speaker_model, identification_threshold))
    rospy.spin()
        
if __name__ == '__main__':
    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(REF_PATH,'config.yml')) as file:
        config = yaml.full_load(file)
    
    init_node(config)
    listener(config)