#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray
import numpy as np
import pickle
import os
import yaml
import h5py

from utils.audio import get_mfcc
from utils.model import get_deep_speaker
from utils.utils import batch_cosine_similarity, dist2id

n_embs = 0
X = []
y = []

def save_dataset():
    predictions = np.array(X)
    dt = h5py.special_dtype(vlen=str)
    labels = np.array(y, dtype=dt) 
    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(REF_PATH, 'data/dataset.hdf5')

    with h5py.File(path, "w") as h5f :
        h5f.create_dataset("predictions", data=predictions)
        h5f.create_dataset("labels", data=labels)

def load_dataset():
    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(REF_PATH, 'data/dataset.hdf5')

    predictions = []
    labels = []
    if os.path.isfile(path):        
        with h5py.File(path, 'r') as f:
            predictions = f['predictions'][:].tolist()
            labels = f['labels'][:].tolist() 
    return predictions, labels       

def get_model(path):
    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    return get_deep_speaker(os.path.join(REF_PATH, path))

def process_audio(data, simple_rate, num_fbanks):
    result = np.array(data)

    # to float32
    result = result.astype(np.float32, order='C') / 32768.0

    # Processing
    result = get_mfcc(result, sample_rate, num_fbanks)
    return result

def get_label_from(prediction, identification_threshold):
    result = None

    if len(X) > 0:
        # Distance between the sample and the support set
        emb_voice = np.repeat(prediction, len(X), 0)

        cos_dist = batch_cosine_similarity(np.array(X), emb_voice)
        
        # Matching
        result = dist2id(cos_dist, y, identification_threshold, mode='avg')

    return result

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
    processed_audio = process_audio(audio.data, simple_rate, num_fbanks)

    prediction = speaker_model.predict(np.expand_dims(processed_audio, 0))

    id_label = get_label_from(prediction, identification_threshold)
    
    if len(X) == 0 or id_label is None:
        c = input("Voce non conosciuta. Vuoi inserire un nuovo campione? (S/N):")
        if c.lower() == 's':
            name = input("Inserisci il nome dello speaker:").lower()
            X.append(prediction[0])
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
    rospy.on_shutdown(save_dataset)
    predictions, labels = load_dataset()
    X.extend(predictions)
    y.extend(labels)

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