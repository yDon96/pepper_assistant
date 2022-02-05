#!/usr/bin/python3
import rospy
from std_msgs.msg import Int16MultiArray, String, Bool
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

def save_dataset(dataset_path):
    predictions = np.array(X)
    dt = h5py.special_dtype(vlen=str)
    labels = np.array(y, dtype=dt) 

    with h5py.File(dataset_path, "w") as h5f :
        h5f.create_dataset("predictions", data=predictions)
        h5f.create_dataset("labels", data=labels)

def load_dataset(dataset_path):
    predictions = []
    labels = []
    if os.path.isfile(dataset_path):        
        with h5py.File(dataset_path, 'r') as dataset:
            predictions = dataset['predictions'][:].tolist()
            labels = dataset['labels'][:].tolist() 
    return predictions, labels   

def get_model(path):
    REF_PATH = os.path.dirname(os.path.abspath(__file__))
    return get_deep_speaker(os.path.join(REF_PATH, path))

def process_audio(data, sample_rate, num_fbanks):
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

def callback(audio, sample_rate, num_fbanks, speaker_model, identification_threshold, sample_phrases, identity_publisher, sample_publisher, speaker_publisher):
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
    processed_audio = process_audio(audio.data, sample_rate, num_fbanks)

    prediction = speaker_model.predict(np.expand_dims(processed_audio, 0))

    id_label = get_label_from(prediction, identification_threshold)

    print("work")
    
    if len(X) == 0 or id_label is None:
        sample_publisher.publish(True)

        predictions = []

        predictions.append(prediction[0])
        
        speaker_publisher.publish("I don't recognize your voice, do you want to register?")
        response = rospy.wait_for_message("identity_text", String)

        if response.data == "yes":      
            print("sta andando")      
            for phrase in sample_phrases:
                print("forse")
                speaker_publisher.publish(phrase)
                result = rospy.wait_for_message("identity_data", Int16MultiArray)
                processed_audio = process_audio(result.data, sample_rate, num_fbanks)
                prediction = speaker_model.predict(np.expand_dims(processed_audio, 0))
                predictions.append(prediction[0])

        
        speaker_publisher.publish("What's your name?")
        name = rospy.wait_for_message("identity_text", String)
        X.extend(predictions)
        print([name]*len(predictions))
        y.extend([name.data]*len(predictions))
        sample_publisher.publish(False)
        identity_publisher.publish(name)
    else:
        identity_publisher.publish(id_label)
        print("The user is:", id_label)

def init_node(node_name, dataset_path):
    """
    Init the node.

    Parameters
    ----------
    node_name 
        Name assigned to the node
    """
    rospy.init_node(node_name, anonymous=True)
    rospy.on_shutdown(lambda:save_dataset(dataset_path))
    predictions, labels = load_dataset(dataset_path)
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
    print(sample_rate)
    speaker_model = get_model(model_path)
    identity_publisher = rospy.Publisher('identity', String, queue_size=1)
    sample_publisher = rospy.Publisher('sample', Bool, queue_size=1)
    speaker_publisher = rospy.Publisher('output_text', String, queue_size=1)
    sample_phrases = ["how are you?", "add bread to my shopping list","change my shopping list"]
    rospy.Subscriber(data_topic, Int16MultiArray, lambda audio : callback(audio, 
                                                                            sample_rate, 
                                                                            num_fbanks, 
                                                                            speaker_model, 
                                                                            identification_threshold,
                                                                            sample_phrases,
                                                                            identity_publisher, 
                                                                            sample_publisher,
                                                                            speaker_publisher))
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
    dataset_path = os.path.join(REF_PATH, config['models']['dataset'])

    init_node(node_name, dataset_path)
    listener(sample_rate, 
                num_fbanks, 
                model_path, 
                identification_threshold, 
                data_topic)