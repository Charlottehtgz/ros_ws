#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Intergrated by Angelo Antikatzidis https://github.com/a-prototype/vosk_ros
# Source code based on https://github.com/alphacep/vosk-api/blob/master/python/example/test_microphone.py from VOSK's example code

# Tuned for the python flavor of VOSK: vosk-0.3.31
# If you do not have vosk then please install it by running $ pip3 install vosk
# If you have a previous version of vosk installed then update it by running $ pip3 install vosk --upgrade
# Tested on ROS Noetic & Melodic. Please advise the "readme" for using it with ROS Melodic 

# This is a node that intergrates VOSK with ROS and supports a TTS engine to be used along with it
# When the TTS engine is speaking some words, the recognizer will stop listenning to the audio stream so it won't listen to it self :)

# It publishes to the topic speech_recognition/vosk_result a custom "speech_recognition" message
# It publishes to the topic speech_recognition/final_result a simple string
# It publishes to the topic speech_recognition/partial_result a simple string

import spacy

import os
import sys
import json
import queue
import vosk
import sounddevice as sd
from mmap import MAP_SHARED
# from playsound import playsound
import rospy
import rospkg
from ros_vosk.msg import speech_recognition
from std_msgs.msg import String, Bool, Int32
import socket
import pyaudio
import time


# def thanku_callback():
#     global saied_thankyou
#     saied_thankyou = False
#     while not saied_thankyou:
#         continue
#     return True

# rospy.Service('thank', ThankYou, thanku_callback)
# import vosk_ros_model_downloader as downloader

class vosk_sr():
    def __init__(self):
        rospack = rospkg.RosPack()
        rospack.list()
        package_path = rospack.get_path('ros_vosk')

        model_path = '/models/'
        model_dir = package_path + model_path
        model = "vosk-model-small-en-us-0.15" #change the name of the model to match the downloaded model's name
        # model = "vosk-model-small-cn-0.22"
        
        # if not os.path.exists(model_dir+model):
        #     print ("No model found! Please use the GUI to download a model...")
        #     model_downloader = downloader.model_downloader()
        #     model_downloader.execute()
        #     model = model_downloader.model_to_download
        
        self.tts_status = False

        # ROS node initialization
        
        self.pub_vosk = rospy.Publisher('speech_recognition/vosk_result',speech_recognition, queue_size=10)
        self.pub_final = rospy.Publisher('speech_recognition/final_result',String, queue_size=10)
        self.pub_partial = rospy.Publisher('speech_recognition/partial_result',String, queue_size=10)
        self.pub_thanku = rospy.Publisher('speech_recognition/thanku',String, queue_size=10)

        self.rate = rospy.Rate(100)

        rospy.on_shutdown(self.cleanup)

        #model_name = rospy.get_param('vosk/model',model)
        model_name = model
        if not rospy.has_param('vosk/model'):
            rospy.set_param('vosk/model', model_name)

        self.msg = speech_recognition()

        self.q = queue.Queue()

        # self.input_dev_num = sd.query_hostapis()[0]['default_input_device']
        # print(self.input_dev_num)
        # if self.input_dev_num == -1:
        #     rospy.logfatal('No input device found')
        #     raise ValueError('No input device found, device number == -1')

        # device_info = sd.query_devices(self.input_dev_num, 'input')
        # soundfile expects an int, sounddevice provides a float:
        
        # self.samplerate = int(device_info['default_samplerate'])
        self.samplerate = 16000
        rospy.set_param('vosk/sample_rate', self.samplerate)

        self.model = vosk.Model(model_dir+model_name)

        #TODO GPUInit automatically selects a CUDA device and allows multithreading.
        # gpu = vosk.GpuInit() #TODO

    
    def cleanup(self):
        rospy.logwarn("Shutting down VOSK speech recognition node...")

    
    def stream_callback(self, indata, frames, time, status):
        #"""This is called (from a separate thread) for each audio block."""
        if status:
            print(status, file=sys.stderr)
        self.q.put(bytes(indata))
        
    def tts_get_status(self,msg):
        self.tts_status = msg.data

    def tts_status_listenner(self):
        rospy.Subscriber('/tts/status', Bool, self.tts_get_status)

    def speech_recognize(self,a,client):    
        try:
            print("Start recognition")
            rospack = rospkg.RosPack()
            rospack.list()
            package_path = rospack.get_path('ros_vosk')

            response_path = '/response/'
            res_dir = package_path + response_path
            self.state = 0
            self.output= str('nothing')##########
            # with sd.RawInputStream(samplerate=self.samplerate, blocksize=16000, device=self.input_dev_num, dtype='int16',
            #                    channels=1, callback=self.stream_callback):
            #################
            print("Before with data")
            wav = pyaudio.PyAudio()
            
            audio_format = pyaudio.paInt16
            #write = wav.open(format=audio_format,channels=1, callback=self.stream_callback,input=True,frames_per_buffer=1024) #输出
            audio_data = a.recv(self.samplerate)
            print(a)
            self.q.put(bytes(audio_data))
            #self.q.put(audio_data)
            # with audio_data:
            print("After with data")
            rospy.logdebug('Started recording')
            print("After with data 22")
            rec = vosk.KaldiRecognizer(self.model, self.samplerate)
            print("Vosk is ready to listen!")
            isRecognized = False
            isRecognized_partially = False
            
        
            while not rospy.is_shutdown():
                #self.output= str('nothing')##########
                self.tts_status_listenner()
                
                if self.tts_status == True:
                    # If the text to speech is operating, clear the queue
                    with self.q.mutex:
                        self.q.queue.clear()
                    rec.Reset()

                elif self.tts_status == False:
                    audio_data = a.recv(self.samplerate)
                    data = audio_data
                    #print(rec.AcceptWaveform(data))
                    if rec.AcceptWaveform(data):
                        #print("accepted")
                        # In case of final result
                        result = rec.FinalResult()

                        diction = json.loads(result)
                        lentext = len(diction["text"])
                        print(result)
                        if lentext > 2:
                            
                            result_text = diction["text"]
                            rospy.loginfo(result_text)
                            if result_text == "hi robot":
                                self.output= "Start recognizing what is required"
                                print(self.output)
                                #playsound(res_dir+'hello.mp3')
                                str_response = 'zzx' + '1' + 'zzy'
                                print(str_response)
                                client.send(str_response.encode('utf-8'))
                                time.sleep(2)
                                self.state = 1
                            if self.state == 1:
                                nlp = spacy.load("en_core_web_sm")
                                doc = nlp(result_text)
                                for token in doc:
                                    if (token.pos_ == "NOUN" and str(token) != "robot"):
                                        self.output= str(token)
                                        print(self.output)
                                        if (str(token) == "person" or str(token) == "bottle" or str(token) == "car"):
                                            #playsound(res_dir+'/ok.mp3')
                                            str_response = 'zzx' + '2' + 'zzy'
                                            print(str_response)
                                            client.send(str_response.encode('utf-8'))
                                            time.sleep(2)
                                            self.state = 0
                            if result_text == "thank you":
                                print("You are welcome")
                                str_response = 'zzx' + '3' + 'zzy'
                                print(str_response)
                                client.send(str_response.encode('utf-8'))
                                time.sleep(2)
                                self.pub_thanku.publish(str("True"))
                                # global saied_thankyou
                                # saied_thankyou = True

                            isRecognized = True
                        else:
                            isRecognized = False
                        # Resets current results so the recognition can continue from scratch
                        rec.Reset()
                                

                    else:
                # In case of partial result
                        result_partial = rec.PartialResult()
                        if (len(result_partial) > 20):

                            isRecognized_partially = True
                            partial_dict = json.loads(result_partial)
                            partial = partial_dict["partial"]

                    if (isRecognized is True):

                        self.msg.isSpeech_recognized = True
                        self.msg.time_recognized = rospy.Time.now()
                        self.msg.final_result = result_text
                        self.msg.partial_result = "unk"
                        self.pub_vosk.publish(self.msg)
                        rospy.sleep(0.1)
                        self.pub_final.publish(self.output)
                        isRecognized = False


                    elif (isRecognized_partially is True):
                        if partial != "unk":
                            self.msg.isSpeech_recognized = False
                            self.msg.time_recognized = rospy.Time.now()
                            self.msg.final_result = "unk"
                            self.msg.partial_result = partial
                            self.pub_vosk.publish(self.msg)
                            rospy.sleep(0.1)
                            self.pub_partial.publish(partial)
                            partial = "unk"
                            isRecognized_partially = False
                            


        except Exception as e:
            exit(type(e).__name__ + ': ' + str(e))
        except KeyboardInterrupt:
            rospy.loginfo("Stopping the VOSK speech recognition node...")
            rospy.sleep(1)
            print("node terminated")
if __name__ == '__main__':
    try:
        server_audio = socket.socket()
        server_audio.bind(('10.0.0.132', 1500))
        server_audio.listen()
        a,b = server_audio.accept()
        client = socket.socket()  
        client.connect(('10.0.0.128', 1800))
        #write = wav.open(format=audio_format, channels=channels, rate=rate, output=True,frames_per_buffer=chunk_size) #输出
        # audio_data = a.recv(350)
            #write.write(audio_data)
        rospy.init_node('vosk', anonymous=False)
        rec = vosk_sr()
        data = rec.speech_recognize(a,client)
        server_audio.close()
        client.close()
    except (KeyboardInterrupt, rospy.ROSInterruptException) as e:
        rospy.logfatal("Error occurred! Stopping the vosk speech recognition node...")
        rospy.sleep(1)
        print("node terminated")


