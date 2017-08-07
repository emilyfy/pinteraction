#!/usr/bin/env python

## Performs real time fourier transform on music file
## Outputs dB values on 10 frequency bands relative to 10.0 on topic /height/1
## Shifts its history with each instance, up to 10 last instance, on topic /height/2 height/3 etc
## Receives feedback data on topic /feedback
## Applies real time filter on played music based on feedback

# signal processing
import numpy as np
from scipy.fftpack import fft, ifft, rfft, irfft, fftfreq

# audio playing
from scipy.io import wavfile
from os import path
import sounddevice
from obspy.signal.filter import bandstop

# ros
import rospy
from std_msgs.msg import UInt16, UInt16MultiArray
from pinteraction.msg import BoolMultiArray

# calculations
from math import ceil, log10
from time import time, sleep

class Sound(object):
    n = 2048             # no of samples in each block
    
    def __init__(self, filename, filepath):
        self.soundfile = path.join(path.dirname(__file__), filepath, filename)
        self.fs, self.y = wavfile.read(self.soundfile)
        
        # average two channels if stereo
        if hasattr(self.y[0], "__len__"):
            # self.y = [int(np.mean(self.y[i])) for i in range(len(self.y))]
            self.y = [self.y[i][0] for i in range(len(self.y))]
        
        self.N = len(self.y)                                     # no of samples
        self.T = 1.0 / self.fs                                   # time btw samples
        self.x = np.linspace(0.0, self.N*self.T, self.N)         # create x axis of sound file
        self.duration = self.N/self.fs                           # duration of song
        self.xf = np.linspace(0.0, 1.0/(2.0*self.T), self.n/2)   # x axis for fft

        # make sure data types are of float64, float32, int32, int16, int8 or uint8
        dt = np.dtype(self.y[0])
        acctype = [np.dtype('float64'), np.dtype('float32'), np.dtype('int32'), np.dtype('int16'), np.dtype('int8'), np.dtype('uint8')]
        if dt not in acctype:
            self.y = np.asarray(self.y, dtype=np.int16)
        

    def start(self):
        sounddevice.play(self.y, self.fs)
        self.starttime = time()
        self.currblock = 0
        self.editedblock = 0
        self.fftblock = 0

    def fft(self, i):
        self.currblock = i/self.n
        
        # # fft to block by block instead of per sample if can't afford computational/communication time
        # # doesn't remove audio glitches though
        if self.currblock == self.fftblock:
            return

        # fft
        yt = self.y[i:i+self.n]
        yf = fft(yt)
        ya = 2.0/self.n * np.abs(yf[0:self.n/2])

        # separate into 10 bands
        yd = []
        for j in range(10):
            ys = []
            for k in range(int((2.0**(j+1))-(2.0**(j)))):
                ys.append(ya[int(2.0**(j))-1+k])
            ysm = np.mean(ys)*100
            if not ysm > 0:
                yd.append(0)
            else:
                yd.append(20*log10(ysm))
    

def main():
    rospy.init_node('fourier')
    rospy.set_param('mode', 2)
    n = 2048
    soundfile = path.join(path.dirname(__file__), '../../../../Pinteraction_audio_files', 'Spirited Away Always With Me - Piano.wav')
    fs, y = wavfile.read(soundfile)

    # average two channels if stereo
    if hasattr(y[0], "__len__"):
        # y = [int(np.mean(y[i])) for i in range(len(y))]
        y = [y[i][0] for i in range(len(y))]

    sounddevice.play(y, fs)
    starttime = time()
    y[100*2048:len(y)] = np.zeros_like(y[100*2048:len(y)])

    N = len(y)                                     # no of samples
    T = 1.0 / fs                                   # time btw samples
    x = np.linspace(0.0, N*T, N)         # create x axis of sound file
    duration = N/fs                           # duration of song
    xf = np.linspace(0.0, 1.0/(2.0*T), n/2)   # x axis for fft

    currtime = time()
    while (currtime < starttime + duration) and not rospy.is_shutdown():
        i = int((currtime - starttime)/T)
        print y[i]
        # Baa.fft(i)
        currtime = time()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print e