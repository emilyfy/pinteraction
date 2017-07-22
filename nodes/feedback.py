#!/usr/bin/env python

## Performs real time fourier transform on music file
## Outputs dB values on 10 frequency bands relative to 10.0 on topic /height/1
## Shifts its history with each instance, up to 10 last instance, on topic /height/2 height/3 etc
## Receives feedback data on topic /feedback
## Applies real time filter on played music based on feedback

# signal processing
import numpy as np
from scipy.fftpack import fft, ifft

# audio playing
from scipy.io import wavfile
from os import path
import sounddevice
from obspy.signal.filter import bandstop, bandpass

# ros
import rospy
from std_msgs.msg import UInt8, UInt16, UInt16MultiArray

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
        
        # to determine ym of each band, run ft on whole music first
        self.ym = [0.0] * 10
        arr = np.linspace(0,self.N,self.N/self.n)
        arr = [int(i) for i in arr]
        arr.pop()
        for i in arr:
            yt = self.y[i:i+self.n]
            yf = fft(yt)
            yf = 2.0/self.n * np.abs(yf[0:self.n/2])

            yd = []
            for j in range(10):
                ys = []
                for k in range(int(((2.0**(j+1))-(2.0**(j)))*(self.n/1980))):
                    ys.append(yf[int((2.0**(j))*(self.n/1980))+k])
                ysm = np.mean(ys)*100
                if not ysm > 0:
                    yd.append(0)
                else:
                    yd.append(20*log10(ysm))
                if yd[j] > self.ym[j]:
                    self.ym[j] = yd[j]

        # ros node setup
        self.pub = [rospy.Publisher] * 10
        for i in range(10):
            self.pub[i] = rospy.Publisher('/height/'+str(i+1), UInt16MultiArray, queue_size=10)

    def start(self):
        sounddevice.play(self.y, self.fs)
        self.starttime = time()
        self.currblock = 0
        # self.fftblock = 0

        rospy.Subscriber('/feedback', UInt8, self.callback, queue_size=10)

        self.output = [UInt16MultiArray() for j in range(10)]

    def fft(self, i):
        self.currblock = i/self.n
        
        # # fft to block by block instead of per sample if can't afford computational time
        # # doesn't remove audio glitches though
        # if self.currblock == self.fftblock:
        #     return

        # fft
        yt = self.y[i:i+self.n]
        yf = fft(yt)
        ya = 2.0/self.n * np.abs(yf[0:self.n/2])

        # separate into 10 bands
        yd = []
        for j in range(10):
            ys = []
            for k in range(int(((2.0**(j+1))-(2.0**(j)))*(self.n/1980))):
                ys.append(ya[int((2.0**(j))*(self.n/1980))+k])
            ysm = np.mean(ys)*100
            if not ysm > 0:
                yd.append(0)
            else:
                yd.append(20*log10(ysm))
		
		# shift history and insert
        height = [int(yd[j]/self.ym[j]*1023.0) for j in range(10)]
        for j in range(10):
            if height[j]<0:
                height[j] = 0
        for j in range(9,0,-1):
			self.output[j].data = self.output[j-1].data
        self.output[0].data = height

		# publish
        for j in range(10):
            self.pub[j].publish(self.output[j])

        # self.fftblock = self.currblock

    def stop(self):
        for j in range(9,0,-1):
			self.output[j].data = self.output[j-1].data
        self.output[0].data = [0 for j in range(10)]
        self.pub[j].publish(self.output[j])
    
    def callback(self, fdb):
        filter = [fdb.data[j] for j in range(10)]
        block = self.y[(self.currblock+1)*self.n:(self.currblock+2)*self.n]
        for j in range(10):
            if filter[j]==0:
                block = bandstop(block, self.xf[int((2.0**(j))*(self.n/1980))],self.xf[int((2.0**(j+1))*(self.n/1980))], self.fs)
        self.y[(self.currblock+1)*self.n:(self.currblock+2)*self.n] = block

def main():
    rospy.init_node('fourier')
    rospy.set_param('mode', 2)
    try:
        soundfilename = rospy.get_param('/feedback/soundfile')
    except (IOError, KeyError) as e:
        soundfilename = 'Spirited Away Always With Me - Piano.wav'
    Baa = Sound(filename=soundfilename, filepath='../../../../Pinteraction_audio_files')
    Baa.start()

    currtime = time()
    while (currtime < Baa.starttime + Baa.duration) and (not rospy.is_shutdown()):
        i = int((currtime - Baa.starttime)/Baa.T)
        rospy.loginfo("sample %d", i)
        rospy.loginfo(currtime)
        Baa.fft(i)
        currtime = time()

    if not rospy.is_shutdown():
        for j in range(10):
            Baa.stop()
            sleep(0.4)
        rospy.loginfo("finished playing song")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('program stopped')
