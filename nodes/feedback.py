#!/usr/bin/env python

## Performs real time fourier transform on music file
## Outputs dB values on 10 frequency bands relative to 10.0 on topic /height/1
## Shifts its history with each instance, up to 10 last instance, on topic /height/2 height/3 etc
## Receives feedback data on topic /feedback
## Applies real time filter on played music based on feedback

# signal processing
import matplotlib.pyplot as plt
import numpy as np
from scipy.fftpack import fft

# audio playing
from scipy.io import wavfile
from os import path
import sounddevice
from obspy.signal.filter import bandstop, bandpass

# ros
import rospy
from std_msgs.msg import UInt16, UInt16MultiArray, MultiArrayDimension

# calculations
from math import ceil, log10
from time import time  

class Sound(object):
    n = 2048             # no of samples in each block
    
    def __init__(self, filename, filepath):
        self.soundfile = path.join(path.dirname(__file__), filepath, filename)
        self.fs, self.y = wavfile.read(self.soundfile)
        
        # average two channels if stereo
        if hasattr(self.y[0], "__len__"):
            self.y = [int(np.mean(self.y[i])) for i in range(len(self.y))]
        
        self.N = len(self.y)                                     # no of samples
        self.T = 1.0 / self.fs                                   # time btw samples
        self.x = np.linspace(0.0, self.N*self.T, self.N)         # create x axis of sound file
        self.duration = self.N/self.fs                           # duration of song
        self.xf = np.linspace(0.0, 1.0/(2.0*self.T), self.n/2)   # x axis for fft
        
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
                ys = 0.0
                for k in range(self.n/20):
                    ys = ys + yf[j*(self.n/20)+k]
                yd.append(20*log10(ys))
                if yd[j] > self.ym[j]:
                    self.ym[j] = yd[j]
        
		# ros node setup
		self.pub = [rospy.Publisher] * 10
		for i in range(10):
			self.pub[i] = rospy.Publisher('/height/'+str(i+1), UInt16MultiArray, queue_size=10)
		rospy.init_node('fourier')

    def start(self):
        sounddevice.play(self.y, self.fs)
        self.starttime = time()
        self.currblock = 0

        rospy.Subscriber('/feedback', UInt16MultiArray, self.callback, queue_size=10)

        self.output = [UInt16MultiArray() for j in range(10)]
        for j in range(10):
            self.output[j].layout.dim = [MultiArrayDimension]
            self.output[j].layout.dim[0].label = str(j+1)
            self.output[j].layout.dim[0].size = 10
            self.output[j].layout.dim[0].stride = 1
            self.output[j].layout.data_offset = 0

        plt.ion()
        self.ax = plt.gca()

    def fft(self, i):
        self.currblock = i/self.n

        # consider fft to block by block instead of per sample, 2048 onwards
        # if can't afford computational time

        # fft
        yt = self.y[i:i+self.n]
        yf = fft(yt)
        ya = 2.0/self.n * np.abs(yf[0:self.n/2])

        # separate into 10 bands
        xd = []
        yd = []
        for j in range(10):
            xd.append(np.mean(self.xf[j*(self.n/20):(j+1)*(self.n/20)]))
            ys = 0.0
            for k in range(self.n/20):
                ys = ys + ya[j*(self.n/20)+k]
            yd.append(20*log10(ys))
		
		# shift history and insert
        height = [int(yd[j]/self.ym[j]*1023.0) for j in range(10)]
        for j in range(9,0,-1):
			self.output[j].data = self.output[j-1].data
        self.output[0].data = height

		# publish
        for j in range(10):
            self.pub[j].publish(self.output[j])
        
        # plot
        self.ax.cla()
        self.ax.set_ylim([0,10])
        self.ax.bar(xd,height)
        plt.draw()

    def stop(self):
        for j in range(10):
            self.output[j].data = [0] * 10
            self.pub[j].publish(self.output[j])
    
    def callback(self, fdb):
        filter = [(fdb.data[i]/1023.0-0.5)*10 for j in range(len(fdb.data))]         # -5dB to +5dB
        ori = y[(self.currblock+1)*self.n:(self.currblock+2)*self.n]
        arr = [ ori ] * len(filter)
        for j in range(len(filter)):
            if filter[j] < 0:
                arr[j] = bandstop(ori, self.xf[j*(self.n/2/len(filter)), (j+1)*(self.n/2/len(filter))], fs)
            elif filter[j] > 0:
                arr[j] = bandpass(ori, self.xf[j*(self.n/20), (j+1)*(self.n/20)], fs)
            arr[j] = # apply dB accordingly . e.g. ....*ori + ....*arr[j]

def main():
    Baa = Sound(filename='Baa-Baa-Black-Sheep.wav', filepath='../../../../../Downloads')
    Baa.start()

    currtime = time()
    while (currtime < Baa.starttime + Baa.duration) and (not rospy.is_shutdown()):
        i = int((currtime - Baa.starttime)/Baa.T)
        # rospy.loginfo("sample %d", i)
        # rospy.loginfo(currtime)
        Baa.fft(i)
        currtime = time()

    if not rospy.is_shutdown():
        Baa.stop()
        rospy.loginfo("finished playing song")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('program stopped')