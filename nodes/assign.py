#!/usr/bin/env python

## Performs fourier transform on music file
## Outputs dB values on 10 frequency bands
## Sends individual pin height to arduino

import matplotlib.pyplot as plt
import numpy as np
from scipy.fftpack import fft
from scipy.io import wavfile
import pygame
from os import path
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from math import ceil, log10
from time import time

rospy.loginfo("reading sound file...")
soundfile = 'Baa-Baa-Black-Sheep.wav'              # change accordingly
script_dir = path.dirname(__file__)
rel_path = '../../../../../Downloads'              # change accordingly
soundfile = path.join(script_dir, rel_path, soundfile)
fs, y = wavfile.read(soundfile)

# average two channels if stereo
if hasattr(y[0], "__len__"):
    y = [int(np.mean(y[i])) for i in range(len(y))]

N = len(y)                      # no of samples
T = 1.0 / fs                    # time btw samples
x = np.linspace(0.0, N*T, N)    # create x axis of sound file
n = 2048                        # no of samples for fft
duration = N/fs                 # duration of song

# to determine ym of each band, run ft on whole music first
rospy.loginfo("calculating maximum...")
ym = [0.0] * 10
arr = np.linspace(0,N,N/n)
arr = [int(i) for i in arr]
arr.pop()
for i in arr:
    yt = y[i:i+n]
    yf = fft(yt)
    yf = 2.0/n * np.abs(yf[0:n/2])
    yd = []
    for j in range(10):
        ys = 0.0
        for k in range(n/20):
            ys = ys + yf[j*(n/20)+k]
        yd.append(20*log10(ys))
        if yd[j] > ym[j]:
            ym[j] = yd[j]

def play():
    pub1 = rospy.Publisher('/fourier', Float64MultiArray, queue_size=10)
    pub2 = rospy.Publisher('/hist2', Float64MultiArray, queue_size=10)
    rospy.init_node('ft_pub', anonymous=True)

    rospy.loginfo("playing sound file")
    pygame.init()
    pygame.mixer.music.load(soundfile)
    starttime = time()
    pygame.mixer.music.play()

    plt.ion()
    ax = plt.gca()
    currtime = time()

    height = [0.0] * 10
    output = Float64MultiArray()
    output.layout.dim = [MultiArrayDimension] * 2
    output.layout.dim[0].label = "fourier"
    output.layout.dim[0].size = 10
    output.layout.dim[0].stride = 100

    output2 = Float64MultiArray()

    while (currtime < starttime + duration) and (not rospy.is_shutdown()):
        i = int((currtime-starttime)/T)
        rospy.loginfo("sample no %d", i)
        xf = np.linspace(0.0, 1.0/(2.0*T), n/2)         # create x axis of fft
        yt = y[i:i+n]                                   # take n samples, starting from currtime's sample
        yf = fft(yt)                                    # fft n samples
        ya = 2.0/n * np.abs(yf[0:n/2])                  # taking absolute of cplx fft
        
        # separate into 10 bands
        xd = []
        yd = []
        for j in range(10):
            xd.append(np.mean(xf[j*(n/20):(j+1)*(n/20)]))   # get x value as mean of freq band
            ys = 0.0
            for k in range(n/20):
                ys = ys + ya[j*(n/20)+k]                    # get y value as summation of amplitudes in band 
            yd.append(20*log10(ys))                         # get dB value
        
        output2.data = height
        height = [yd[i]/ym[i]*10.0 for i in range(10)]
        rospy.loginfo(height)
        
        ax.cla()
        ax.set_ylim([0,10])
        ax.bar(xd,height)
        plt.draw()

        output.data = height
        pub1.publish(output)
        pub2.publish(output2)
        currtime = time()

    if not rospy.is_shutdown():
        output.data = [0.0] * 10
        output2.data = [0.0] * 10
        pub1.publish(output)
        pub2.publish(output2)
        rospy.loginfo("finished playing song")

if __name__ == '__main__':
    try:
        play()
    except rospy.ROSInterruptException:
        pass