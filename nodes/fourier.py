#!/usr/bin/env python

## Performs real time fourier transform on music file
## Outputs dB values on 10 frequency bands relative to 10.0 on topic /height/1
## Shifts its history with each instance, up to 10 last instance, on topic /height/2 height/3 etc
## Receives feedback data on topic /feedback
## Applies real time filter on played music based on feedback

# signal processing
import numpy as np
from scipy.fftpack import fft, ifft, rfft, irfft

# audio playing
from scipy.io import wavfile
from os import path
import sounddevice

# ros
import rospy
from std_msgs.msg import UInt16, UInt16MultiArray
from pinteraction.msg import BoolMultiArray

# calculations
from math import ceil, log10
from time import time

# global variables
currblock = 0
editedblock = 0
y = []
y1 = y
n = 2048
xf = []
off = [False, False, False, False, False, False, False, False, False, False]
    
def callback(fdb):
    global currblock, editedblock, y, y1, n, xf, off

    if editedblock > currblock+10:
        return

    for j in range(20):
        block = y[(editedblock+1+j)*n:(editedblock+2+j)*n]
        filteredbf = np.asarray(rfft(block))
        for k in range(10):
            if not fdb.data[k]:
                filteredbf[np.argwhere((xf>=xf[int(2.0**(k))-1]) & (xf<xf[int(2.0**(k+1))-1]))] = 0
                off[k] = True
            else:
                off[k] = False
        filteredblock = irfft(filteredbf)
        filteredblock = [int(filteredblock[k]) for k in range(len(filteredblock))]

        try:
            l = 0
            for k in range((editedblock+1+j)*n,(editedblock+2+j)*n):
                y1[k] = [filteredblock[l], filteredblock[l]]
                # y[k] = filteredblock[l]
                l +=1
        except NameError:
            y[(editedblock+1+j)*n:(editedblock+2+j)*n] = filteredblock
    
    editedblock = editedblock+20

def main():
    global currblock, editedblock, y, y1, n, xf, off

    rospy.init_node('fourier')
    
    try:
        soundfilename = rospy.get_param('/fourier/soundfile')
    except (IOError, KeyError) as e:
        soundfilename = 'Spirited Away Always With Me - Piano.wav'
    soundfile = path.join(path.dirname(__file__), '../../../../Pinteraction_audio_files', soundfilename)
    fs,y = wavfile.read(soundfile)

    # average two channels if stereo
    if hasattr(y[0], "__len__"):
        # y = [int(np.mean(y[i])) for i in range(len(y))]
        y1 = y
        y = [y[i][0] for i in range(len(y))]
    
    N = len(y)                                     # no of samples
    T = 1.0 / fs                                   # time btw samples
    x = np.linspace(0.0, N*T, N)                   # create x axis of sound file
    duration = N/fs                                # duration of song
    xf = np.linspace(0.0, 1.0/(2.0*T), n/2)        # x axis for fft
    n = 2048                                       # no of samples in each block

    # make sure data types are of float64, float32, int32, int16, int8 or uint8
    dt = np.dtype(y[0])
    acctype = [np.dtype('float64'), np.dtype('float32'), np.dtype('int32'), np.dtype('int16'), np.dtype('int8'), np.dtype('uint8')]
    if dt not in acctype:
        y = np.asarray(y, dtype=np.int16)
    
    # to determine ym of each band, run ft on whole music first
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
            ys = []
            for k in range(int((2.0**(j+1))-(2.0**(j)))):
                ys.append(yf[int(2.0**(j))-1+k])
            ysm = np.mean(ys)*100
            if not ysm > 0:
                yd.append(0)
            else:
                yd.append(20*log10(ysm))
            if yd[j] > ym[j]:
                ym[j] = yd[j]

    # ros node setup
    pub = [rospy.Publisher] * 10
    for i in range(10):
        pub[i] = rospy.Publisher('/height/'+str(i+1), UInt16MultiArray, queue_size=10)
    
    # start playing song
    sounddevice.play(y1, fs)
    starttime = time()
    currblock = 0
    editedblock = 0
    fftblock = 0
    rospy.set_param('mode', 2)

    rospy.Subscriber('/feedback', BoolMultiArray, callback, queue_size=10)

    output = []
    for j in range(10):
        output.append(UInt16MultiArray())
        output[j].layout.data_offset = 0

    currtime = time()
    while (currtime < starttime + duration) and (not rospy.is_shutdown()):
        i = int((currtime - starttime)/T)
        # print y1[i]
        currblock = i/n
        
        # fft
        if currblock > fftblock:
            yt = y[i:i+n]
            yf = fft(yt)
            ya = 2.0/n * np.abs(yf[0:n/2])

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
            
            # shift history and insert
            height = [int(yd[j]/ym[j]*1023.0) for j in range(10)]
            for j in range(10):
                height[j] = 0 if off[j] or height[j]<0 else height[j]
            for j in range(9,0,-1):
                output[j].data = output[j-1].data
            output[0].data = height

            # publish
            for j in range(10):
                pub[j].publish(output[j])

            fftblock = currblock

        currtime = time()

    if not rospy.is_shutdown():
        rospy.loginfo("finished playing song")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('program stopped')
