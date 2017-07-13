import matplotlib.pyplot as plt
import numpy as np
from scipy.fftpack import fft
from scipy.io import wavfile
from scipy import signal
import pygame
from os import path
from math import ceil, log10
from time import time, sleep

print "reading sound file..."
soundfile = 'Spirited Away Always With Me - Piano.wav'
script_dir = path.dirname(__file__)
rel_path = '/home/kjchen/Downloads'
soundfile = path.join(script_dir, rel_path, soundfile)
fs, y = wavfile.read(soundfile)

# average two channels if stereo
if hasattr(y[0], "__len__"):
    y = [int(np.mean(y[i])) for i in range(len(y))]

# normalize file
'''def bit(x):
    return {
        'float32': 32,
        'int32'  : 32,
        'int16'  : 16,
        'uint8'  : 8,
    }.get(x, 8)

b = bit(y.dtype.name)
y = [(i/2**b)*2-1 for i in y]'''

N = len(y)                      # no of samples
T = 1.0 / fs                    # time btw samples
x = np.linspace(0.0, N*T, N)    # create x axis of sound file
n = 2048                        # no of samples for fft
duration = N/fs                 # duration of song

# to determine ym, run ft on whole music first
# ym is amplitude, not logged!
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
        if yd[j] > ym[j]:               # change maximum if greater
            ym[j] = yd[j]
print "ym =", ym

print "playing sound file"
pygame.init()
pygame.mixer.music.load(soundfile)
starttime = time()
pygame.mixer.music.play()

plt.ion()
ax = plt.gca()
currtime = time()

while currtime < starttime + duration:
    i = int((currtime-starttime)/T)
    # i = 302395
    print "sample no", i
    xf = np.linspace(0.0, 1.0/(2.0*T), n/2)         # create x axis of fft
    yt = y[i:i+n]                                   # take n samples, starting from currtime's sample
    yf = fft(yt)                                    # fft n samples
    ya = 2.0/n * np.abs(yf[0:n/2])                  # taking absolute of cplx fft
    # must add amplitudes before log cos aft log amplitudes < 1 give -ve dB
    
    # separate into 10 bands
    xd = []
    yd = []
    for j in range(10):
        xd.append(np.mean(xf[j*(n/20):(j+1)*(n/20)]))   # get x value as mean of freq band
        ys = 0.0
        for k in range(n/20):
            ys = ys + ya[j*(n/20)+k]                    # get y value as summation of amplitudes in band
        yd.append(20*log10(ys))                         # get dB value
    
    # yd = [yd[i]/ym[i]*10000000 for i in range(10)]      # as ratio of max, *10^7 so that log max is 7 (leave 3cm pin height for feedback)
    height = [yd[i]/ym[i]*10.0 for i in range(10)]

    ax.cla()
    ax.set_ylim([0,10])
    ax.bar(xd,height)
    plt.draw()

    currtime = time()

print "finished playing song"