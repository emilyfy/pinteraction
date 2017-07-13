from obspy.signal.filter import bandstop
import sounddevice as sd
from scipy.io import wavfile
from time import time
import matplotlib.pyplot as plt

fs, y = wavfile.read('Alan Walker - Faded.wav')

arr = bandstop(y[fs*100:fs*200], 7000,22000, fs)
plt.plot(y[fs*100:fs*20], 'r', arr, 'b')
plt.show()
y[fs*100:fs*200] = arr

sd.play(y, fs)
starttime = time()

a = 120
while time() - starttime < 200:
    a = a+5
    if time() - starttime < 100:
        print "original"
    else:
        print "low pass"
