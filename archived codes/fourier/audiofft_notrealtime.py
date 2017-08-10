import matplotlib.pyplot as plt
from scipy.fftpack import fft
from scipy.io import wavfile                        # get the api
fs, data = wavfile.read('casiopianoc5.wav')         # load the data
from math import log10


a = data.T[0:data.size]   

b=[(ele/2**8.)*2-1 for ele in a]                    # this is 8-bit track, b is now normalized on [-1,1)
c = fft(b)                                          # calculate fourier transform (complex numbers list)

d = len(c)/2                                        # you only need half of the fft list (real signal symmetry)


c = abs(c)

c = [log10(c[i]) for i in range(len(c))]    


plt.plot(c[:(d-1)],'r') 
print c[:(d-1)]
plt.show()

cmax = max(c)
print cmax