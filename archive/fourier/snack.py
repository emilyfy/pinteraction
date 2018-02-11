from Tkinter import *
root = Tk()
root.tk.eval('info tclversion')

import tkSnack
tkSnack.initializeSnack(root)

mysound = tkSnack.Sound()
mysound.read('~/Downloads/Baa-Baa-Black-Sheep.wav')
mysound.play()

mysound.dBPowerSpectrum(start=,end=,winlength=16,fftlength=16)