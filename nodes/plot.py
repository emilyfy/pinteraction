#!/usr/bin/env python

## Receives fft data via /height/1 subscription
## Plots the height real time

import rospy
from std_msgs.msg import UInt16MultiArray
import matplotlib.pyplot as plt
import numpy as np

yd = []
update = False

def pltCb(height):
    global yd, update
    yd = [height.data[i]/1023.0*10.0 for i in range(10)]
    update = True

def main():
    global yd, update
    n = 2048
    T = 1.0/44100                     # assume, lazy to read param zzz
    xf = np.linspace(0.0, 1.0/(2.0*T), n/2)
    xd = []
    for i in range(10):
        xd.append(np.mean(xf[int((2.0**(i))*(n/1980)):int((2.0**(i+1))*(n/1980))]))
    
    rospy.init_node('plot')
    rospy.Subscriber('/height/1', UInt16MultiArray, pltCb, queue_size=10)
    
    while not update:
        continue

    plt.ion()
    ax = plt.gca()
    
    while not rospy.is_shutdown():
        if not update:
            continue
        ax.cla()
        ax.set_ylim([0,10])
        ax.bar(xd,yd)
        plt.draw()
        update = False

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("program stopped")