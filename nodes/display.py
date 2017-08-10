#!/usr/bin/env python

## Generates 10 out-of-phase sine waves to be output as pin heights

import rospy
from std_msgs.msg import UInt16MultiArray
from math import *
from time import time, sleep

pub = [rospy.Publisher] * 10
for i in range(10):
    pub[i] = rospy.Publisher('/height/'+str(i+1), UInt16MultiArray, queue_size=1)
rospy.init_node('wave')

output = []
for i in range(10):
    output.append(UInt16MultiArray())
    # output[i].layout.dim = [MultiArrayDimension]
    # output[i].layout.dim[0].label = str(i+1)
    # output[i].layout.dim[0].size = 10
    # output[i].layout.dim[0].stride = 1
    output[i].layout.data_offset = 0
    output[i].data = [0] * 10

def main():
    rospy.set_param('mode', 1)
    starttime = time()

    while not rospy.is_shutdown():
        currtime = time()
        i = (currtime-starttime)%10
        
        for j in range(10):
            for k in range(10):
                # y = 5*sin(pi/5*(10*i+2*j+k))+5                         # sine waves
                y = 5*sin(pi/5*(10*i-abs(5-j)-abs(5-k)))+5             # ripples
                output[j].data[k] = int(y/10.0*1023)
            pub[j].publish(output[j])
        sleep(0.05)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('program stopped')