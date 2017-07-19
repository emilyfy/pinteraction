#!/usr/bin/env python

## Subscribes to topic /command for the actuation data
## Publishes to the corresponding arduino the actuation data

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension
from multiprocessing import Process, Value
from time import time

pub = [rospy.Publisher] * 10
for i in range(10):
    pub[i] = rospy.Publisher('/height/'+str(i+1), UInt16MultiArray, queue_size=10)
rospy.init_node('demo')

output = [UInt16MultiArray() for i in range(10)]
for i in range(10):
    output[i].layout.dim = [MultiArrayDimension]
    output[i].layout.dim[0].label = str(i+1)
    output[i].layout.dim[0].size = 10
    output[i].layout.dim[0].stride = 1
    output[i].layout.data_offset = 0
    output[i].data = [0] * 10

starttime = []
duration = []
row = []
col = []
stop = False

def callback(msg):
    global starttime, duration, row, col, stop

    if len(msg.buttons)==0:
        stop = True
        return

    input_row = msg.buttons[0]
    input_col = msg.buttons[1]
    input_height = msg.axes[0]
    input_duration = msg.axes[1]

    if input_row in row:
        indices = [i for i, x in enumerate(row) if x == input_row]
        for index in indices:
            if col[index] == i_col:
                del starttime[index]
                del duration[index]
                del row[index]
                del col[index]
            
    output[input_row].data[input_col] = int(input_height/10.0*1023)
    pub[input_row].publish(output[input_row])
    starttime.append(time())
    duration.append(input_duration)
    row.append(input_row)
    col.append(input_col)

rospy.Subscriber('/command', Joy, callback, queue_size=20)

def main():
    rospy.set_param('mode', 1)
    global starttime, duration, row, col

    while not rospy.is_shutdown() and not stop:
        currtime = time()
        for i in range(len(starttime)):
            if starttime[i] + duration[i] < currtime:
                output[row[i]].data[col[i]] = 0
                pub[row[i]].publish(output[row[i]])
                del starttime[i]
                del duration[i]
                del row[i]
                del col[i]

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('program stopped')