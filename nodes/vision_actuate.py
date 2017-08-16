#!/usr/bin/env python

## Subscribes to topic /center for the center of red object
## Actuates the pins corresponding to center to move up

import rospy
from std_msgs.msg import UInt16, UInt16MultiArray
from time import time

# constants
img_width = rospy.get_param('/usb_cam/image_width')
img_height = rospy.get_param('/usb_cam/image_width')
clearance = 5.0

# global variables
height = 5.0
row = []
col = []
starttime = []
updatetime = time()

pub = [rospy.Publisher] * 10
for i in range(10):
    pub[i] = rospy.Publisher('/height/'+str(i+1), UInt16MultiArray, queue_size=10)

output = []
for i in range(10):
    output.append(UInt16MultiArray())
    output[i].layout.data_offset = 0
    output[i].data = [0] * 10

def targetCb(obj):
    global img_width, img_height, row, col, updatetime, starttime
    updatetime = time()

    tl_row = int(10.0 * obj.data[1] / img_height)
    tl_col = int(10.0 * obj.data[0] / img_width)
    br_row = int(10.0 * obj.data[3] / img_height)
    br_col = int(10.0 * obj.data[2] / img_width)

    rows = range(tl_row, br_row+1)
    cols = range(tl_col, br_col+1)
    
    input_rows = []
    input_cols = []
    for i in rows:
        for j in cols:
            input_rows.append(i)
            input_cols.append(j)
    
    for i in range(len(input_rows)):
        input_row = input_rows[i]
        input_col = input_cols[i]

        if input_row in row:
            j = 0
            index = [i for i, x in enumerate(row) if x == input_row]
            while j < len(index):
                if col[index[j]] == input_col:
                    del starttime[index[j]]
                    del row[index[j]]
                    del col[index[j]]
                j += 1
                index = [i for i, x in enumerate(row) if x == input_row]

        row.append(input_row)
        col.append(input_col)
        starttime.append(updatetime)

def distCb(dst):
    global clearance, height, output, row, col
    x = dst.data
    distance = -3.09*0.0000001*x*x*x + 4.124*0.0001*x*x-1.95*0.1*x+37.70
    if len(row):
        curr = output[row[len(row)-1]].data[col[len(col)-1]]/1023.0*10.0
    else:
        curr = 0.0
    height = curr + distance - clearance
    height = 0.0 if height<0 else height
    height = 10.0 if height>10 else height

rospy.Subscriber('/center', UInt16MultiArray, targetCb, queue_size=10)
# rospy.Subscriber('/distance', UInt16, distCb, queue_size=10)

def main():
    global height, row, col, starttime, updatetime, pub, output
    rospy.init_node('vision_actuate')
    rate = rospy.Rate(10)
    rospy.set_param('mode', 3)

    while not rospy.is_shutdown():
        currtime = time()

        try:
            for i in range(len(starttime)):
                if currtime - starttime[i] > 0.1:
                    del starttime[i]
                    del row[i]
                    del col[i]
        except IndexError:
            pass

        for i in range(10):
            for j in range(10):
                output[i].data[j] = 0
        
        try:
            for i in range(len(row)):
                if row[i]>0:
                    if col[i]>0:
                        output[row[i]-1].data[col[i]-1] = int(height/10.0*1023)
                    output[row[i]-1].data[col[i]] = int(height/10.0*1023)
                    if col[i]<9:
                        output[row[i]-1].data[col[i]+1] = int(height/10.0*1023)
                
                if row[i]<9:
                    if col[i]>0:
                        output[row[i]+1].data[col[i]-1] = int(height/10.0*1023)
                    output[row[i]+1].data[col[i]] = int(height/10.0*1023)
                    if col[i]<9:
                        output[row[i]+1].data[col[i]+1] = int(height/10.0*1023)
                
                output[row[i]].data[col[i]] = int(height/10.0*1023)
                if col[i]>0:
                    output[row[i]].data[col[i]-1] = int(height/10.0*1023)
                if col<9:
                    output[row[i]].data[col[i]+1] = int(height/10.0*1023)
        except IndexError:
            pass

        for i in range(10):
            pub[i].publish(output[i])
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('program stopped')