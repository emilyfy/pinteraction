#!/usr/bin/env python

## Prompts user for row, column number, height and duration to move
## Publishes to the corresponding arduino the actuation data

# is lock needed to ensure only one thread is editing at a time
# I'm not sure this sharing works. may need to try Manager() instead

import rospy
from std_msgs.msg import UInt16MultiArray, MultiArrayDimension
from multiprocessing import Process, Value
from time import time

print "Hello! Welcome to the Pinteraction demo"
print "This program allows you to actuate a pin of your choice to any height and for any duration"

input_row = Value('i', 10)
input_col = Value('i', 0)
input_height = Value('d', 0.0)
input_duration = Value('d', 0.0)
stop = Value('i', 0)

def actuate(input_row, input_col, input_height, input_duration, stop):
    
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

    while True:
        currtime = time()
        for i in range(len(starttime)):
            if starttime[i] + duration[i] < currtime:
                output[row[i]].data[col[i]] = 0
                pub[row[i]].publish(output[row[i]])
                del starttime[i]
                del duration[i]
                del row[i]
                del col[i]
        
        i_row = input_row.value
        i_col = input_col.value
        i_height = input_height.value
        i_duration = input_duration.value      

        if i_row == 10:
            continue
        else:
            if i_row in row:
                indices = [i for i, x in enumerate(row) if x == i_row]
                for index in indices:
                    if col[index] == i_col:
                        del starttime[index]
                        del duration[index]
                        del row[index]
                        del col[index]
            
            output[i_row].data[i_col] = int(i_height/10.0*1023)
            pub[i_row].publish(output[i_row])
            starttime.append(time())
            duration.append(i_duration)
            row.append(i_row)
            col.append(i_col)
            print "The pin has been actuated."
            input_row.value = 10
        
        stopp = stop.value
        print "stop", stopp
        if stopp == 1:
            print stopp
            break

def main():
    p = Process(target=actuate, args=(input_row, input_col, input_height, input_duration, stop))
    p.start()
    
    while True:
        input_row_t = raw_input("Please enter the row number (0 to 9): ")
        input_col_t = raw_input("Please enter the column number (0 to 9): ")
        input_height_t = raw_input("Please enter the height you would like to move to (in cm): ")
        input_duration_t = raw_input("Please enter the time duration for the pin to stay there (in seconds): ")

        try:
            input_row_t = int(input_row_t)
            input_col_t = int(input_col_t)
            input_height_t = float(input_height_t)
            input_duration_t = float(input_duration_t)
            if input_row_t > 9 or input_col_t >9 or input_row_t < 0 or input_col_t < 0 or input_height_t > 10 or input_height_t < 0 or input_duration_t < 0:
                print "Invalid data input. Please try again"
                continue
        except Exception, e:
            print "Invalid data input. Please try again"
            print str(e)
            continue

        input_row.value = input_row_t
        input_col.value = input_col_t
        input_height.value = input_height_t
        input_duration.value = input_duration_t

        while input_row.value != 10:
            continue

        input = raw_input("Enter x to exit the program, or any other key to actuate another pin. ")
        if input == 'x' or input == 'X':
            stop.value = 1
            break
    
    print "Thank you and we hope you have enjoyed that :)"
    p.join()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('program stopped')