#!/usr/bin/env python

## Prompts user for row, column number, height and duration to move
## Publishes to the topic /command

import rospy
from sensor_msgs.msg import Joy

def main():
    print "Hello! Welcome to the Pinteraction demo"
    print "This program allows you to actuate a pin of your choice to any height and for any duration"

    pub = rospy.Publisher('/command', Joy, queue_size=20)
    rospy.init_node('demo_interface')
    msg = Joy()

    while not rospy.is_shutdown():
        input_row = raw_input("Please enter the row number (0 to 9): ")
        input_col = raw_input("Please enter the column number (0 to 9): ")
        input_height = raw_input("Please enter the height you would like to move to (in cm): ")
        input_duration = raw_input("Please enter the time duration for the pin to stay there (in seconds): ")

        try:
            input_row = int(input_row)
            input_col = int(input_col)
            input_height = float(input_height)
            input_duration = float(input_duration)
            if input_row>9 or input_col>9 or input_row<0 or input_col<0 or input_height>10 or input_height<0 or input_duration<0:
                print "Invalid data input. Please try again"
                continue
        except Exception, e:
            print "Invalid data input. Please try again"
            print str(e)
            continue

        msg.buttons = [input_row, input_col]
        msg.axes = [input_height, input_duration]
        pub.publish(msg)
        print "The pin has been actuated."

        input = raw_input("Enter x to exit the program, or any other key to actuate another pin. ")
        if input == 'x' or input == 'X':
            break
    
    if not rospy.is_shutdown():
        print "Thank you and we hope you have enjoyed that :)"
        msg.buttons = []
        msg.axes = []
        pub.publish(msg)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo('program stopped')
        msg.buttons = []
        msg.axes = []
        pub.publish(msg)