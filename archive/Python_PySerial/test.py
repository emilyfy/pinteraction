# A simple test to run pyserial
# 1) Download the archive from http://pypi.python.org/pypi/pyserial. 
# 2) Unpack the archive, enter the pyserial-x.y directory and run:
#   python setup.py install
#   For Python 3.x:
#   python3 setup.py install
#
# upload the accompanying arduino code in the same folder as this file to an arduino
# the code will echo whatever is send to it via usb and this program will print it out and then ends
#
# to run the test, go to the folder with this file and type:
#   sudo python test.py

import serial

arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

if arduino.is_open:
    print("Start")

    received=""
    while received=="":
        arduino.write(chr(5)) # chr(5) sends the literal value 5. if you just enter '5' it will send the ascii value of 5
        received = arduino.read(10) #reads up to 10bytes at one go. has a timeout of 1 sec (specified above)
        print("Received: " + received) #prints the received value

print("Test Ended")
