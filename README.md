# Pinteraction

<br>A dynamic array of 100 actuator pins in symphonic motion that responds interactively to audio and visual input
<br>100 actuators controlled by 10 Arduino Mega (one each row) in serial communication with a Raspberry Pi running ROS
<br>Topics /height/1 to /height/10 published by nodes in Raspberry Pi, subscribed to by each Mega for the respective row's pin heights
<br>
<br>Three different modes of operation for the pins:
<br>1) Display coordinated motion
<br>2) Audio visualizer & equalizer
<br>3) Visual hand interaction
<br>
<br>
## 1) Display
```bash
  roslaunch pinteraction display.launch
```
<br>runs the node display.py which actuates the pins to generate waves based on pre-programmed formulaes
<br>
## 2) Audio
```bash
  roslaunch pinteraction audio.launch
  roslaunch pinteraction audio.launch soundfile:="filename.wav"
```
<br>runs the node fourier.py which performs fourier transform on the audio file and outputs the result in dB as /height/1
<br>/height/2 to /height/10 correspond to shifts in time
<br>reads equalizer feedback from the first row's Arduino Mega and performs inverse fourier transform real time on the audio array
<br>terminates when the audio finished playing
<br>node plot.py plots the height output real time on the screen
<br>change the audio file to be played by setting argument soundfile in calling roslaunch. The file has to saved inside '~/Pinteraction_audio_files'. By default Spirited Away Always With Me is played
<br>
## 3) Visual
```bash
  roslaunch pinteraction visual.launch
  roslaunch pinteraction visual.launch video_device:="/dev/video0"
```
<br>runs the nodes vision_target.cpp and vision_actuate.cpp which actuates the pins below the user's hand to 5cm below the hand
<br>vision_target uses OpenCV to get the position of the hand, which must be wearing a red glove
<br>vision_actuate actuates the pin directly below the center of the hand and the pins surrounding it
<br>subscribes to /distance published by the first_row_upload Mega, which gives the distance between the hand and the pins directly below
<br>outputs the pin heights, setting them such that they are always 5cm below the hand
<br>change the camera device to the one to the USB camera by setting the argument video_device in calling roslaunch. Default is /dev/video1
<br>
## Required Packages
<br>Python: scipy, numpy, matplotlib
<br>OpenCV
<br>ROS: usb_cam (https://github.com/bosch-ros-pkg/usb_cam.git)
<br>
### Setting udev rules
<br>set the udev rules in /etc/udev/rules.d to that in archived codes/99-usb-serial.rules