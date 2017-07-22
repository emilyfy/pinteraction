# Pinteraction

A dynamic array of 100 actuator pins in symphonic motion that responds interactively to audio and visual input
100 actuators controlled by 10 Arduino Mega (one each row) in serial communication with a Raspberry Pi running ROS
Topics /height/1 to /height/10 published by nodes in Raspberry Pi, subscribed to by each Mega for the respective row's pin heights

Three different modes of operation for the pins:
1) Demonstration, for manual row and column input
2) Audio visualizer & equalizer
3) Hand interaction


## 1) Demonstration
```bash
  roslaunch pinteraction demo.launch
```
runs two nodes, one for interfacing with the user and one for actuation
topic /command used to communicate between them
user inputs row number, column number, height to actuate to and duration of actuation
keeps prompting for input until user chooses to exit

## 2) Audio
```bash
  roslaunch pinteraction audio.launch
  roslaunch pinteraction audio.launch soundfile:="filename.wav"
```
runs the node feedback.py which performs fourier transform on the audio file and outputs the result in dB as /height/1
/height/2 to /height/10 correspond to shifts in time
reads equalizer feedback from the first row's Arduino Mega and performs inverse fourier transform real time on the audio array
terminates when the audio finished playing
node plot.py plots the height output real time on the screen
change the audio file to be played by setting argument soundfile in calling roslaunch. The file has to saved inside '~/Pinteraction_audio_files'. By default Spirited Away Always With Me is played

## 3) Hand interaction
```bash
  roslaunch pinteraction vision.launch video_device:="/dev/video0
```
runs the node vision.cpp which uses OpenCV to get the position of the hand, which must be wearing a red glove
subscribes to /distance published by the first row's Mega, which gives the distance between the hand and the pins directly below
outputs the pin heights, setting them such that they are always 11cm below the hand
change the camera device to the one to the USB camera by setting the argument video_device in calling roslaunch. Default is /dev/video1