# Pinteraction
A dynamic array of 100 actuator pins in symphonic motion that responds interactively to audio and visual input.

<a href="http://www.youtube.com/watch?v=1_eq6VfAiIk" target="_blank"><center><img src="https://img.youtube.com/vi/1_eq6VfAiIk/0.jpg" 
alt="Pinteraction Project Video" width="480" height="360" /></center></a>

Three different modes of operation for the pins:
1. Display coordinated motion
2. Audio visualizer & equalizer
3. Visual hand interaction


## 1) Display
```bash
  roslaunch pinteraction display.launch type:="wave"
  roslaunch pinteraction display.launch type:="ripple"
```
runs the node display.py which actuates the pins to generate waves or ripples based on pre-programmed formulaes.
Choose the display type by passing in argument for type as "wave" or "ripple". By default wave is displayed.


## 2) Audio
```bash
  roslaunch pinteraction audio.launch
  roslaunch pinteraction audio.launch soundfile:="filename.wav"
```
runs the node fourier.py which
- performs fourier transform on the audio file and outputs the result in dB as `/height/1`. `/height/2` to `/height/10` correspond to shifts in time
- reads equalizer feedback from the first row's Arduino Mega and performs inverse fourier transform real time on the audio array.
- terminates when the audio finished playing

Node plot.py plots the height output real time on the screen.

Change the audio file to be played by setting argument soundfile in calling roslaunch. The file has to saved inside the folder `audio_files/`. By default *Spirited Away - Always With Me* is played.


## 3) Visual
```bash
  roslaunch pinteraction visual.launch
  roslaunch pinteraction visual.launch video_device:="/dev/video1"
```
runs the nodes vision_target.cpp and vision_actuate.cpp which actuates the pins below the user's hand to 5cm below the hand.  
`vision_target` uses OpenCV to get the position of the hand, which must be wearing a red glove
`vision_actuate` actuates the pins directly below the hand
- subscribes to /distance published by the first_row_upload Mega, which gives the distance between the hand and the pins directly below
- outputs the pin heights, setting them such that they are always 5cm below the hand

Change the camera device to the one to the USB camera by setting the argument video_device in calling roslaunch. Default is /dev/video0


## Required Packages
- Python: scipy, numpy, matplotlib
- OpenCV
- ROS: rosserial, usb_cam, image_view, image_proc


## Setting udev rules
Set the udev rules in /etc/udev/rules.d to that in archive/99-usb-serial.rules


# A garage project by

## Project members
* **Chen Kai Jun**  
* **Delia Ong**  
* **Emily Fatima**  
* **Foo Zi Hao**  
* **Li Yanling**

## Mentor
* **Er Jie Kai**

## Special thanks to
* **Garage@CresPion**
* **CN Yang Scholars Programme**
* **SPMS Making & Tinkering Lab**