# Pinteraction software manual
Note: lines starting with `$` means commands to key into the terminal

## Installing Ubuntu 14.04:
Download 64-bit PC (AMD64) desktop image ISO image from the [ubuntu website](http://releases.ubuntu.com/14.04/)  
Use a disc burner (Rufus is recommended) to burn the image to an empty thumbdrive  
Shut down computer, insert thumbdrive, go to BIOS and boot from USB  
Choose install Ubuntu and follow instructions  


## Installing ROS Indigo:
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
$ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
$ sudo apt-get update
$ sudo apt-get install ros-indigo-desktop-full
$ sudo rosdep init
$ rosdep update
$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
$ sudo apt-get install python-rosinstall
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
close terminal and open again.

Note: the catkin_ws directory is a catkin workspace to put ROS packages, you can choose to create the catkin_ws directory anywhere. It doesn't have to be inside `~/` and neither does it have to be named catkin_ws, but we'll assume so here.


## Getting the pinteraction package from Github:
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/emilyfy/pinteraction.git
```
if clone fails as destination path already exists and you want to replace everything,
```bash
$ rm -rf pinteraction
$ git clone https://github.com/emilyfy/pinteraction.git
$ cd ..
$ catkin_make
```

## Installing other required packages:
```bash
$ sudo apt-get install build-essential libssl-dev libffi-dev python-dev
$ sudo apt-get install python-scipy python-numpy python-matplotlib
$ sudo apt-get install ros-indigo-rosserial ros-indigo-usb-cam ros-indigo-image-proc ros-indigo-image-view

$ sudo apt-get install python-pip
$ python -m pip install setuptools --user
$ python -m pip install cffi --user
$ python -m pip install sounddevice --user
```

## Setting udev rules:
```bash
$ wget https://raw.githubusercontent.com/emilyfy/pinteraction/master/archive/99-usb-serial.rules
$ sudo mv 99-usb-serial.rules /etc/udev/rules.d/99-usb-serial.rules
```
enter password when prompted.

## Running the scripts:
### DISPLAY MODE
```bash
$ roslaunch pinteraction display.launch
```
to display ripple,
```bash
$ roslaunch pinteraction display.launch type:=ripple
```

### AUDIO MODE
```bash
$ roslaunch pinteraction audio.launch
```
to play another audio file,
```
$ roslaunch pinteraction audio.launch soundfile:="filename.wav"
```
audio file needs to be located inside the directory [audio_files](../audio_files) inside the package and needs to be a .wav file.  
If error occurs for that particular file, it may have been converted into an unsupported encoding.  
One way that definitely works is to find the audio on youtube and get its wav format from [here](https://www.youtubeto.com/WAV.php).

### VISUAL MODE
```bash
$ roslaunch pinteraction visual.launch
```
if USB cam is the second camera device connected to computer (eg in a laptop with a webcam)
```bash
$ roslaunch pinteraction visual.launch video_device:=/dev/video1
```
There are two versions of codes, one actuates all the pins from top left to bottom right of detected red contours and the other only the 9 pins around the center of the detection.  
If the pinteraction package is newly cloned from github, the current codes set the actuation on all the pins from top left to bottom right of contour by default (but the one last put inside the garage computer may be different).  
To actuate only 9 pins at all times (Jie Kai said this gave better results), run the command
```bash
$ roslaunch pinteraction visual.launch actuate_nine:=true
```
to change the default, edit the launch file and set default to true.


## For remote terminal:
Accessing the garage computer on a remote terminal doesn't allow us to open up graphical windows on the computer, hence we have to disable the plotting on audio mode or viewing the camera input on display mode. You can do this with:
```bash
$ roslaunch pinteraction audio.launch remote:=true
$ roslaunch pinteraction visual.launch remote:=true
```
If the computer getting the access has ROS installed and is connected to the same network as the garage computer, it can be setup to share the same ROS master (more on that below) and the actual image input from the camera can still be seen (though it'll be laggy)
```bash
$ rosrun image_view image_view image:="image_rect_color"
```
and the graph can be plotted provided the computer also has the package pinteraction and numpy & matplotlib installed:
```bash
$ rosrun pinteraction plot.py
```


## Regarding sharing a common ROS master:
Connect both setup computer (garage computer) and remote computer (your laptop) to the same wifi network  
Check the assigned IP on both computers by:
```bash
$ hostname -I
```
also check the setup computer's hostname, by:
```bash
$ hostname
```
The steps below will assume the setup computer IP as 192.168.1.100 the remote computer as 192.168.1.101, and the user account name on the setup computer is garagecrespion. Change the discrepancies accordingly.

On the remote computer,
```bash
$ cd /etc
$ sudo gedit hosts
```
add in the setup computer's IP and hostname to the list, save and close.

The steps above need only be done once, those below need to be done every time (make sure to check the IP address every time is a static IP isn't reserved).
```bash
$ ssh garagecrespion@192.168.1.100
```
enter password of garagecrespion account if prompted.  
The terminal prompt should change to that of garagecrespion account
```bash
$ export ROS_MASTER_URI=http://192.168.1.100:11311
$ export ROS_IP=192.168.1.100
```
keep this terminal open and run all the roslaunch... from here.  
Open another terminal on the remote computer
```bash
$ export ROS_MASTER_URI=http://192.168.1.100:11311
$ export ROS_IP=192.168.1.101
```
now the image_view or plot.py can be run in this terminal

Other than editing the /etc/hosts file, all the other steps need to be done every time a new terminal is opened.  
If possible, use the wi-fi adapter on the setup computer and set our own network so that we can reserve IP addresses for the setup and remote computers so it stays the same every time.  
This way, you don't need to check the setup computer's IP all the time and the `ROS_MASTER_URI` and `ROS_IP` environment variables can be set permanently (just put the commands inside `~/.bashrc`) instead of having to do them everytime.

### If you did the above, please read further on
The last time we tried running with the garage computer it was too slow to handle the fourier and opencv computation, resulting in laggy runs.  
Since you've set up the computer to share the ROS master with a remote laptop, it's now possible to run all the computation in your laptop and let the computer only run the programs that communicate with the arduino and camera.

First, make sure the above is done and that both computers have the pinteraction package and all its dependencies.  
In the garage computer, simply run
```bash
$ roslaunch arduino.launch
$ roslaunch camera.launch
```
the camera.launch is only needed for visual mode.

On your laptop, run the usual launch files with the without_arduino argument set to true.
```bash
$ roslaunch display.launch without_arduino:=true
$ roslaunch audio.launch without_arudino:=true
$ roslaunch visual.launch without_arduino:=true
```
Make sure to launch the arduino and camera (when needed) nodes first before launching the anything else on your laptop, else the nodes on your laptop might just die.  
Also make sure every terminal used has the environment variables `ROS_MASTER_URI` and `ROS_IP` set first, whether on the garage computer or your laptop.