# RxROS
RxROS a domain specific language for programming robots.

## Introduction
This code is part of my Master Thesis at ITU, Copenhagen Denmark.
As the title indicates is the goal to produce a DSL for programming robots.
The DSL will be based on Reactive C++ and ROS.
<p> 
Part of the thesis is to build a Lego Mindstorms NXT robot.
The robot will be used to gain experiences with programming robots.
<p>
Please feel free to comment and use the code as you like.
My hope is that it will bring inspriation and fun to everyone.

## Setup and installation
In order to make use of this software you must
install the following software on your computer:

1. Ubuntu Bionic (18.04)<br>
You can download and image of the Ubuntu Bionic Linux distribution at
<br>
https://www.ubuntu.com/#download

2. ROS Melodic Morenia<br>
Installation instruction of how to install ROS Melodic Morenia can be found at
<br>
http://wiki.ros.org/melodic/Installation/Ubuntu
<br>
I did more specific execute the following commands from a terminal

```
 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' 
 sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 
 sudo apt-get update 
 sudo apt-get install ros-melodic-desktop-full 
 sudo rosdep init 
 rosdep update 
 echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc 
 sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential 
```

3. Reactive C++
