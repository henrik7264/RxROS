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

### Ubuntu Bionic (18.04)

You can download an image of the Ubuntu Bionic Linux distribution at<br>
https://www.ubuntu.com/#download<br>
I found it useful in addition to add the following packages:

```
sudo apt-get install git doxygen graphviz-* meld cmake
sudo apt-get install emacs gtcreator qt5-*
sudo apt-get install tree gimp
```

### ROS Melodic Morenia

Installation instruction of how to install ROS Melodic Morenia can be found at<br>
http://wiki.ros.org/melodic/Installation/Ubuntu<br>
Execute the following commands to install ROS Melodic Morenia:

```
 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' 
 sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 
 sudo apt-get update 
 sudo apt-get install ros-melodic-desktop-full 
 sudo rosdep init 
 rosdep update 
 echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc 
 sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
 sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-keyboard
 sudo apt-get install ros-melodic-navigation
```

### NXT-ROS

NXT-ROS is a github project found at<br> 
https://github.com/NXT-ROS/nxt<br>
The installation process of NXT-ROS is a bit complicated, but here is the procedure I got working:<br>
Start to install the nxt-python package. NXT-ROS will not work without it!

```
sudo apt-get install libnxt nxt-python 
```

Create then a lego group and add the udev USB definition for the NXT controller:

```
sudo groupadd lego 
sudo usermod -a -G lego $(id -un) 
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="0694", GROUP="lego", MODE="0660"' > /tmp/70-lego.rules && sudo mv /tmp/70-lego.rules /etc/udev/rules.d/70-lego.rules 
reboot
```

Finally, execute the following commands in a directory where you keep your ROS workspaces - any directory will work.

```
mkdir –p nxt 
cd  nxt 
git clone --recursive https://github.com/NXT-ROS/nxt.git src
catkin_make
cd ..
mkdir -p nxt_teleop/src
cd nxt_teleop/src
git clone --recursive https://github.com/NXT-ROS/nxt_teleop.git
cd ..
catkin_make
cd ..
```

### Reactive C++

Reactive C++ is also a github project. It can be found at<br>
https://github.com/ReactiveX/RxCpp<br>
To install RxCpp execute the following commands:

```
git clone --recursive  https://github.com/ReactiveX/RxCpp.git 
cd RxCpp 
mkdir -p projects/build 
cd projects/build 
cmake -G"Unix Makefiles" -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++ -DCMAKE_BUILD_TYPE=RelWithDebInfo -B. ../CMake 
make 
sudo make install 
```

After the installation has completed you can remove the RxCpp directory.
The needed C++ header files have been installed in /usr/include/rxcpp.

### RxROS

Finally, we have come to the RxROS project. To install RxROS do the following:
