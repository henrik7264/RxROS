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
sudo apt-get install emacs qtcreator qt5-*
sudo apt-get install tree gimp
sudo apt-get install liburdfdom-tools
sudo apt-get install libcanberra-gtk-module
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
sudo apt-get install libnxt python-nxt 
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
mkdir -p nxt_viz/src
cd nxt_viz/src
git clone --recursive https://github.com/NXT-ROS/nxt_viz.git
cd ..
catkin_make
cd ..
```

### Slamtec A2 RPLIDAR
Installation instruction of how to install Slamtec A2 RPLIDAR can be found at<br>
https://github.com/slamtec/rplidar_ros<br>
Execute the following commands to install Slamtec A2 RPLIDAR:

```
mkdir -p slamtec/src
cd slamtec
catkin_make
cd src
git clone https://github.com/Slamtec/rplidar_ros.git
cd ..
catkin_make
cd ..
```

After the installation a new package named rplidar_ros is created.<br>
When you perform a catkin_make the new package will be included in the project.<br>
Finally, ensure that the udev rules for rplidar is configured correctly:

```
sudo cp slamtec/src/rplidar_ros/scripts/rplidar.rules /etc/udev/rules.d/70-rplidar.rules
```

### Bosma Scheduler
The Bosma scheduler is a blue print of how software should be created.
It has a very clean interface as is very easy to use and understand.
It can be found at<br>
https://github.com/Bosma/Scheduler<br>
To install the Bosma Scheduler execute the following commands:

```
git clone https://github.com/vit-vit/CTPL.git
cd CTPL
sudo cp *.h /usr/local/include
cd ..

git clone https://github.com/Bosma/Scheduler.git
cd Scheduler
cmake CMakeLists.txt
make
sudo cp *.h /usr/local/include
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
cd ..
```

After the installation has completed you can remove the RxCpp directory.
The needed C++ header files have been installed in /usr/include/rxcpp.

### RxROS

Finally, we have come to the RxROS project. To install RxROS do the following:
```
git clone https://github.com/henrik7264/RxROS.git
cd RxROS
catkin_make
# catkin_make may fail to compile if the platform is not amd64. 
# Remove the build and devel directory to fix this pronblem
# and run catkin_make again.
sudo ./src/rxros_lang/src/install.sh
cd .. 
```

The RxROS language depends on the following software:<br>
1. Ubuntu Binoic 18.04
2. ROS Melodic
3. Reactive C++ v2

The software must be installed as described above.<br>
The RxROS itself consist of a singe file named rxros.h.
The file most be installed in /usr/local/include.<br>

Now, lets look at the language in more details.
The RxROS provides simple access to ROS via a set of classes.
The classes provides more precisely an extension to RxCpp that
gives simple access to ROS.<br>

####Initial setup
A RxROS program is in principle a ROS node,
so the first step is not surprisingly to initialise it and specify the node name.
This is done by means of the init function<br>

```cpp
rxros::init(argc, argv, "Name_of_ROS_node");
```

#####Example

```cpp
#include <rxros.h>
int main(int argc, char** argv) {'
    rxros::init(argc, argv, "velocity_publisher"); // Name of this node.

    // ... here 

    rxros::spin();
}
```

####Parameters
<br>

####Logging
<br>

####Topics
<br>

####Broadcasters
<br>

####Actions
<br>

####Services
<br>

#### Example
The following example is a full implementation of a velocity publisher
that takes input from a keyboard and joystick and publishes Twist messages
on the /cmd_vel topic:

```cpp
#include <rxros.h>
#include <teleop_msgs/Joystick.h>
#include <teleop_msgs/Keyboard.h>
#include <geometry_msgs/Twist.h>
#include "JoystickPublisher.h"
#include "KeyboardPublisher.h"


int main(int argc, char** argv) {
    rxros::init(argc, argv, "velocity_publisher"); // Name of this node.

    const auto frequencyInHz = rxros::Parameter::get("/velocity_publisher/frequency", 10.0); // hz
    const auto minVelLinear = rxros::Parameter::get("/velocity_publisher/min_vel_linear", 0.04); // m/s
    const auto maxVelLinear = rxros::Parameter::get("/velocity_publisher/max_vel_linear", 0.10); // m/s
    const auto minVelAngular = rxros::Parameter::get("/velocity_publisher/min_vel_angular", 0.64); // rad/s
    const auto maxVelAngular = rxros::Parameter::get("/velocity_publisher/max_vel_angular", 1.60); // rad/s
    const auto deltaVelLinear = (maxVelLinear - minVelLinear) / 10.0;
    const auto deltaVelAngular = (maxVelAngular - minVelAngular) / 10.0;

    rxros::Logging().info() << "frequency: " << frequencyInHz;
    rxros::Logging().info() << "min_vel_linear: " << minVelLinear << " m/s";
    rxros::Logging().info() << "max_vel_linear: " << maxVelLinear << " m/s";
    rxros::Logging().info() << "min_vel_angular: " << minVelAngular << " rad/s";
    rxros::Logging().info() << "max_vel_angular: " << maxVelAngular << " rad/s";

    auto adaptVelocity = [=] (auto newVel, auto minVel, auto maxVel, auto isIncrVel) {
        if (newVel > maxVel)
            return maxVel;
        else if (newVel < -maxVel)
            return -maxVel;
        else if (newVel > -minVel && newVel < minVel)
            return (isIncrVel) ? minVel : -minVel;
        else
            return newVel;};

    auto teleop2VelTuple = [=](const auto& prevVelTuple, const int event) {
        const auto prevVelLinear = std::get<0>(prevVelTuple);  // use previous linear and angular velocity
        const auto prevVelAngular = std::get<1>(prevVelTuple); // to calculate the new linear and angular velocity.
        if (event == JS_EVENT_BUTTON0_DOWN || event == JS_EVENT_BUTTON1_DOWN || event == KB_EVENT_SPACE)
            return std::make_tuple(0.0, 0.0); // Stop the robot
        else if (event == JS_EVENT_AXIS_UP || event == KB_EVENT_UP)
            return std::make_tuple(adaptVelocity((prevVelLinear + deltaVelLinear), minVelLinear, maxVelLinear, true), prevVelAngular); // move forward
        else if (event == JS_EVENT_AXIS_DOWN || event == KB_EVENT_DOWN)
            return std::make_tuple(adaptVelocity((prevVelLinear - deltaVelLinear), minVelLinear, maxVelLinear, false), prevVelAngular); // move backward
        else if (event == JS_EVENT_AXIS_LEFT || event == KB_EVENT_LEFT)
            return std::make_tuple(prevVelLinear, adaptVelocity((prevVelAngular + deltaVelAngular), minVelAngular, maxVelAngular, true)); // move left
        else if (event == JS_EVENT_AXIS_RIGHT || event == KB_EVENT_RIGHT)
            return std::make_tuple(prevVelLinear, adaptVelocity((prevVelAngular - deltaVelAngular), minVelAngular, maxVelAngular, false));}; // move right

    auto velTuple2TwistMsg = [](auto velTuple) {
        geometry_msgs::Twist vel;
        vel.linear.x = std::get<0>(velTuple);
        vel.angular.z = std::get<1>(velTuple);
        return vel;};

    auto joyObsrv = rxros::Observable::fromTopic<teleop_msgs::Joystick>("/joystick") // create an Observable stream from "/joystick" topic
        | map([](teleop_msgs::Joystick joy) { return joy.event; });
    auto keyObsrv = rxros::Observable::fromTopic<teleop_msgs::Keyboard>("/keyboard") // create an Observable stream from "/keyboard" topic
        | map([](teleop_msgs::Keyboard key) { return key.event; });
    joyObsrv.merge(keyObsrv)                                  // merge the joystick and keyboard messages into an Observable teleop stream.
        | scan(std::make_tuple(0.0, 0.0), teleop2VelTuple)    // turn the teleop stream into a linear and angular velocity stream.
        | map(velTuple2TwistMsg)                              // turn the linear and angular velocity stream into a Twist stream.
        | sample_with_frequency(frequencyInHz)                // take latest Twist msg and populate it with the specified frequency.
        | publish_to_topic<geometry_msgs::Twist>("/cmd_vel"); // publish the Twist messages to the topic "/cmd_vel"

    rxros::Logging().info() << "Spinning velocity_publisher ...";
    rxros::spin();
}
```




## Problems and observations

1. Drifting odom when rotating the robot<br>
The following picture shows the robot and laser scan seen from the odom frame.
acml and map has been disabled/turned off. The robot has not moved and the laserscan
shows straight lines. Observe that the laserscan decay time has been set to 30 sec as
I want to demonstrate some problems with the odom of the robot. Please also observe
the small red dots in the picture. They are USB wires that runs from my PC to the robot.<br>
![odom_no_movement](/images/robot_odom_no_movement.png)<br>
The next picture shows the laserscan after the robot has moved straight forward and backwards
a couple of times. There is very little drift in the odom of robot. When amcl is activated it
is actually able to correct the position of the robot. This indicates to me that the drift of
the odom is acceptable.<br>
![odom_moving_forward](/images/robot_odom_moving_forward_and_back.png)<br>
Now we come to the actual problem. When I start to rotate the robot the drift of the odom goes crazy
as shown in the picture below. I cannot determine the actual cause of this drift, but it has a bad
impact on amcl. I actually thinks it disable amcl as I see no corrections of the position after I
have make a couple of turns with the robot<br>
![odom_moving_left](/images/robot_odom_turning_left_and_right.png)<br>
2. roswtf reports errors<br>
When roswtf is executed the following errors are shown<br>
![roswtf fail](/images/roswtf_error.png)<br>
The impact if the error is currently unknown.
3. The rosgraph of the robot<br>
![rosgraph](/images/ros_robot_rosgraph.svg)<br>
![rosgraph](/images/ros_robot_rosgraph2.svg)<br>
<br>
