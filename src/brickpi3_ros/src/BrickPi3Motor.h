//
// Created by hl on 2/10/19.
//

#ifndef BRICKPI3_ROS_BRICKPI3MOTOR_H
#define BRICKPI3_ROS_BRICKPI3MOTOR_H

class BrickPi3Motor {
private:
    Bosma::Scheduler scheduler;
    ros::NodeHandle nodeHandle;
    std::string name;
    std::string port;
    int freq;

    void schedulerCB();

public:
    BrickPi3Motor(const std::string& name, const std::string& port, const int frequency);
    virtual ~BrickPi3Motor() {}
    void run() {ros::spin();}
};

#endif //BRICKPI3_ROS_BRICKPI3MOTOR_H
