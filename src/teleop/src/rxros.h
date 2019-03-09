//
// Created by hl on 2/24/19.
//

#ifndef RXROS_H
#define RXROS_H

#include <cassert>
#include <rxcpp/rx.hpp>
#include <ros/ros.h>
#include <ros/console.h>
namespace Rx {
using namespace rxcpp;
using namespace rxcpp::sources;
using namespace rxcpp::operators;
using namespace rxcpp::util;
}
using namespace Rx;


namespace rxcpp
{
namespace operators
{
auto sample_every(const std::chrono::milliseconds& durationInMs) {
    return [=](auto &&source) {
        return rxcpp::observable<>::interval(durationInMs)
            .with_latest_from([=](const auto x, const auto y) {return y;}, source);};};
}
}


namespace rxros
{

class Logging : public std::ostringstream
{
private:
    enum LogLevel {DEBUG, INFO, WARN, ERROR, FATAL};
    LogLevel logLevel;

public:
    Logging() {}
    virtual ~Logging() {
        switch(logLevel) {
            case DEBUG:
                ROS_DEBUG("%s\n", str().c_str());
                break;
            case INFO:
                std::cout << str() << std::endl;
                ROS_INFO("%s\n", str().c_str());
                break;
            case WARN:
                ROS_WARN("%s\n", str().c_str());
                break;
            case ERROR:
                ROS_ERROR("%s\n", str().c_str());
                break;
            case FATAL:
                ROS_FATAL("%s\n", str().c_str());
                break;
            default:
                ROS_FATAL("Ups!!!!");
                break;
        }
    }

    Logging& debug() {
        logLevel = DEBUG;
        return *this;
    }

    Logging& info() {
        logLevel = INFO;
        return *this;
    }

    Logging& warn() {
        logLevel = WARN;
        return *this;
    }

    Logging& error() {
        logLevel = ERROR;
        return *this;
    }

    Logging& fatal() {
        logLevel = FATAL;
        return *this;
    }
};

class Parameter
{
private:
    ros::NodeHandle nodeHandle;

public:
    Parameter() {};
    virtual ~Parameter() {};

    template<typename T>
    auto get(const std::string& name, const T& defaultValue)
    {
        T param;
        nodeHandle.param<T>(name, param, defaultValue);
        return param;
    }

    auto get(const std::string& name, const int defaultValue)
    {
        int param;
        nodeHandle.param(name, param, defaultValue);
        return param;
    }

    auto get(const std::string& name, const double defaultValue)
    {
        double param;
        nodeHandle.param(name, param, defaultValue);
        return param;
    }

    auto get(const std::string& name, const std::string& defaultValue)
    {
        return get<std::string>(name, defaultValue);
    }
};

template<class T>
class Observable
{
private:
    /* A subject is an entity that is simultaneously
     * an Observer and an Observable. It helps to
     * relay notifications from Observable to a
     * set of Observers. */
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber;
    rxcpp::subjects::subject<T> subject;

    auto getSubjectSubscriber() {return subject.get_subscriber();}
    auto getSubjectObservable() {return subject.get_observable();}

    // Callback function used by ROS subscriber
    void callback(const T& val) {
        getSubjectSubscriber().on_next(val);
    }

public:
    // We subscribe to a ROS topic and use the callback function to handle updates of the topic.
    Observable() {}
    Observable(const std::string& topic, const uint32_t queueSize):
        subscriber(nodeHandle.subscribe(topic, queueSize, &Observable::callback, this)) {}
    virtual ~Observable() {std::cout << "Calling mr ~Observable." << std::endl;}

    auto fromTopic(const std::string& topic, const uint32_t queueSize = 10) {
        Observable* observable1 = new Observable(topic, queueSize);
        return observable1->getSubjectObservable(); // and return the RxCpp observable of the subject.
    }
};


template<class T>
class Publish
{
private:
    ros::NodeHandle nodeHandle;
    ros::Publisher publisher;
    long i;

public:
    Publish(const std::string& topic, const uint32_t queueSize = 10) :
        publisher(nodeHandle.advertise<T>(topic, queueSize)) {i = 0;}
    virtual ~Publish() {}

    void fromObservable(rxcpp::observable<T>& observ) {
        observ.subscribe_on(synchronize_new_thread()).subscribe(
            [&](const T& msg) {publisher.publish(msg); std::cout << i++ << std::endl;});}
};

} // end of namespace rxros

#endif //RXROS_H

