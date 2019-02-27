//
// Created by hl on 2/24/19.
//

#ifndef RXROS_H
#define RXROS_H

#include <cassert>
#include <rxcpp/rx.hpp>
//#include <rxcpp/rx-subscriber.hpp>
#include <ros/ros.h>
#include <ros/console.h>

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

    template<typename T>
    auto getParam(const std::string& name, const T& defaultValue)
    {
        T param;
        nodeHandle.param<T>(name, param, defaultValue);
        return param;
    }

    auto getParam(const std::string& name, const int defaultValue)
    {
        int param;
        nodeHandle.param(name, param, defaultValue);
        return param;
    }
    auto getParam(const std::string& name, const double defaultValue)
    {
        double param;
        nodeHandle.param(name, param, defaultValue);
        return param;
    }

public:
    Parameter() {};
    virtual ~Parameter() {};

    template<typename T>
    static auto get(const std::string& name, const T& defaultValue)
    {
        Parameter self;
        return self.getParam<T>(name, defaultValue);
    }

    static auto get(const std::string& name, const int defaultValue)
    {
        Parameter self;
        return self.getParam(name, defaultValue);
    }

    static auto get(const std::string& name, const double defaultValue)
    {
        Parameter self;
        return self.getParam(name, defaultValue);
    }

    static auto get(const std::string& name, const std::string& defaultValue)
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
    rxcpp::subjects::subject<T> subject;
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber;

    auto getSubjectSubscriber() {return subject.get_subscriber();}
    auto getSubjectObservable() {return subject.get_observable();}

    // Callback function used by ROS subscriber
    void callback(const T& val) {
        getSubjectSubscriber().on_next(val);
    }

public:
    // We subscribe to a ROS topic and use the callback function to handle updates of the topic.
    Observable(const std::string& topic, const uint32_t queueSize = 10) :
        subscriber(nodeHandle.subscribe(topic, queueSize, &Observable::callback, this)) {}
    virtual ~Observable() {}

    static auto fromTopic(const std::string& topic, const uint32_t queueSize = 10)
    {
        static Observable* self = nullptr;
        assert(self == nullptr);
        self = new Observable(topic, queueSize); // We create a new rxros::Observable which will setup an appropriate ROS subscription of the topic.
        return self->getSubjectObservable().finally([](){delete self;}); // and return the RxCpp observable of the subject.
    }
};


//template<class T>
//class Subscriber: public rxcpp::subscriber<T>
//{
//private:
//    ros::NodeHandle nodeHandle;
//    ros::Publisher publisher;
//
//public:
//    explicit Subscriber(const std::string& topic, const uint32_t queueSize = 10) :
//        publisher(nodeHandle.advertise<T>(topic, queueSize)) {}
//    virtual ~Subscriber() {}
//
//    template<class V>
//    void on_next(V&& v) const {
//        rxcpp::subscriber<T>::on_next(v);
//        publisher.publish(v);
//    }
//
//    void on_error(rxcpp::rxu::error_ptr e) const {
//        rxcpp::subscriber<T>::on_error(e);
//        publisher.shutdown();
//    }
//
//    void on_completed() const {
//        rxcpp::subscriber<T>::on_completed();
//        publisher.shutdown();
//    }
//};

}

#endif //RXROS_H