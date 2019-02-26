//
// Created by hl on 2/24/19.
//

#ifndef RXROS_H
#define RXROS_H

class error_ptr;
#include <rxcpp/rx.hpp>
//#include <rxcpp/rx-subscriber.hpp>
#include <ros/ros.h>

namespace rxros
{

template<class T>
class Observable
{
private:
    rxcpp::subjects::subject<T> subject;
    ros::NodeHandle nodeHandle;
    ros::Subscriber subscriber;

    auto getSubscriber() {return subject.get_subscriber();}
    auto getObservable() {return subject.get_observable();}

    void callback(const T& val) {
        getSubscriber().on_next(val);
    }

public:
    Observable(const std::string& topic, const uint32_t queueSize = 10) :
        subscriber(nodeHandle.subscribe(topic, queueSize, &Observable::callback, this)) {}
    virtual ~Observable() {}

    static auto fromTopic(const std::string& topic, const uint32_t queueSize = 10)
    {
        static Observable* self = nullptr;
        if (self == nullptr) {
           self = new Observable(topic, queueSize);
        }
        return self->getObservable().finally([](){delete self;});
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
};

template<class T>
class Subscriber: public rxcpp::subscriber<T>
{
private:
    ros::NodeHandle nodeHandle;
    ros::Publisher publisher;

public:
    explicit Subscriber(const std::string& topic, const uint32_t queueSize = 10) :
        publisher(nodeHandle.advertise<T>(topic, queueSize)) {}
    virtual ~Subscriber() {}

    template<class V>
    void on_next(V&& v) const {
        rxcpp::subscriber<T>::on_next(v);
        publisher.publish(v);
    }

//    void on_error(rxcpp::rxu::error_ptr e) const {
//        rxcpp::subscriber<T>::on_error(e);
//        publisher.shutdown();
//    }
//
//    void on_completed() const {
//        rxcpp::subscriber<T>::on_completed();
//        publisher.shutdown();
//    }
};


}

#endif //RXROS_H
