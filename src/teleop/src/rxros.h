//
// Created by hl on 2/24/19.
//

#ifndef RXROS_H
#define RXROS_H

#include <cassert>
#include <rxcpp/rx.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace Rx {
using namespace rxcpp;
using namespace rxcpp::sources;
using namespace rxcpp::operators;
using namespace rxcpp::util;
}
using namespace Rx;

namespace rxros
{
    static void init(int argc, char** argv, const std::string& name) {ros::init(argc, argv, name);}
    static void spin() {ros::spin();}

    class Logging: public std::ostringstream
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

        Parameter() {};

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
        virtual ~Parameter() {};

        template<typename T>
        static auto get(const std::string& name, const T& defaultValue)
        {
            return Parameter().getParam<T>(name, defaultValue);
        }

        static auto get(const std::string& name, const int defaultValue)
        {
            return Parameter().getParam(name, defaultValue);
        }

        static auto get(const std::string& name, const double defaultValue)
        {
            return Parameter().getParam(name, defaultValue);
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

        // We subscribe to a ROS topic and use the callback function to handle updates of the topic.
        Observable(const std::string& topic, const uint32_t queueSize = 10):
            subscriber(nodeHandle.subscribe(topic, queueSize, &Observable::callback, this)) {}

        auto getSubjectSubscriber() {return subject.get_subscriber();}
        auto getSubjectObservable() {return subject.get_observable();}

        // Callback function used by ROS subscriber
        void callback(const T& val) {
            getSubjectSubscriber().on_next(val);
        }

    public:
        virtual ~Observable() {}

        static auto fromTopic(const std::string& topic, const uint32_t queueSize = 10) {
            Observable* self = new Observable(topic, queueSize);
            return self->getSubjectObservable(); // and return the RxCpp observable of the subject.
        }
    };

    template<class T>
    class Publisher
    {
    private:
        ros::NodeHandle nodeHandle;
        ros::Publisher publisher;

        Publisher(const std::string& topic, const uint32_t queueSize = 10) :
            publisher(nodeHandle.advertise<T>(topic, queueSize)) {}

    public:
        virtual ~Publisher() {}

        static auto publish(const rxcpp::observable<T> &observ, const std::string &topic, const uint32_t queueSize = 10) {
            Publisher* self = new Publisher(topic, queueSize);
            observ.subscribe_on(synchronize_new_thread()).subscribe(
                [=](const T& msg) {self->publisher.publish(msg);});
            return observ;}
    };

    class TransformBroadcaster
    {
    private:
        tf::TransformBroadcaster transformBroadcaster;

        TransformBroadcaster() {}

    public:
        virtual ~TransformBroadcaster() {}

        static auto sendTransform(const rxcpp::observable<tf::Transform> &observ, const std::string& frame_id, const std::string& child_frame_id) {
            TransformBroadcaster* self = new TransformBroadcaster();
            observ.subscribe_on(synchronize_new_thread()).subscribe(
                [=](const tf::Transform& tf) {self->transformBroadcaster.sendTransform(tf::StampedTransform(tf, ros::Time::now(), frame_id, child_frame_id));});
            return observ;}
    };


//    class Observable
//    {
//    private:
//        tf::TransformListener transformListener;
//
//        Observable() {}
//
//    public:
//        virtual ~Observable() {}
//
//
//    };

}; // end of namespace rxros


namespace rxcpp
{
    namespace operators
    {
        auto sample_every(const std::chrono::milliseconds &durationInMs) {
            return [=](auto &&source) {
                return rxcpp::observable<>::interval(durationInMs).with_latest_from(
                        [=](const auto x, const auto y) { return y; }, source);};};

        template<typename T>
        auto publishToTopic(const std::string &topic, const uint32_t queueSize = 10) {
            return [=](auto &&source) {
                return rxros::Publisher<T>::publish(source, topic, queueSize);};};

        auto sendTransform(const std::string& frame_id, const std::string& child_frame_id) {
            return [=](auto &&source) {
                return rxros::TransformBroadcaster::sendTransform(source, frame_id, child_frame_id);};};

    }; // end namespace operators
}; // end namespace rxcpp


#endif //RXROS_H

