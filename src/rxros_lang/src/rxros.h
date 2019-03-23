//
// Created by hl on 2/24/19.
//

#ifndef RXROS_H
#define RXROS_H

#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
#include <cassert>
#include <linux/input.h>
#include <rxcpp/rx.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace rxcpp;
using namespace rxcpp::sources;
using namespace rxcpp::operators;
using namespace rxcpp::util;


namespace rxros
{
    static void init(int argc, char** argv, const std::string& name) {ros::init(argc, argv, name);}
    static void spin() {ros::spin();}
    static bool ok() {return ros::ok();}

    class Node
    {
    private:
        Node() = default;
    public:
        ~Node() = default;
        static auto getHandle() {
            static ros::NodeHandle nodehandle;
            return nodehandle;
        }
    }; // end of class Node

    class Exception
    {
    private:
        Exception() = default;

    public:
        ~Exception() = default;
        static auto systemError(const int errCode, const std::string &msg)
        {
            return std::make_exception_ptr(std::system_error(std::error_code(errCode, std::generic_category()), msg));
        }
    }; // end of class Exception


    class Logging: public std::ostringstream
    {
    private:
        enum LogLevel {DEBUG, INFO, WARN, ERROR, FATAL};
        LogLevel logLevel;

    public:
        Logging() = default;
        ~Logging() override {
            switch(logLevel) {
                case DEBUG:
                    ROS_DEBUG("%s", str().c_str());
                    break;
                case INFO:
                    ROS_INFO("%s", str().c_str());
                    break;
                case WARN:
                    ROS_WARN("%s", str().c_str());
                    break;
                case ERROR:
                    ROS_ERROR("%s", str().c_str());
                    break;
                case FATAL:
                    ROS_FATAL("%s", str().c_str());
                    break;
                default:
                    ROS_FATAL("Ups!!!!");
                    break;
            }
        }

        Logging& debug()
        {
            logLevel = DEBUG;
            return *this;
        }

        Logging& info()
        {
            logLevel = INFO;
            return *this;
        }

        Logging& warn()
        {
            logLevel = WARN;
            return *this;
        }

        Logging& error()
        {
            logLevel = ERROR;
            return *this;
        }

        Logging& fatal()
        {
            logLevel = FATAL;
            return *this;
        }
    }; // end of class Logging


    class Parameter
    {
    private:
        Parameter() = default;;

        template<typename T>
        auto getParam(const std::string& name, const T& defaultValue)
        {
            T param;
            Node::getHandle().param<T>(name, param, defaultValue);
            return param;
        }

        auto getParam(const std::string& name, const int defaultValue)
        {
            int param;
            Node::getHandle().param(name, param, defaultValue);
            return param;
        }

        auto getParam(const std::string& name, const double defaultValue)
        {
            double param;
            Node::getHandle().param(name, param, defaultValue);
            return param;
        }

    public:
        virtual ~Parameter() = default;;

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

        static auto get(const std::string& name, const char* defaultValue)
        {
            return get<std::string>(name, defaultValue);
        }

        static auto get(const std::string& name, const std::string& defaultValue)
        {
            return get<std::string>(name, defaultValue);
        }
    }; // end of class Parameter


    template<class T>
    class Observable
    {
    private:
        /* A subject is an entity that is simultaneously
         * an Observer and an Observable. It helps to
         * relay notifications from Observable to a
         * set of Observers. */
        rxcpp::subjects::subject<T> subject;
        ros::Subscriber subscriber;

        // We subscribe to a ROS topic and use the callback function to handle updates of the topic.
        explicit Observable(const std::string& topic, const uint32_t queueSize = 10):
            subscriber(Node::getHandle().subscribe(topic, queueSize, &Observable::callback, this)) {}

         void callback(const T &val)
         {
            subject.get_subscriber().on_next(val);
         }

    public:
        virtual ~Observable() = default;

        static auto fromTopic(const std::string& topic, const uint32_t queueSize = 10)
        {
            auto* self = new Observable(topic, queueSize);
            return self->subject.get_observable(); // and return the RxCpp observable of the subject.
        }

        static auto fromTransformListener(const std::string& frameId, const std::string& childFrameId, const double frequencyInHz = 10.0)
        {
            assert(typeid(T) == typeid(tf::StampedTransform));
            return rxcpp::observable<>::create<T>(
                [=](rxcpp::subscriber<T> subscriber) {
                    tf::TransformListener listener;
                    ros::Rate rate(frequencyInHz);
                    while (rxros::ok()) {
                        try {
                            tf::StampedTransform transform;
                            listener.lookupTransform(frameId, childFrameId, ros::Time(0), transform);
                            subscriber.on_next(transform);
                        }
                        catch (...) {
                            std::exception_ptr err = std::current_exception();
                            subscriber.on_error(err);
                            break;
                        }
                        rate.sleep();
                    }
                    if (!rxros::ok()) {
                        subscriber.on_completed();
                    }});
        }

        static auto fromDevice(const std::string& deviceName)
        {
            return rxcpp::observable<>::create<T>(
                [=](rxcpp::subscriber<T> subscriber) {
                    int fd = open(deviceName.c_str(), O_RDONLY | O_NONBLOCK);
                    if (fd < 0)
                        subscriber.on_error(rxros::Exception::systemError(errno, std::string("Cannot open device ") + deviceName));
                    else {
                        fd_set readfds; // initialize file descriptor set.
                        FD_ZERO(&readfds);
                        FD_SET(fd, &readfds);
                        T event{};
                        bool doLoop = true;
                        while (doLoop && rxros::ok()) {
                            int rc = select(fd + 1, &readfds, nullptr, nullptr, nullptr);  // wait for input on keyboard device
                            if (rc == -1 &&
                                errno == EINTR) { // select was interrupted. This is not an error but we exit the loop
                                subscriber.on_completed();
                                close(fd);
                                doLoop = false;
                            } else if (rc == -1 || rc == 0) { // select failed and we issue an error.
                                subscriber.on_error(rxros::Exception::systemError(errno, std::string("Failed to read device ") + deviceName));
                                close(fd);
                                doLoop = false;
                            } else if (FD_ISSET(fd, &readfds)) {
                                ssize_t sz = read(fd, &event, sizeof(T)); // read pressed key
                                if (sz == -1) {
                                    subscriber.on_error(rxros::Exception::systemError(errno, std::string("Failed to read device ") + deviceName));
                                    close(fd);
                                    doLoop = false;
                                } else if (sz == sizeof(T)) {
                                    subscriber.on_next(event); // populate the event on the
                                }
                            }
                        }
                        if (!rxros::ok()) {
                            subscriber.on_completed();
                        }
                    }
                });
        }

        // Parse the ros_robot.yaml file and create an observable stream of the sensors and actuators configurations
        static auto fromRobotYaml(const std::string& aNamespace)
        {
            assert(typeid(T) == typeid(XmlRpc::XmlRpcValue));
            return rxcpp::observable<>::create<T>(
                [=](rxcpp::subscriber<T> subscriber) {
                    XmlRpc::XmlRpcValue robotConfig;
                    Node::getHandle().getParam(aNamespace, robotConfig);
                    assert (robotConfig.getType() == XmlRpc::XmlRpcValue::TypeArray);
                    for (int i = 0; i < robotConfig.size(); i++)
                        subscriber.on_next(robotConfig[i]);
                    subscriber.on_completed();
                });
        }
    }; // end of class Observable


    template<class T>
    class Publisher
    {
    private:
        ros::Publisher publisher;

        explicit Publisher(const std::string& topic, const uint32_t queueSize = 10) :
            publisher(Node::getHandle().advertise<T>(topic, queueSize)) {}

    public:
        virtual ~Publisher() = default;

        static auto publish(const rxcpp::observable<T> &observ, const std::string &topic, const uint32_t queueSize = 10)
        {
            auto* self = new Publisher(topic, queueSize);
            observ.subscribe_on(synchronize_new_thread()).subscribe(
                [=](const T& msg) {self->publisher.publish(msg);});
            return observ;
        }
    }; // end of class Publisher

}; // end of namespace rxros


namespace rxros
{
    namespace utils
    {
        template <typename E, typename F>
        auto select(E exp, F&& func) {
            if (exp)
                return func();
        }

        template <typename E, typename F, typename ...EFs>
        auto select(E exp, F&& func, EFs... exp_func_pairs) {
            if (exp)
                return func();
            return select(exp_func_pairs...);
        }
    }; // end of namespace utils
}; // end of namespace rxros


namespace rxcpp
{
    namespace operators
    {
        auto sample_with_frequency(const double frequencyInHz) {
            return [=](auto &&source) {
                const std::chrono::milliseconds durationInMs(static_cast<long>(1000.0/frequencyInHz));
                return rxcpp::observable<>::interval(durationInMs).with_latest_from(
                        [=](const auto intervalObsrv, const auto sourceObsrv) { return sourceObsrv; }, source);};};


        template<typename T>
        auto publish_to_topic(const std::string &topic, const uint32_t queueSize = 10) {
            return [=](auto &&source) {
                return rxros::Publisher<T>::publish(source, topic, queueSize);};};


        auto send_transform(const std::string &frame_id, const std::string &child_frame_id) {
            return [=](auto &&source) {
                static tf::TransformBroadcaster transformBroadcaster;
                source.subscribe_on(synchronize_new_thread()).subscribe(
                    [=](const tf::Transform& tf) {transformBroadcaster.sendTransform(tf::StampedTransform(tf, ros::Time::now(), frame_id, child_frame_id));});
                return source;};};

    }; // end namespace operators
}; // end namespace rxcpp


#endif //RXROS_H
