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
        Logging() = default;
        ~Logging() override {
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
        ros::NodeHandle nodeHandle;

        Parameter() = default;;

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

        static auto get(const std::string& name, const std::string& defaultValue)
        {
            return get<std::string>(name, defaultValue);
        }
    }; // end of class Parameter


    class NodeHandle : public ros::NodeHandle
    {
    public:
        template<class T>
        ros::Subscriber subscribe(const std::string& topic, uint32_t queue_size, const std::function<void(const T&)>& callback, const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(), const ros::TransportHints& transport_hints = ros::TransportHints())
        {
            return ros::NodeHandle::subscribe<T>(topic, queue_size, static_cast<boost::function<void(const T&)>>(callback), tracked_object, transport_hints);
        }

        template<class T>
        ros::ServiceServer advertiseService(const std::string& service, const std::function<bool(typename T::Request&, typename T::Response&)> callback,const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr())
        {
            return ros::NodeHandle::advertiseService<typename T::Request, typename T::Response>(service, static_cast<boost::function<bool(typename T::Request&, typename T::Response&)>>(callback), tracked_object);
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
        rxros::NodeHandle nodeHandle;
        ros::Subscriber subscriber;

        // We subscribe to a ROS topic and use the callback function to handle updates of the topic.
        explicit Observable(const std::string& topic, const uint32_t queueSize = 10):
            subscriber(nodeHandle.subscribe(topic, queueSize, [=](const T& val){subject.get_subscriber().on_next(val);})) {}

    public:
        virtual ~Observable() = default;

        static auto fromTopic(const std::string& topic, const uint32_t queueSize = 10)
        {
            auto* self = new Observable(topic, queueSize);
            return self->subject.get_observable(); // and return the RxCpp observable of the subject.
        }


        static auto fromTransformListener(const std::string& frameId, const std::string& childFrameId, const double frequencyInHz = 10.0)
        {
            //assert(typeid(T) == typeid(tf::StampedTransform));
            return rxcpp::observable<>::create<T>(
                [=](rxcpp::subscriber<T> subscriber) {
                    ros::NodeHandle nodeHandle;
                    tf::TransformListener listener;
                    ros::Rate rate(frequencyInHz);
                    while (nodeHandle.ok()) {
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
                    if (!nodeHandle.ok()) {
                        subscriber.on_completed();
                    }});
        };
    }; // end of class Observable


    template<class T>
    class Publisher
    {
    private:
        ros::NodeHandle nodeHandle;
        ros::Publisher publisher;

        explicit Publisher(const std::string& topic, const uint32_t queueSize = 10) :
            publisher(nodeHandle.advertise<T>(topic, queueSize)) {}

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
        template <typename T, typename F>
        auto iff(bool b, T&& t, F&& f) {
            if (b)
                return t;
            return f;
        }

//        template <typename V, typename... Vs, typename T, typename F>
//        auto eq(V v, Vs... vs, T&& t, F&& f) {
//            for (auto e: vs) {
//                if (v == e)
//                    return t;
//            }
//            return f;};

    }; // end of namespace utils
}; // end of namespace rxros


namespace rxcpp
{
    namespace operators
    {
        auto sample_every(const double frequencyInHz) {
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
