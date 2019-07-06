//
// Created by hl on 6/24/19.
//

#ifndef RXCPP_LANG_BRICKPI3UTILS_H
#define RXCPP_LANG_BRICKPI3UTILS_H

#include <ros/ros.h>

// Support class to simplify access to configuration parameters for device
class DeviceConfig
{
private:
    std::string type;
    std::string name;
    std::string frameId;
    std::string port;
    double frequency;
    double minRange;
    double maxRange;
    double spreadAngle;

public:
    explicit DeviceConfig(XmlRpc::XmlRpcValue& value) {
        type = value.hasMember("type") ? (std::string) value["type"] : std::string("");
        name = value.hasMember("name") ? (std::string) value["name"] : std::string("");
        frameId = value.hasMember("frame_id") ? (std::string) value["frame_id"] : std::string("");
        port = value.hasMember("port") ? (std::string) value["port"] : std::string("");
        frequency = value.hasMember("frequency") ? (double) value["frequency"] : 0.0;
        minRange = value.hasMember("min_range") ? (double) value["min_range"] : 0.0;
        maxRange = value.hasMember("max_range") ? (double) value["max_range"] : 0.0;
        spreadAngle = value.hasMember("spread_angle") ? (double) value["spread_angle"] : 0.0;
    }
    ~DeviceConfig() = default;

    const std::string& getType() const {return type;}
    const std::string& getName() const {return name;}
    const std::string& getFrameId() const {return frameId;}
    const std::string& getPort() const {return port;}
    double getFrequency() const {return frequency;}
    double getMinRange() const { return minRange;}
    double getMaxRange() const { return  maxRange;}
    double getSpreadAngle() const { return spreadAngle;}
};

#endif //RXCPP_LANG_BRICKPI3UTILS_H
