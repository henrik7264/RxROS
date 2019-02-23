// Auto-generated. Do not edit!

// (in-package brickpi3_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Range {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.range = null;
      this.range_min = null;
      this.range_max = null;
      this.spread_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('range')) {
        this.range = initObj.range
      }
      else {
        this.range = 0.0;
      }
      if (initObj.hasOwnProperty('range_min')) {
        this.range_min = initObj.range_min
      }
      else {
        this.range_min = 0.0;
      }
      if (initObj.hasOwnProperty('range_max')) {
        this.range_max = initObj.range_max
      }
      else {
        this.range_max = 0.0;
      }
      if (initObj.hasOwnProperty('spread_angle')) {
        this.spread_angle = initObj.spread_angle
      }
      else {
        this.spread_angle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Range
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [range]
    bufferOffset = _serializer.float64(obj.range, buffer, bufferOffset);
    // Serialize message field [range_min]
    bufferOffset = _serializer.float64(obj.range_min, buffer, bufferOffset);
    // Serialize message field [range_max]
    bufferOffset = _serializer.float64(obj.range_max, buffer, bufferOffset);
    // Serialize message field [spread_angle]
    bufferOffset = _serializer.float64(obj.spread_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Range
    let len;
    let data = new Range(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [range]
    data.range = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [range_min]
    data.range_min = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [range_max]
    data.range_max = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [spread_angle]
    data.spread_angle = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'brickpi3_msgs/Range';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '37a3243712d0ae8a8c4863ac843a1c75';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64 range
    float64 range_min
    float64 range_max
    float64 spread_angle
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Range(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.range !== undefined) {
      resolved.range = msg.range;
    }
    else {
      resolved.range = 0.0
    }

    if (msg.range_min !== undefined) {
      resolved.range_min = msg.range_min;
    }
    else {
      resolved.range_min = 0.0
    }

    if (msg.range_max !== undefined) {
      resolved.range_max = msg.range_max;
    }
    else {
      resolved.range_max = 0.0
    }

    if (msg.spread_angle !== undefined) {
      resolved.spread_angle = msg.spread_angle;
    }
    else {
      resolved.spread_angle = 0.0
    }

    return resolved;
    }
};

module.exports = Range;
