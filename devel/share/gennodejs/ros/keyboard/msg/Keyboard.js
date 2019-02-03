// Auto-generated. Do not edit!

// (in-package keyboard.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Keyboard {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.time = null;
      this.key = null;
    }
    else {
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('key')) {
        this.key = initObj.key
      }
      else {
        this.key = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Keyboard
    // Serialize message field [time]
    bufferOffset = _serializer.time(obj.time, buffer, bufferOffset);
    // Serialize message field [key]
    bufferOffset = _serializer.uint16(obj.key, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Keyboard
    let len;
    let data = new Keyboard(null);
    // Deserialize message field [time]
    data.time = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [key]
    data.key = _deserializer.uint16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'keyboard/Keyboard';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'aa33d4f1ca40c17b9cc7a9d013da833e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time time
    uint16 key
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Keyboard(null);
    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = {secs: 0, nsecs: 0}
    }

    if (msg.key !== undefined) {
      resolved.key = msg.key;
    }
    else {
      resolved.key = 0
    }

    return resolved;
    }
};

module.exports = Keyboard;
