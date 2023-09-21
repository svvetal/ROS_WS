// Auto-generated. Do not edit!

// (in-package custom_msg_week3.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class custom {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.user_input = null;
    }
    else {
      if (initObj.hasOwnProperty('user_input')) {
        this.user_input = initObj.user_input
      }
      else {
        this.user_input = new std_msgs.msg.String();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type custom
    // Serialize message field [user_input]
    bufferOffset = std_msgs.msg.String.serialize(obj.user_input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type custom
    let len;
    let data = new custom(null);
    // Deserialize message field [user_input]
    data.user_input = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.user_input);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'custom_msg_week3/custom';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f3d8e52418e879f2a35002f345d5e6fb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/String user_input
    ================================================================================
    MSG: std_msgs/String
    string data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new custom(null);
    if (msg.user_input !== undefined) {
      resolved.user_input = std_msgs.msg.String.Resolve(msg.user_input)
    }
    else {
      resolved.user_input = new std_msgs.msg.String()
    }

    return resolved;
    }
};

module.exports = custom;
