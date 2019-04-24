// Auto-generated. Do not edit!

// (in-package eurobot2019_messages.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class drop_command {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.left = null;
      this.right = null;
      this.middle = null;
    }
    else {
      if (initObj.hasOwnProperty('left')) {
        this.left = initObj.left
      }
      else {
        this.left = false;
      }
      if (initObj.hasOwnProperty('right')) {
        this.right = initObj.right
      }
      else {
        this.right = false;
      }
      if (initObj.hasOwnProperty('middle')) {
        this.middle = initObj.middle
      }
      else {
        this.middle = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type drop_command
    // Serialize message field [left]
    bufferOffset = _serializer.bool(obj.left, buffer, bufferOffset);
    // Serialize message field [right]
    bufferOffset = _serializer.bool(obj.right, buffer, bufferOffset);
    // Serialize message field [middle]
    bufferOffset = _serializer.bool(obj.middle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type drop_command
    let len;
    let data = new drop_command(null);
    // Deserialize message field [left]
    data.left = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [right]
    data.right = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [middle]
    data.middle = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 3;
  }

  static datatype() {
    // Returns string type for a message object
    return 'eurobot2019_messages/drop_command';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c498bd496c426b7314def7449c775a44';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Which tower to drop from
    # 0 is idle, 1 is unload
    bool left
    bool right
    bool middle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new drop_command(null);
    if (msg.left !== undefined) {
      resolved.left = msg.left;
    }
    else {
      resolved.left = false
    }

    if (msg.right !== undefined) {
      resolved.right = msg.right;
    }
    else {
      resolved.right = false
    }

    if (msg.middle !== undefined) {
      resolved.middle = msg.middle;
    }
    else {
      resolved.middle = false
    }

    return resolved;
    }
};

module.exports = drop_command;
