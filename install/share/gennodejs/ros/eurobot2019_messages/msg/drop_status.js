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

class drop_status {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.left_tower_contents = null;
      this.middle_tower_contents = null;
      this.right_tower_contents = null;
    }
    else {
      if (initObj.hasOwnProperty('left_tower_contents')) {
        this.left_tower_contents = initObj.left_tower_contents
      }
      else {
        this.left_tower_contents = 0;
      }
      if (initObj.hasOwnProperty('middle_tower_contents')) {
        this.middle_tower_contents = initObj.middle_tower_contents
      }
      else {
        this.middle_tower_contents = 0;
      }
      if (initObj.hasOwnProperty('right_tower_contents')) {
        this.right_tower_contents = initObj.right_tower_contents
      }
      else {
        this.right_tower_contents = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type drop_status
    // Serialize message field [left_tower_contents]
    bufferOffset = _serializer.uint8(obj.left_tower_contents, buffer, bufferOffset);
    // Serialize message field [middle_tower_contents]
    bufferOffset = _serializer.uint8(obj.middle_tower_contents, buffer, bufferOffset);
    // Serialize message field [right_tower_contents]
    bufferOffset = _serializer.uint8(obj.right_tower_contents, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type drop_status
    let len;
    let data = new drop_status(null);
    // Deserialize message field [left_tower_contents]
    data.left_tower_contents = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [middle_tower_contents]
    data.middle_tower_contents = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [right_tower_contents]
    data.right_tower_contents = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 3;
  }

  static datatype() {
    // Returns string type for a message object
    return 'eurobot2019_messages/drop_status';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '86a774b860e3f87b013422373c4196ad';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # The number of pucks in the respective tower
    uint8 left_tower_contents
    uint8 middle_tower_contents
    uint8 right_tower_contents
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new drop_status(null);
    if (msg.left_tower_contents !== undefined) {
      resolved.left_tower_contents = msg.left_tower_contents;
    }
    else {
      resolved.left_tower_contents = 0
    }

    if (msg.middle_tower_contents !== undefined) {
      resolved.middle_tower_contents = msg.middle_tower_contents;
    }
    else {
      resolved.middle_tower_contents = 0
    }

    if (msg.right_tower_contents !== undefined) {
      resolved.right_tower_contents = msg.right_tower_contents;
    }
    else {
      resolved.right_tower_contents = 0
    }

    return resolved;
    }
};

module.exports = drop_status;
