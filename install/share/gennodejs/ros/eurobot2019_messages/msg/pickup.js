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

class pickup {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.is_vertical = null;
      this.pos_reached = null;
      this.colour = null;
    }
    else {
      if (initObj.hasOwnProperty('is_vertical')) {
        this.is_vertical = initObj.is_vertical
      }
      else {
        this.is_vertical = false;
      }
      if (initObj.hasOwnProperty('pos_reached')) {
        this.pos_reached = initObj.pos_reached
      }
      else {
        this.pos_reached = false;
      }
      if (initObj.hasOwnProperty('colour')) {
        this.colour = initObj.colour
      }
      else {
        this.colour = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pickup
    // Serialize message field [is_vertical]
    bufferOffset = _serializer.bool(obj.is_vertical, buffer, bufferOffset);
    // Serialize message field [pos_reached]
    bufferOffset = _serializer.bool(obj.pos_reached, buffer, bufferOffset);
    // Serialize message field [colour]
    bufferOffset = _serializer.uint8(obj.colour, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pickup
    let len;
    let data = new pickup(null);
    // Deserialize message field [is_vertical]
    data.is_vertical = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [pos_reached]
    data.pos_reached = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [colour]
    data.colour = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 3;
  }

  static datatype() {
    // Returns string type for a message object
    return 'eurobot2019_messages/pickup';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fc43b8165e5ce088844c3951cd1006fd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Is the target puck vertical?
    bool is_vertical
    
    # Have we reached the correct position yet?
    # Note that the picker will be told to open up and get to the right z
    # Before continuing by setting this to false
    bool pos_reached
    
    # 0 is "idle at bottom front"
    # 1 is red
    # 2 is green
    # 3 is blue
    # 4 is gold
    # 5 is "idle at top back"
    uint8 colour
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new pickup(null);
    if (msg.is_vertical !== undefined) {
      resolved.is_vertical = msg.is_vertical;
    }
    else {
      resolved.is_vertical = false
    }

    if (msg.pos_reached !== undefined) {
      resolved.pos_reached = msg.pos_reached;
    }
    else {
      resolved.pos_reached = false
    }

    if (msg.colour !== undefined) {
      resolved.colour = msg.colour;
    }
    else {
      resolved.colour = 0
    }

    return resolved;
    }
};

module.exports = pickup;
