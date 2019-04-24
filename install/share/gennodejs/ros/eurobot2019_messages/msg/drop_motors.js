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

class drop_motors {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.left_z = null;
      this.left_x = null;
      this.middle_z = null;
      this.middle_x = null;
      this.right_z = null;
      this.right_x = null;
    }
    else {
      if (initObj.hasOwnProperty('left_z')) {
        this.left_z = initObj.left_z
      }
      else {
        this.left_z = 0.0;
      }
      if (initObj.hasOwnProperty('left_x')) {
        this.left_x = initObj.left_x
      }
      else {
        this.left_x = 0.0;
      }
      if (initObj.hasOwnProperty('middle_z')) {
        this.middle_z = initObj.middle_z
      }
      else {
        this.middle_z = 0.0;
      }
      if (initObj.hasOwnProperty('middle_x')) {
        this.middle_x = initObj.middle_x
      }
      else {
        this.middle_x = 0.0;
      }
      if (initObj.hasOwnProperty('right_z')) {
        this.right_z = initObj.right_z
      }
      else {
        this.right_z = 0.0;
      }
      if (initObj.hasOwnProperty('right_x')) {
        this.right_x = initObj.right_x
      }
      else {
        this.right_x = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type drop_motors
    // Serialize message field [left_z]
    bufferOffset = _serializer.float32(obj.left_z, buffer, bufferOffset);
    // Serialize message field [left_x]
    bufferOffset = _serializer.float32(obj.left_x, buffer, bufferOffset);
    // Serialize message field [middle_z]
    bufferOffset = _serializer.float32(obj.middle_z, buffer, bufferOffset);
    // Serialize message field [middle_x]
    bufferOffset = _serializer.float32(obj.middle_x, buffer, bufferOffset);
    // Serialize message field [right_z]
    bufferOffset = _serializer.float32(obj.right_z, buffer, bufferOffset);
    // Serialize message field [right_x]
    bufferOffset = _serializer.float32(obj.right_x, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type drop_motors
    let len;
    let data = new drop_motors(null);
    // Deserialize message field [left_z]
    data.left_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_x]
    data.left_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [middle_z]
    data.middle_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [middle_x]
    data.middle_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_z]
    data.right_z = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_x]
    data.right_x = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'eurobot2019_messages/drop_motors';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'da232c9309df34c64bd87bc76c440433';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # The z position of the left stepper motor
    float32 left_z
    # The x position of the left pusher
    float32 left_x
    # The z position of the middle stepper motor
    float32 middle_z
    # The x position of the middle pusher
    float32 middle_x
    # The z position of the right stepper motor
    float32 right_z
    # The x position of the right pusher
    float32 right_x
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new drop_motors(null);
    if (msg.left_z !== undefined) {
      resolved.left_z = msg.left_z;
    }
    else {
      resolved.left_z = 0.0
    }

    if (msg.left_x !== undefined) {
      resolved.left_x = msg.left_x;
    }
    else {
      resolved.left_x = 0.0
    }

    if (msg.middle_z !== undefined) {
      resolved.middle_z = msg.middle_z;
    }
    else {
      resolved.middle_z = 0.0
    }

    if (msg.middle_x !== undefined) {
      resolved.middle_x = msg.middle_x;
    }
    else {
      resolved.middle_x = 0.0
    }

    if (msg.right_z !== undefined) {
      resolved.right_z = msg.right_z;
    }
    else {
      resolved.right_z = 0.0
    }

    if (msg.right_x !== undefined) {
      resolved.right_x = msg.right_x;
    }
    else {
      resolved.right_x = 0.0
    }

    return resolved;
    }
};

module.exports = drop_motors;
