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

class grabber_motors {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.z_pos_mm = null;
      this.open_pos_mm = null;
      this.z_twist_rad = null;
      this.servo_state = null;
    }
    else {
      if (initObj.hasOwnProperty('z_pos_mm')) {
        this.z_pos_mm = initObj.z_pos_mm
      }
      else {
        this.z_pos_mm = 0.0;
      }
      if (initObj.hasOwnProperty('open_pos_mm')) {
        this.open_pos_mm = initObj.open_pos_mm
      }
      else {
        this.open_pos_mm = 0.0;
      }
      if (initObj.hasOwnProperty('z_twist_rad')) {
        this.z_twist_rad = initObj.z_twist_rad
      }
      else {
        this.z_twist_rad = 0.0;
      }
      if (initObj.hasOwnProperty('servo_state')) {
        this.servo_state = initObj.servo_state
      }
      else {
        this.servo_state = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type grabber_motors
    // Serialize message field [z_pos_mm]
    bufferOffset = _serializer.float32(obj.z_pos_mm, buffer, bufferOffset);
    // Serialize message field [open_pos_mm]
    bufferOffset = _serializer.float32(obj.open_pos_mm, buffer, bufferOffset);
    // Serialize message field [z_twist_rad]
    bufferOffset = _serializer.float32(obj.z_twist_rad, buffer, bufferOffset);
    // Serialize message field [servo_state]
    bufferOffset = _serializer.bool(obj.servo_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type grabber_motors
    let len;
    let data = new grabber_motors(null);
    // Deserialize message field [z_pos_mm]
    data.z_pos_mm = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [open_pos_mm]
    data.open_pos_mm = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z_twist_rad]
    data.z_twist_rad = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [servo_state]
    data.servo_state = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'eurobot2019_messages/grabber_motors';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1cacf4e8a711d50ed3a69eafde8b2ff7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 z_pos_mm
    float32 open_pos_mm
    float32 z_twist_rad
    bool servo_state
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new grabber_motors(null);
    if (msg.z_pos_mm !== undefined) {
      resolved.z_pos_mm = msg.z_pos_mm;
    }
    else {
      resolved.z_pos_mm = 0.0
    }

    if (msg.open_pos_mm !== undefined) {
      resolved.open_pos_mm = msg.open_pos_mm;
    }
    else {
      resolved.open_pos_mm = 0.0
    }

    if (msg.z_twist_rad !== undefined) {
      resolved.z_twist_rad = msg.z_twist_rad;
    }
    else {
      resolved.z_twist_rad = 0.0
    }

    if (msg.servo_state !== undefined) {
      resolved.servo_state = msg.servo_state;
    }
    else {
      resolved.servo_state = false
    }

    return resolved;
    }
};

module.exports = grabber_motors;
