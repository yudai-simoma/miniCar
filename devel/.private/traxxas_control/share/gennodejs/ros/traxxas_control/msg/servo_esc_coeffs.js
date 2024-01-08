// Auto-generated. Do not edit!

// (in-package traxxas_control.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class servo_esc_coeffs {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.servo_neutral = null;
      this.esc_neutral = null;
      this.angle_mult = null;
      this.throt_mult = null;
    }
    else {
      if (initObj.hasOwnProperty('servo_neutral')) {
        this.servo_neutral = initObj.servo_neutral
      }
      else {
        this.servo_neutral = 0;
      }
      if (initObj.hasOwnProperty('esc_neutral')) {
        this.esc_neutral = initObj.esc_neutral
      }
      else {
        this.esc_neutral = 0;
      }
      if (initObj.hasOwnProperty('angle_mult')) {
        this.angle_mult = initObj.angle_mult
      }
      else {
        this.angle_mult = 0;
      }
      if (initObj.hasOwnProperty('throt_mult')) {
        this.throt_mult = initObj.throt_mult
      }
      else {
        this.throt_mult = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type servo_esc_coeffs
    // Serialize message field [servo_neutral]
    bufferOffset = _serializer.int32(obj.servo_neutral, buffer, bufferOffset);
    // Serialize message field [esc_neutral]
    bufferOffset = _serializer.int32(obj.esc_neutral, buffer, bufferOffset);
    // Serialize message field [angle_mult]
    bufferOffset = _serializer.int32(obj.angle_mult, buffer, bufferOffset);
    // Serialize message field [throt_mult]
    bufferOffset = _serializer.int32(obj.throt_mult, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type servo_esc_coeffs
    let len;
    let data = new servo_esc_coeffs(null);
    // Deserialize message field [servo_neutral]
    data.servo_neutral = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [esc_neutral]
    data.esc_neutral = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [angle_mult]
    data.angle_mult = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [throt_mult]
    data.throt_mult = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'traxxas_control/servo_esc_coeffs';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2d202238d5f284960838988ffb4a6570';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 servo_neutral
    int32 esc_neutral
    
    int32 angle_mult
    int32 throt_mult
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new servo_esc_coeffs(null);
    if (msg.servo_neutral !== undefined) {
      resolved.servo_neutral = msg.servo_neutral;
    }
    else {
      resolved.servo_neutral = 0
    }

    if (msg.esc_neutral !== undefined) {
      resolved.esc_neutral = msg.esc_neutral;
    }
    else {
      resolved.esc_neutral = 0
    }

    if (msg.angle_mult !== undefined) {
      resolved.angle_mult = msg.angle_mult;
    }
    else {
      resolved.angle_mult = 0
    }

    if (msg.throt_mult !== undefined) {
      resolved.throt_mult = msg.throt_mult;
    }
    else {
      resolved.throt_mult = 0
    }

    return resolved;
    }
};

module.exports = servo_esc_coeffs;
