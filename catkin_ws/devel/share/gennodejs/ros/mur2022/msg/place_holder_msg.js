// Auto-generated. Do not edit!

// (in-package mur2022.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class place_holder_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.temp_bool = null;
      this.temp_uint32 = null;
      this.temp_int32 = null;
      this.temp_float32 = null;
      this.temp_float64 = null;
      this.temp_string = null;
      this.temp_float64_array = null;
    }
    else {
      if (initObj.hasOwnProperty('temp_bool')) {
        this.temp_bool = initObj.temp_bool
      }
      else {
        this.temp_bool = false;
      }
      if (initObj.hasOwnProperty('temp_uint32')) {
        this.temp_uint32 = initObj.temp_uint32
      }
      else {
        this.temp_uint32 = 0;
      }
      if (initObj.hasOwnProperty('temp_int32')) {
        this.temp_int32 = initObj.temp_int32
      }
      else {
        this.temp_int32 = 0;
      }
      if (initObj.hasOwnProperty('temp_float32')) {
        this.temp_float32 = initObj.temp_float32
      }
      else {
        this.temp_float32 = 0.0;
      }
      if (initObj.hasOwnProperty('temp_float64')) {
        this.temp_float64 = initObj.temp_float64
      }
      else {
        this.temp_float64 = 0.0;
      }
      if (initObj.hasOwnProperty('temp_string')) {
        this.temp_string = initObj.temp_string
      }
      else {
        this.temp_string = '';
      }
      if (initObj.hasOwnProperty('temp_float64_array')) {
        this.temp_float64_array = initObj.temp_float64_array
      }
      else {
        this.temp_float64_array = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type place_holder_msg
    // Serialize message field [temp_bool]
    bufferOffset = _serializer.bool(obj.temp_bool, buffer, bufferOffset);
    // Serialize message field [temp_uint32]
    bufferOffset = _serializer.uint32(obj.temp_uint32, buffer, bufferOffset);
    // Serialize message field [temp_int32]
    bufferOffset = _serializer.int32(obj.temp_int32, buffer, bufferOffset);
    // Serialize message field [temp_float32]
    bufferOffset = _serializer.float32(obj.temp_float32, buffer, bufferOffset);
    // Serialize message field [temp_float64]
    bufferOffset = _serializer.float64(obj.temp_float64, buffer, bufferOffset);
    // Serialize message field [temp_string]
    bufferOffset = _serializer.string(obj.temp_string, buffer, bufferOffset);
    // Serialize message field [temp_float64_array]
    bufferOffset = _arraySerializer.float64(obj.temp_float64_array, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type place_holder_msg
    let len;
    let data = new place_holder_msg(null);
    // Deserialize message field [temp_bool]
    data.temp_bool = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [temp_uint32]
    data.temp_uint32 = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [temp_int32]
    data.temp_int32 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [temp_float32]
    data.temp_float32 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [temp_float64]
    data.temp_float64 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [temp_string]
    data.temp_string = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [temp_float64_array]
    data.temp_float64_array = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.temp_string.length;
    length += 8 * object.temp_float64_array.length;
    return length + 29;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mur2022/place_holder_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '13548be3cae6b9d8d8453c21a93e0345';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool temp_bool
    uint32 temp_uint32
    int32 temp_int32
    float32 temp_float32
    float64 temp_float64
    string temp_string
    float64[] temp_float64_array
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new place_holder_msg(null);
    if (msg.temp_bool !== undefined) {
      resolved.temp_bool = msg.temp_bool;
    }
    else {
      resolved.temp_bool = false
    }

    if (msg.temp_uint32 !== undefined) {
      resolved.temp_uint32 = msg.temp_uint32;
    }
    else {
      resolved.temp_uint32 = 0
    }

    if (msg.temp_int32 !== undefined) {
      resolved.temp_int32 = msg.temp_int32;
    }
    else {
      resolved.temp_int32 = 0
    }

    if (msg.temp_float32 !== undefined) {
      resolved.temp_float32 = msg.temp_float32;
    }
    else {
      resolved.temp_float32 = 0.0
    }

    if (msg.temp_float64 !== undefined) {
      resolved.temp_float64 = msg.temp_float64;
    }
    else {
      resolved.temp_float64 = 0.0
    }

    if (msg.temp_string !== undefined) {
      resolved.temp_string = msg.temp_string;
    }
    else {
      resolved.temp_string = ''
    }

    if (msg.temp_float64_array !== undefined) {
      resolved.temp_float64_array = msg.temp_float64_array;
    }
    else {
      resolved.temp_float64_array = []
    }

    return resolved;
    }
};

module.exports = place_holder_msg;
