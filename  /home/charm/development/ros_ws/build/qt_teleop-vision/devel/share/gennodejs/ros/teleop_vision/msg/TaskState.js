// Auto-generated. Do not edit!

// (in-package teleop_vision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class TaskState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.task_name = null;
      this.task_state = null;
      this.number_of_repetition = null;
      this.time_stamp = null;
      this.position_error_norm = null;
    }
    else {
      if (initObj.hasOwnProperty('task_name')) {
        this.task_name = initObj.task_name
      }
      else {
        this.task_name = '';
      }
      if (initObj.hasOwnProperty('task_state')) {
        this.task_state = initObj.task_state
      }
      else {
        this.task_state = 0;
      }
      if (initObj.hasOwnProperty('number_of_repetition')) {
        this.number_of_repetition = initObj.number_of_repetition
      }
      else {
        this.number_of_repetition = 0;
      }
      if (initObj.hasOwnProperty('time_stamp')) {
        this.time_stamp = initObj.time_stamp
      }
      else {
        this.time_stamp = 0.0;
      }
      if (initObj.hasOwnProperty('position_error_norm')) {
        this.position_error_norm = initObj.position_error_norm
      }
      else {
        this.position_error_norm = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TaskState
    // Serialize message field [task_name]
    bufferOffset = _serializer.string(obj.task_name, buffer, bufferOffset);
    // Serialize message field [task_state]
    bufferOffset = _serializer.uint8(obj.task_state, buffer, bufferOffset);
    // Serialize message field [number_of_repetition]
    bufferOffset = _serializer.uint8(obj.number_of_repetition, buffer, bufferOffset);
    // Serialize message field [time_stamp]
    bufferOffset = _serializer.float64(obj.time_stamp, buffer, bufferOffset);
    // Serialize message field [position_error_norm]
    bufferOffset = _serializer.float64(obj.position_error_norm, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TaskState
    let len;
    let data = new TaskState(null);
    // Deserialize message field [task_name]
    data.task_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [task_state]
    data.task_state = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [number_of_repetition]
    data.number_of_repetition = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [time_stamp]
    data.time_stamp = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [position_error_norm]
    data.position_error_norm = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.task_name.length;
    return length + 22;
  }

  static datatype() {
    // Returns string type for a message object
    return 'teleop_vision/TaskState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '88680b1f4b0d4199c729e843287035e4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string task_name
    uint8 task_state
    uint8 number_of_repetition
    float64 time_stamp
    float64 position_error_norm
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TaskState(null);
    if (msg.task_name !== undefined) {
      resolved.task_name = msg.task_name;
    }
    else {
      resolved.task_name = ''
    }

    if (msg.task_state !== undefined) {
      resolved.task_state = msg.task_state;
    }
    else {
      resolved.task_state = 0
    }

    if (msg.number_of_repetition !== undefined) {
      resolved.number_of_repetition = msg.number_of_repetition;
    }
    else {
      resolved.number_of_repetition = 0
    }

    if (msg.time_stamp !== undefined) {
      resolved.time_stamp = msg.time_stamp;
    }
    else {
      resolved.time_stamp = 0.0
    }

    if (msg.position_error_norm !== undefined) {
      resolved.position_error_norm = msg.position_error_norm;
    }
    else {
      resolved.position_error_norm = 0.0
    }

    return resolved;
    }
};

module.exports = TaskState;
