// Auto-generated. Do not edit!

// (in-package teleop_vision.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class CalculateStereoCamsTransfromFromTopicsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cam_1_pose_topic_name = null;
      this.cam_2_pose_topic_name = null;
    }
    else {
      if (initObj.hasOwnProperty('cam_1_pose_topic_name')) {
        this.cam_1_pose_topic_name = initObj.cam_1_pose_topic_name
      }
      else {
        this.cam_1_pose_topic_name = '';
      }
      if (initObj.hasOwnProperty('cam_2_pose_topic_name')) {
        this.cam_2_pose_topic_name = initObj.cam_2_pose_topic_name
      }
      else {
        this.cam_2_pose_topic_name = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CalculateStereoCamsTransfromFromTopicsRequest
    // Serialize message field [cam_1_pose_topic_name]
    bufferOffset = _serializer.string(obj.cam_1_pose_topic_name, buffer, bufferOffset);
    // Serialize message field [cam_2_pose_topic_name]
    bufferOffset = _serializer.string(obj.cam_2_pose_topic_name, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CalculateStereoCamsTransfromFromTopicsRequest
    let len;
    let data = new CalculateStereoCamsTransfromFromTopicsRequest(null);
    // Deserialize message field [cam_1_pose_topic_name]
    data.cam_1_pose_topic_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [cam_2_pose_topic_name]
    data.cam_2_pose_topic_name = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.cam_1_pose_topic_name.length;
    length += object.cam_2_pose_topic_name.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'teleop_vision/CalculateStereoCamsTransfromFromTopicsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ba287fa136b90cf933f718565b5b1486';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string cam_1_pose_topic_name
    string cam_2_pose_topic_name
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CalculateStereoCamsTransfromFromTopicsRequest(null);
    if (msg.cam_1_pose_topic_name !== undefined) {
      resolved.cam_1_pose_topic_name = msg.cam_1_pose_topic_name;
    }
    else {
      resolved.cam_1_pose_topic_name = ''
    }

    if (msg.cam_2_pose_topic_name !== undefined) {
      resolved.cam_2_pose_topic_name = msg.cam_2_pose_topic_name;
    }
    else {
      resolved.cam_2_pose_topic_name = ''
    }

    return resolved;
    }
};

class CalculateStereoCamsTransfromFromTopicsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cam_1_to_cam_2_pose = null;
    }
    else {
      if (initObj.hasOwnProperty('cam_1_to_cam_2_pose')) {
        this.cam_1_to_cam_2_pose = initObj.cam_1_to_cam_2_pose
      }
      else {
        this.cam_1_to_cam_2_pose = new geometry_msgs.msg.Pose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CalculateStereoCamsTransfromFromTopicsResponse
    // Serialize message field [cam_1_to_cam_2_pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.cam_1_to_cam_2_pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CalculateStereoCamsTransfromFromTopicsResponse
    let len;
    let data = new CalculateStereoCamsTransfromFromTopicsResponse(null);
    // Deserialize message field [cam_1_to_cam_2_pose]
    data.cam_1_to_cam_2_pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 56;
  }

  static datatype() {
    // Returns string type for a service object
    return 'teleop_vision/CalculateStereoCamsTransfromFromTopicsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1972cf9e36a60114f123cbe4d3fcc017';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Pose cam_1_to_cam_2_pose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CalculateStereoCamsTransfromFromTopicsResponse(null);
    if (msg.cam_1_to_cam_2_pose !== undefined) {
      resolved.cam_1_to_cam_2_pose = geometry_msgs.msg.Pose.Resolve(msg.cam_1_to_cam_2_pose)
    }
    else {
      resolved.cam_1_to_cam_2_pose = new geometry_msgs.msg.Pose()
    }

    return resolved;
    }
};

module.exports = {
  Request: CalculateStereoCamsTransfromFromTopicsRequest,
  Response: CalculateStereoCamsTransfromFromTopicsResponse,
  md5sum() { return '28012a097e6af14c818b4f0effba0194'; },
  datatype() { return 'teleop_vision/CalculateStereoCamsTransfromFromTopics'; }
};
