// Auto-generated. Do not edit!

// (in-package glider_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class imumsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.gpsStatus = null;
      this.latitude = null;
      this.longitude = null;
      this.altitude = null;
      this.linearX = null;
      this.linearY = null;
      this.linearZ = null;
      this.angularX = null;
      this.angularY = null;
      this.angularZ = null;
      this.orientationX = null;
      this.orientationY = null;
      this.orientationZ = null;
      this.orientationW = null;
      this.angularVeloX = null;
      this.angularVeloY = null;
      this.angularVeloZ = null;
      this.linearAccelerationX = null;
      this.linearAccelerationY = null;
      this.linearAccelerationZ = null;
      this.position_covariance = null;
      this.orientationCovariance = null;
      this.linearAccelerationCovariance = null;
      this.RPY = null;
      this.systemStatusMessage = null;
      this.statusLevel = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('gpsStatus')) {
        this.gpsStatus = initObj.gpsStatus
      }
      else {
        this.gpsStatus = 0;
      }
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
      if (initObj.hasOwnProperty('altitude')) {
        this.altitude = initObj.altitude
      }
      else {
        this.altitude = 0.0;
      }
      if (initObj.hasOwnProperty('linearX')) {
        this.linearX = initObj.linearX
      }
      else {
        this.linearX = 0.0;
      }
      if (initObj.hasOwnProperty('linearY')) {
        this.linearY = initObj.linearY
      }
      else {
        this.linearY = 0.0;
      }
      if (initObj.hasOwnProperty('linearZ')) {
        this.linearZ = initObj.linearZ
      }
      else {
        this.linearZ = 0.0;
      }
      if (initObj.hasOwnProperty('angularX')) {
        this.angularX = initObj.angularX
      }
      else {
        this.angularX = 0.0;
      }
      if (initObj.hasOwnProperty('angularY')) {
        this.angularY = initObj.angularY
      }
      else {
        this.angularY = 0.0;
      }
      if (initObj.hasOwnProperty('angularZ')) {
        this.angularZ = initObj.angularZ
      }
      else {
        this.angularZ = 0.0;
      }
      if (initObj.hasOwnProperty('orientationX')) {
        this.orientationX = initObj.orientationX
      }
      else {
        this.orientationX = 0.0;
      }
      if (initObj.hasOwnProperty('orientationY')) {
        this.orientationY = initObj.orientationY
      }
      else {
        this.orientationY = 0.0;
      }
      if (initObj.hasOwnProperty('orientationZ')) {
        this.orientationZ = initObj.orientationZ
      }
      else {
        this.orientationZ = 0.0;
      }
      if (initObj.hasOwnProperty('orientationW')) {
        this.orientationW = initObj.orientationW
      }
      else {
        this.orientationW = 0.0;
      }
      if (initObj.hasOwnProperty('angularVeloX')) {
        this.angularVeloX = initObj.angularVeloX
      }
      else {
        this.angularVeloX = 0.0;
      }
      if (initObj.hasOwnProperty('angularVeloY')) {
        this.angularVeloY = initObj.angularVeloY
      }
      else {
        this.angularVeloY = 0.0;
      }
      if (initObj.hasOwnProperty('angularVeloZ')) {
        this.angularVeloZ = initObj.angularVeloZ
      }
      else {
        this.angularVeloZ = 0.0;
      }
      if (initObj.hasOwnProperty('linearAccelerationX')) {
        this.linearAccelerationX = initObj.linearAccelerationX
      }
      else {
        this.linearAccelerationX = 0.0;
      }
      if (initObj.hasOwnProperty('linearAccelerationY')) {
        this.linearAccelerationY = initObj.linearAccelerationY
      }
      else {
        this.linearAccelerationY = 0.0;
      }
      if (initObj.hasOwnProperty('linearAccelerationZ')) {
        this.linearAccelerationZ = initObj.linearAccelerationZ
      }
      else {
        this.linearAccelerationZ = 0.0;
      }
      if (initObj.hasOwnProperty('position_covariance')) {
        this.position_covariance = initObj.position_covariance
      }
      else {
        this.position_covariance = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('orientationCovariance')) {
        this.orientationCovariance = initObj.orientationCovariance
      }
      else {
        this.orientationCovariance = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('linearAccelerationCovariance')) {
        this.linearAccelerationCovariance = initObj.linearAccelerationCovariance
      }
      else {
        this.linearAccelerationCovariance = new Array(9).fill(0);
      }
      if (initObj.hasOwnProperty('RPY')) {
        this.RPY = initObj.RPY
      }
      else {
        this.RPY = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('systemStatusMessage')) {
        this.systemStatusMessage = initObj.systemStatusMessage
      }
      else {
        this.systemStatusMessage = '';
      }
      if (initObj.hasOwnProperty('statusLevel')) {
        this.statusLevel = initObj.statusLevel
      }
      else {
        this.statusLevel = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type imumsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [gpsStatus]
    bufferOffset = _serializer.int16(obj.gpsStatus, buffer, bufferOffset);
    // Serialize message field [latitude]
    bufferOffset = _serializer.float32(obj.latitude, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float32(obj.longitude, buffer, bufferOffset);
    // Serialize message field [altitude]
    bufferOffset = _serializer.float32(obj.altitude, buffer, bufferOffset);
    // Serialize message field [linearX]
    bufferOffset = _serializer.float32(obj.linearX, buffer, bufferOffset);
    // Serialize message field [linearY]
    bufferOffset = _serializer.float32(obj.linearY, buffer, bufferOffset);
    // Serialize message field [linearZ]
    bufferOffset = _serializer.float32(obj.linearZ, buffer, bufferOffset);
    // Serialize message field [angularX]
    bufferOffset = _serializer.float32(obj.angularX, buffer, bufferOffset);
    // Serialize message field [angularY]
    bufferOffset = _serializer.float32(obj.angularY, buffer, bufferOffset);
    // Serialize message field [angularZ]
    bufferOffset = _serializer.float32(obj.angularZ, buffer, bufferOffset);
    // Serialize message field [orientationX]
    bufferOffset = _serializer.float32(obj.orientationX, buffer, bufferOffset);
    // Serialize message field [orientationY]
    bufferOffset = _serializer.float32(obj.orientationY, buffer, bufferOffset);
    // Serialize message field [orientationZ]
    bufferOffset = _serializer.float32(obj.orientationZ, buffer, bufferOffset);
    // Serialize message field [orientationW]
    bufferOffset = _serializer.float32(obj.orientationW, buffer, bufferOffset);
    // Serialize message field [angularVeloX]
    bufferOffset = _serializer.float32(obj.angularVeloX, buffer, bufferOffset);
    // Serialize message field [angularVeloY]
    bufferOffset = _serializer.float32(obj.angularVeloY, buffer, bufferOffset);
    // Serialize message field [angularVeloZ]
    bufferOffset = _serializer.float32(obj.angularVeloZ, buffer, bufferOffset);
    // Serialize message field [linearAccelerationX]
    bufferOffset = _serializer.float32(obj.linearAccelerationX, buffer, bufferOffset);
    // Serialize message field [linearAccelerationY]
    bufferOffset = _serializer.float32(obj.linearAccelerationY, buffer, bufferOffset);
    // Serialize message field [linearAccelerationZ]
    bufferOffset = _serializer.float32(obj.linearAccelerationZ, buffer, bufferOffset);
    // Check that the constant length array field [position_covariance] has the right length
    if (obj.position_covariance.length !== 9) {
      throw new Error('Unable to serialize array field position_covariance - length must be 9')
    }
    // Serialize message field [position_covariance]
    bufferOffset = _arraySerializer.float64(obj.position_covariance, buffer, bufferOffset, 9);
    // Check that the constant length array field [orientationCovariance] has the right length
    if (obj.orientationCovariance.length !== 9) {
      throw new Error('Unable to serialize array field orientationCovariance - length must be 9')
    }
    // Serialize message field [orientationCovariance]
    bufferOffset = _arraySerializer.float64(obj.orientationCovariance, buffer, bufferOffset, 9);
    // Check that the constant length array field [linearAccelerationCovariance] has the right length
    if (obj.linearAccelerationCovariance.length !== 9) {
      throw new Error('Unable to serialize array field linearAccelerationCovariance - length must be 9')
    }
    // Serialize message field [linearAccelerationCovariance]
    bufferOffset = _arraySerializer.float64(obj.linearAccelerationCovariance, buffer, bufferOffset, 9);
    // Check that the constant length array field [RPY] has the right length
    if (obj.RPY.length !== 3) {
      throw new Error('Unable to serialize array field RPY - length must be 3')
    }
    // Serialize message field [RPY]
    bufferOffset = _arraySerializer.float32(obj.RPY, buffer, bufferOffset, 3);
    // Serialize message field [systemStatusMessage]
    bufferOffset = _serializer.string(obj.systemStatusMessage, buffer, bufferOffset);
    // Serialize message field [statusLevel]
    bufferOffset = _serializer.int16(obj.statusLevel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type imumsg
    let len;
    let data = new imumsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [gpsStatus]
    data.gpsStatus = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [altitude]
    data.altitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [linearX]
    data.linearX = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [linearY]
    data.linearY = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [linearZ]
    data.linearZ = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angularX]
    data.angularX = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angularY]
    data.angularY = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angularZ]
    data.angularZ = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [orientationX]
    data.orientationX = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [orientationY]
    data.orientationY = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [orientationZ]
    data.orientationZ = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [orientationW]
    data.orientationW = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angularVeloX]
    data.angularVeloX = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angularVeloY]
    data.angularVeloY = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angularVeloZ]
    data.angularVeloZ = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [linearAccelerationX]
    data.linearAccelerationX = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [linearAccelerationY]
    data.linearAccelerationY = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [linearAccelerationZ]
    data.linearAccelerationZ = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [position_covariance]
    data.position_covariance = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    // Deserialize message field [orientationCovariance]
    data.orientationCovariance = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    // Deserialize message field [linearAccelerationCovariance]
    data.linearAccelerationCovariance = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    // Deserialize message field [RPY]
    data.RPY = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [systemStatusMessage]
    data.systemStatusMessage = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [statusLevel]
    data.statusLevel = _deserializer.int16(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.systemStatusMessage.length;
    return length + 312;
  }

  static datatype() {
    // Returns string type for a message object
    return 'glider_msg/imumsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd2f0d176490184beac4b63cba6827925';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    int16 gpsStatus
    float32 latitude
    float32 longitude
    float32 altitude
    float32 linearX
    float32 linearY
    float32 linearZ
    float32 angularX
    float32 angularY
    float32 angularZ
    float32 orientationX
    float32 orientationY
    float32 orientationZ
    float32 orientationW
    float32 angularVeloX
    float32 angularVeloY
    float32 angularVeloZ
    float32 linearAccelerationX
    float32 linearAccelerationY
    float32 linearAccelerationZ
    float64[9] position_covariance
    float64[9] orientationCovariance
    float64[9] linearAccelerationCovariance
    float32[3] RPY
    string systemStatusMessage
    int16 statusLevel
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new imumsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.gpsStatus !== undefined) {
      resolved.gpsStatus = msg.gpsStatus;
    }
    else {
      resolved.gpsStatus = 0
    }

    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    if (msg.altitude !== undefined) {
      resolved.altitude = msg.altitude;
    }
    else {
      resolved.altitude = 0.0
    }

    if (msg.linearX !== undefined) {
      resolved.linearX = msg.linearX;
    }
    else {
      resolved.linearX = 0.0
    }

    if (msg.linearY !== undefined) {
      resolved.linearY = msg.linearY;
    }
    else {
      resolved.linearY = 0.0
    }

    if (msg.linearZ !== undefined) {
      resolved.linearZ = msg.linearZ;
    }
    else {
      resolved.linearZ = 0.0
    }

    if (msg.angularX !== undefined) {
      resolved.angularX = msg.angularX;
    }
    else {
      resolved.angularX = 0.0
    }

    if (msg.angularY !== undefined) {
      resolved.angularY = msg.angularY;
    }
    else {
      resolved.angularY = 0.0
    }

    if (msg.angularZ !== undefined) {
      resolved.angularZ = msg.angularZ;
    }
    else {
      resolved.angularZ = 0.0
    }

    if (msg.orientationX !== undefined) {
      resolved.orientationX = msg.orientationX;
    }
    else {
      resolved.orientationX = 0.0
    }

    if (msg.orientationY !== undefined) {
      resolved.orientationY = msg.orientationY;
    }
    else {
      resolved.orientationY = 0.0
    }

    if (msg.orientationZ !== undefined) {
      resolved.orientationZ = msg.orientationZ;
    }
    else {
      resolved.orientationZ = 0.0
    }

    if (msg.orientationW !== undefined) {
      resolved.orientationW = msg.orientationW;
    }
    else {
      resolved.orientationW = 0.0
    }

    if (msg.angularVeloX !== undefined) {
      resolved.angularVeloX = msg.angularVeloX;
    }
    else {
      resolved.angularVeloX = 0.0
    }

    if (msg.angularVeloY !== undefined) {
      resolved.angularVeloY = msg.angularVeloY;
    }
    else {
      resolved.angularVeloY = 0.0
    }

    if (msg.angularVeloZ !== undefined) {
      resolved.angularVeloZ = msg.angularVeloZ;
    }
    else {
      resolved.angularVeloZ = 0.0
    }

    if (msg.linearAccelerationX !== undefined) {
      resolved.linearAccelerationX = msg.linearAccelerationX;
    }
    else {
      resolved.linearAccelerationX = 0.0
    }

    if (msg.linearAccelerationY !== undefined) {
      resolved.linearAccelerationY = msg.linearAccelerationY;
    }
    else {
      resolved.linearAccelerationY = 0.0
    }

    if (msg.linearAccelerationZ !== undefined) {
      resolved.linearAccelerationZ = msg.linearAccelerationZ;
    }
    else {
      resolved.linearAccelerationZ = 0.0
    }

    if (msg.position_covariance !== undefined) {
      resolved.position_covariance = msg.position_covariance;
    }
    else {
      resolved.position_covariance = new Array(9).fill(0)
    }

    if (msg.orientationCovariance !== undefined) {
      resolved.orientationCovariance = msg.orientationCovariance;
    }
    else {
      resolved.orientationCovariance = new Array(9).fill(0)
    }

    if (msg.linearAccelerationCovariance !== undefined) {
      resolved.linearAccelerationCovariance = msg.linearAccelerationCovariance;
    }
    else {
      resolved.linearAccelerationCovariance = new Array(9).fill(0)
    }

    if (msg.RPY !== undefined) {
      resolved.RPY = msg.RPY;
    }
    else {
      resolved.RPY = new Array(3).fill(0)
    }

    if (msg.systemStatusMessage !== undefined) {
      resolved.systemStatusMessage = msg.systemStatusMessage;
    }
    else {
      resolved.systemStatusMessage = ''
    }

    if (msg.statusLevel !== undefined) {
      resolved.statusLevel = msg.statusLevel;
    }
    else {
      resolved.statusLevel = 0
    }

    return resolved;
    }
};

module.exports = imumsg;
