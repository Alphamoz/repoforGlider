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

class dvlmsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.errCode = null;
      this.dataGood = null;
      this.altitudeBeam = null;
      this.bottomVelocityBeam = null;
      this.waterVelocityBeam = null;
      this.waterVelocityCredit = null;
      this.velocityInst = null;
      this.velocityInstFlag = null;
      this.velocityEarth = null;
      this.velocityEarthFlag = null;
      this.waterVelocityInst = null;
      this.waterVelocityInstFlag = null;
      this.waterVelocityEarth = null;
      this.waterVelocityEarthFlag = null;
      this.roll = null;
      this.pitch = null;
      this.heading = null;
      this.altitude = null;
      this.temperature = null;
      this.pressure = null;
      this.salinity = null;
      this.soundSpeed = null;
      this.rawData = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('errCode')) {
        this.errCode = initObj.errCode
      }
      else {
        this.errCode = '';
      }
      if (initObj.hasOwnProperty('dataGood')) {
        this.dataGood = initObj.dataGood
      }
      else {
        this.dataGood = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('altitudeBeam')) {
        this.altitudeBeam = initObj.altitudeBeam
      }
      else {
        this.altitudeBeam = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('bottomVelocityBeam')) {
        this.bottomVelocityBeam = initObj.bottomVelocityBeam
      }
      else {
        this.bottomVelocityBeam = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('waterVelocityBeam')) {
        this.waterVelocityBeam = initObj.waterVelocityBeam
      }
      else {
        this.waterVelocityBeam = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('waterVelocityCredit')) {
        this.waterVelocityCredit = initObj.waterVelocityCredit
      }
      else {
        this.waterVelocityCredit = new Array(4).fill(0);
      }
      if (initObj.hasOwnProperty('velocityInst')) {
        this.velocityInst = initObj.velocityInst
      }
      else {
        this.velocityInst = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('velocityInstFlag')) {
        this.velocityInstFlag = initObj.velocityInstFlag
      }
      else {
        this.velocityInstFlag = 0;
      }
      if (initObj.hasOwnProperty('velocityEarth')) {
        this.velocityEarth = initObj.velocityEarth
      }
      else {
        this.velocityEarth = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('velocityEarthFlag')) {
        this.velocityEarthFlag = initObj.velocityEarthFlag
      }
      else {
        this.velocityEarthFlag = 0;
      }
      if (initObj.hasOwnProperty('waterVelocityInst')) {
        this.waterVelocityInst = initObj.waterVelocityInst
      }
      else {
        this.waterVelocityInst = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('waterVelocityInstFlag')) {
        this.waterVelocityInstFlag = initObj.waterVelocityInstFlag
      }
      else {
        this.waterVelocityInstFlag = 0;
      }
      if (initObj.hasOwnProperty('waterVelocityEarth')) {
        this.waterVelocityEarth = initObj.waterVelocityEarth
      }
      else {
        this.waterVelocityEarth = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('waterVelocityEarthFlag')) {
        this.waterVelocityEarthFlag = initObj.waterVelocityEarthFlag
      }
      else {
        this.waterVelocityEarthFlag = 0;
      }
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0.0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('heading')) {
        this.heading = initObj.heading
      }
      else {
        this.heading = 0.0;
      }
      if (initObj.hasOwnProperty('altitude')) {
        this.altitude = initObj.altitude
      }
      else {
        this.altitude = 0.0;
      }
      if (initObj.hasOwnProperty('temperature')) {
        this.temperature = initObj.temperature
      }
      else {
        this.temperature = 0.0;
      }
      if (initObj.hasOwnProperty('pressure')) {
        this.pressure = initObj.pressure
      }
      else {
        this.pressure = 0.0;
      }
      if (initObj.hasOwnProperty('salinity')) {
        this.salinity = initObj.salinity
      }
      else {
        this.salinity = 0.0;
      }
      if (initObj.hasOwnProperty('soundSpeed')) {
        this.soundSpeed = initObj.soundSpeed
      }
      else {
        this.soundSpeed = 0.0;
      }
      if (initObj.hasOwnProperty('rawData')) {
        this.rawData = initObj.rawData
      }
      else {
        this.rawData = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type dvlmsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [errCode]
    bufferOffset = _serializer.string(obj.errCode, buffer, bufferOffset);
    // Check that the constant length array field [dataGood] has the right length
    if (obj.dataGood.length !== 4) {
      throw new Error('Unable to serialize array field dataGood - length must be 4')
    }
    // Serialize message field [dataGood]
    bufferOffset = _arraySerializer.int16(obj.dataGood, buffer, bufferOffset, 4);
    // Check that the constant length array field [altitudeBeam] has the right length
    if (obj.altitudeBeam.length !== 4) {
      throw new Error('Unable to serialize array field altitudeBeam - length must be 4')
    }
    // Serialize message field [altitudeBeam]
    bufferOffset = _arraySerializer.float32(obj.altitudeBeam, buffer, bufferOffset, 4);
    // Check that the constant length array field [bottomVelocityBeam] has the right length
    if (obj.bottomVelocityBeam.length !== 4) {
      throw new Error('Unable to serialize array field bottomVelocityBeam - length must be 4')
    }
    // Serialize message field [bottomVelocityBeam]
    bufferOffset = _arraySerializer.float32(obj.bottomVelocityBeam, buffer, bufferOffset, 4);
    // Check that the constant length array field [waterVelocityBeam] has the right length
    if (obj.waterVelocityBeam.length !== 4) {
      throw new Error('Unable to serialize array field waterVelocityBeam - length must be 4')
    }
    // Serialize message field [waterVelocityBeam]
    bufferOffset = _arraySerializer.float32(obj.waterVelocityBeam, buffer, bufferOffset, 4);
    // Check that the constant length array field [waterVelocityCredit] has the right length
    if (obj.waterVelocityCredit.length !== 4) {
      throw new Error('Unable to serialize array field waterVelocityCredit - length must be 4')
    }
    // Serialize message field [waterVelocityCredit]
    bufferOffset = _arraySerializer.float32(obj.waterVelocityCredit, buffer, bufferOffset, 4);
    // Check that the constant length array field [velocityInst] has the right length
    if (obj.velocityInst.length !== 3) {
      throw new Error('Unable to serialize array field velocityInst - length must be 3')
    }
    // Serialize message field [velocityInst]
    bufferOffset = _arraySerializer.float32(obj.velocityInst, buffer, bufferOffset, 3);
    // Serialize message field [velocityInstFlag]
    bufferOffset = _serializer.int16(obj.velocityInstFlag, buffer, bufferOffset);
    // Check that the constant length array field [velocityEarth] has the right length
    if (obj.velocityEarth.length !== 3) {
      throw new Error('Unable to serialize array field velocityEarth - length must be 3')
    }
    // Serialize message field [velocityEarth]
    bufferOffset = _arraySerializer.float32(obj.velocityEarth, buffer, bufferOffset, 3);
    // Serialize message field [velocityEarthFlag]
    bufferOffset = _serializer.int16(obj.velocityEarthFlag, buffer, bufferOffset);
    // Check that the constant length array field [waterVelocityInst] has the right length
    if (obj.waterVelocityInst.length !== 3) {
      throw new Error('Unable to serialize array field waterVelocityInst - length must be 3')
    }
    // Serialize message field [waterVelocityInst]
    bufferOffset = _arraySerializer.float32(obj.waterVelocityInst, buffer, bufferOffset, 3);
    // Serialize message field [waterVelocityInstFlag]
    bufferOffset = _serializer.int16(obj.waterVelocityInstFlag, buffer, bufferOffset);
    // Check that the constant length array field [waterVelocityEarth] has the right length
    if (obj.waterVelocityEarth.length !== 3) {
      throw new Error('Unable to serialize array field waterVelocityEarth - length must be 3')
    }
    // Serialize message field [waterVelocityEarth]
    bufferOffset = _arraySerializer.float32(obj.waterVelocityEarth, buffer, bufferOffset, 3);
    // Serialize message field [waterVelocityEarthFlag]
    bufferOffset = _serializer.int16(obj.waterVelocityEarthFlag, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float32(obj.roll, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float32(obj.pitch, buffer, bufferOffset);
    // Serialize message field [heading]
    bufferOffset = _serializer.float32(obj.heading, buffer, bufferOffset);
    // Serialize message field [altitude]
    bufferOffset = _serializer.float32(obj.altitude, buffer, bufferOffset);
    // Serialize message field [temperature]
    bufferOffset = _serializer.float32(obj.temperature, buffer, bufferOffset);
    // Serialize message field [pressure]
    bufferOffset = _serializer.float32(obj.pressure, buffer, bufferOffset);
    // Serialize message field [salinity]
    bufferOffset = _serializer.float32(obj.salinity, buffer, bufferOffset);
    // Serialize message field [soundSpeed]
    bufferOffset = _serializer.float32(obj.soundSpeed, buffer, bufferOffset);
    // Serialize message field [rawData]
    bufferOffset = _serializer.string(obj.rawData, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type dvlmsg
    let len;
    let data = new dvlmsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [errCode]
    data.errCode = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [dataGood]
    data.dataGood = _arrayDeserializer.int16(buffer, bufferOffset, 4)
    // Deserialize message field [altitudeBeam]
    data.altitudeBeam = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [bottomVelocityBeam]
    data.bottomVelocityBeam = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [waterVelocityBeam]
    data.waterVelocityBeam = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [waterVelocityCredit]
    data.waterVelocityCredit = _arrayDeserializer.float32(buffer, bufferOffset, 4)
    // Deserialize message field [velocityInst]
    data.velocityInst = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [velocityInstFlag]
    data.velocityInstFlag = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [velocityEarth]
    data.velocityEarth = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [velocityEarthFlag]
    data.velocityEarthFlag = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [waterVelocityInst]
    data.waterVelocityInst = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [waterVelocityInstFlag]
    data.waterVelocityInstFlag = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [waterVelocityEarth]
    data.waterVelocityEarth = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [waterVelocityEarthFlag]
    data.waterVelocityEarthFlag = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [heading]
    data.heading = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [altitude]
    data.altitude = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [temperature]
    data.temperature = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pressure]
    data.pressure = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [salinity]
    data.salinity = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [soundSpeed]
    data.soundSpeed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [rawData]
    data.rawData = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.errCode.length;
    length += object.rawData.length;
    return length + 168;
  }

  static datatype() {
    // Returns string type for a message object
    return 'glider_msg/dvlmsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c18145d5becba32686a61649fd23cb97';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    string errCode
    int16[4] dataGood
    float32[4] altitudeBeam
    float32[4] bottomVelocityBeam
    float32[4] waterVelocityBeam
    float32[4] waterVelocityCredit
    float32[3] velocityInst
    int16 velocityInstFlag
    float32[3] velocityEarth
    int16 velocityEarthFlag
    float32[3] waterVelocityInst
    int16 waterVelocityInstFlag
    float32[3] waterVelocityEarth
    int16 waterVelocityEarthFlag
    float32 roll
    float32 pitch
    float32 heading
    float32 altitude
    float32 temperature
    float32 pressure
    float32 salinity
    float32 soundSpeed
    string rawData
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
    const resolved = new dvlmsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.errCode !== undefined) {
      resolved.errCode = msg.errCode;
    }
    else {
      resolved.errCode = ''
    }

    if (msg.dataGood !== undefined) {
      resolved.dataGood = msg.dataGood;
    }
    else {
      resolved.dataGood = new Array(4).fill(0)
    }

    if (msg.altitudeBeam !== undefined) {
      resolved.altitudeBeam = msg.altitudeBeam;
    }
    else {
      resolved.altitudeBeam = new Array(4).fill(0)
    }

    if (msg.bottomVelocityBeam !== undefined) {
      resolved.bottomVelocityBeam = msg.bottomVelocityBeam;
    }
    else {
      resolved.bottomVelocityBeam = new Array(4).fill(0)
    }

    if (msg.waterVelocityBeam !== undefined) {
      resolved.waterVelocityBeam = msg.waterVelocityBeam;
    }
    else {
      resolved.waterVelocityBeam = new Array(4).fill(0)
    }

    if (msg.waterVelocityCredit !== undefined) {
      resolved.waterVelocityCredit = msg.waterVelocityCredit;
    }
    else {
      resolved.waterVelocityCredit = new Array(4).fill(0)
    }

    if (msg.velocityInst !== undefined) {
      resolved.velocityInst = msg.velocityInst;
    }
    else {
      resolved.velocityInst = new Array(3).fill(0)
    }

    if (msg.velocityInstFlag !== undefined) {
      resolved.velocityInstFlag = msg.velocityInstFlag;
    }
    else {
      resolved.velocityInstFlag = 0
    }

    if (msg.velocityEarth !== undefined) {
      resolved.velocityEarth = msg.velocityEarth;
    }
    else {
      resolved.velocityEarth = new Array(3).fill(0)
    }

    if (msg.velocityEarthFlag !== undefined) {
      resolved.velocityEarthFlag = msg.velocityEarthFlag;
    }
    else {
      resolved.velocityEarthFlag = 0
    }

    if (msg.waterVelocityInst !== undefined) {
      resolved.waterVelocityInst = msg.waterVelocityInst;
    }
    else {
      resolved.waterVelocityInst = new Array(3).fill(0)
    }

    if (msg.waterVelocityInstFlag !== undefined) {
      resolved.waterVelocityInstFlag = msg.waterVelocityInstFlag;
    }
    else {
      resolved.waterVelocityInstFlag = 0
    }

    if (msg.waterVelocityEarth !== undefined) {
      resolved.waterVelocityEarth = msg.waterVelocityEarth;
    }
    else {
      resolved.waterVelocityEarth = new Array(3).fill(0)
    }

    if (msg.waterVelocityEarthFlag !== undefined) {
      resolved.waterVelocityEarthFlag = msg.waterVelocityEarthFlag;
    }
    else {
      resolved.waterVelocityEarthFlag = 0
    }

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0.0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.heading !== undefined) {
      resolved.heading = msg.heading;
    }
    else {
      resolved.heading = 0.0
    }

    if (msg.altitude !== undefined) {
      resolved.altitude = msg.altitude;
    }
    else {
      resolved.altitude = 0.0
    }

    if (msg.temperature !== undefined) {
      resolved.temperature = msg.temperature;
    }
    else {
      resolved.temperature = 0.0
    }

    if (msg.pressure !== undefined) {
      resolved.pressure = msg.pressure;
    }
    else {
      resolved.pressure = 0.0
    }

    if (msg.salinity !== undefined) {
      resolved.salinity = msg.salinity;
    }
    else {
      resolved.salinity = 0.0
    }

    if (msg.soundSpeed !== undefined) {
      resolved.soundSpeed = msg.soundSpeed;
    }
    else {
      resolved.soundSpeed = 0.0
    }

    if (msg.rawData !== undefined) {
      resolved.rawData = msg.rawData;
    }
    else {
      resolved.rawData = ''
    }

    return resolved;
    }
};

module.exports = dvlmsg;
