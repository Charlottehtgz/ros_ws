// Auto-generated. Do not edit!

// (in-package ros_vosk.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class speech_recognition {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.time_recognized = null;
      this.final_result = null;
      this.partial_result = null;
      this.isSpeech_recognized = null;
    }
    else {
      if (initObj.hasOwnProperty('time_recognized')) {
        this.time_recognized = initObj.time_recognized
      }
      else {
        this.time_recognized = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('final_result')) {
        this.final_result = initObj.final_result
      }
      else {
        this.final_result = '';
      }
      if (initObj.hasOwnProperty('partial_result')) {
        this.partial_result = initObj.partial_result
      }
      else {
        this.partial_result = '';
      }
      if (initObj.hasOwnProperty('isSpeech_recognized')) {
        this.isSpeech_recognized = initObj.isSpeech_recognized
      }
      else {
        this.isSpeech_recognized = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type speech_recognition
    // Serialize message field [time_recognized]
    bufferOffset = _serializer.time(obj.time_recognized, buffer, bufferOffset);
    // Serialize message field [final_result]
    bufferOffset = _serializer.string(obj.final_result, buffer, bufferOffset);
    // Serialize message field [partial_result]
    bufferOffset = _serializer.string(obj.partial_result, buffer, bufferOffset);
    // Serialize message field [isSpeech_recognized]
    bufferOffset = _serializer.bool(obj.isSpeech_recognized, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type speech_recognition
    let len;
    let data = new speech_recognition(null);
    // Deserialize message field [time_recognized]
    data.time_recognized = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [final_result]
    data.final_result = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [partial_result]
    data.partial_result = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [isSpeech_recognized]
    data.isSpeech_recognized = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.final_result);
    length += _getByteLength(object.partial_result);
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ros_vosk/speech_recognition';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd3b0b667a7ab9370dea4b81b6e8de6c9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time    time_recognized
    string  final_result
    string  partial_result 
    bool    isSpeech_recognized
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new speech_recognition(null);
    if (msg.time_recognized !== undefined) {
      resolved.time_recognized = msg.time_recognized;
    }
    else {
      resolved.time_recognized = {secs: 0, nsecs: 0}
    }

    if (msg.final_result !== undefined) {
      resolved.final_result = msg.final_result;
    }
    else {
      resolved.final_result = ''
    }

    if (msg.partial_result !== undefined) {
      resolved.partial_result = msg.partial_result;
    }
    else {
      resolved.partial_result = ''
    }

    if (msg.isSpeech_recognized !== undefined) {
      resolved.isSpeech_recognized = msg.isSpeech_recognized;
    }
    else {
      resolved.isSpeech_recognized = false
    }

    return resolved;
    }
};

module.exports = speech_recognition;
