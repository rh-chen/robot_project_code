// Auto-generated. Do not edit!

// (in-package coverage_path_planning.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let nav_msgs = _finder('nav_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class GetCoveragePathRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start = null;
      this.goal = null;
      this.erosion_radius = null;
      this.robot_radius = null;
      this.occupancy_threshold = null;
      this.map = null;
    }
    else {
      if (initObj.hasOwnProperty('start')) {
        this.start = initObj.start
      }
      else {
        this.start = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('goal')) {
        this.goal = initObj.goal
      }
      else {
        this.goal = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('erosion_radius')) {
        this.erosion_radius = initObj.erosion_radius
      }
      else {
        this.erosion_radius = 0.0;
      }
      if (initObj.hasOwnProperty('robot_radius')) {
        this.robot_radius = initObj.robot_radius
      }
      else {
        this.robot_radius = 0.0;
      }
      if (initObj.hasOwnProperty('occupancy_threshold')) {
        this.occupancy_threshold = initObj.occupancy_threshold
      }
      else {
        this.occupancy_threshold = 0;
      }
      if (initObj.hasOwnProperty('map')) {
        this.map = initObj.map
      }
      else {
        this.map = new nav_msgs.msg.OccupancyGrid();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCoveragePathRequest
    // Serialize message field [start]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.start, buffer, bufferOffset);
    // Serialize message field [goal]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.goal, buffer, bufferOffset);
    // Serialize message field [erosion_radius]
    bufferOffset = _serializer.float64(obj.erosion_radius, buffer, bufferOffset);
    // Serialize message field [robot_radius]
    bufferOffset = _serializer.float64(obj.robot_radius, buffer, bufferOffset);
    // Serialize message field [occupancy_threshold]
    bufferOffset = _serializer.int8(obj.occupancy_threshold, buffer, bufferOffset);
    // Serialize message field [map]
    bufferOffset = nav_msgs.msg.OccupancyGrid.serialize(obj.map, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCoveragePathRequest
    let len;
    let data = new GetCoveragePathRequest(null);
    // Deserialize message field [start]
    data.start = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [goal]
    data.goal = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [erosion_radius]
    data.erosion_radius = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [robot_radius]
    data.robot_radius = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [occupancy_threshold]
    data.occupancy_threshold = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [map]
    data.map = nav_msgs.msg.OccupancyGrid.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.start);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.goal);
    length += nav_msgs.msg.OccupancyGrid.getMessageSize(object.map);
    return length + 17;
  }

  static datatype() {
    // Returns string type for a service object
    return 'coverage_path_planning/GetCoveragePathRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a4219cf242dbc3b080f93747a5b0ea84';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    
    
    geometry_msgs/PoseStamped start
    
    
    geometry_msgs/PoseStamped goal
    
    
    
    float64 erosion_radius
    
    
    
    float64 robot_radius
    
    
    
    int8 occupancy_threshold
    
    nav_msgs/OccupancyGrid map
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
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
    
    ================================================================================
    MSG: nav_msgs/OccupancyGrid
    # This represents a 2-D grid map, in which each cell represents the probability of
    # occupancy.
    
    Header header 
    
    #MetaData for the map
    MapMetaData info
    
    # The map data, in row-major order, starting with (0,0).  Occupancy
    # probabilities are in the range [0,100].  Unknown is -1.
    int8[] data
    
    ================================================================================
    MSG: nav_msgs/MapMetaData
    # This hold basic information about the characterists of the OccupancyGrid
    
    # The time at which the map was loaded
    time map_load_time
    # The map resolution [m/cell]
    float32 resolution
    # Map width [cells]
    uint32 width
    # Map height [cells]
    uint32 height
    # The origin of the map [m, m, rad].  This is the real-world pose of the
    # cell (0,0) in the map.
    geometry_msgs/Pose origin
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCoveragePathRequest(null);
    if (msg.start !== undefined) {
      resolved.start = geometry_msgs.msg.PoseStamped.Resolve(msg.start)
    }
    else {
      resolved.start = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.goal !== undefined) {
      resolved.goal = geometry_msgs.msg.PoseStamped.Resolve(msg.goal)
    }
    else {
      resolved.goal = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.erosion_radius !== undefined) {
      resolved.erosion_radius = msg.erosion_radius;
    }
    else {
      resolved.erosion_radius = 0.0
    }

    if (msg.robot_radius !== undefined) {
      resolved.robot_radius = msg.robot_radius;
    }
    else {
      resolved.robot_radius = 0.0
    }

    if (msg.occupancy_threshold !== undefined) {
      resolved.occupancy_threshold = msg.occupancy_threshold;
    }
    else {
      resolved.occupancy_threshold = 0
    }

    if (msg.map !== undefined) {
      resolved.map = nav_msgs.msg.OccupancyGrid.Resolve(msg.map)
    }
    else {
      resolved.map = new nav_msgs.msg.OccupancyGrid()
    }

    return resolved;
    }
};

class GetCoveragePathResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.plan = null;
    }
    else {
      if (initObj.hasOwnProperty('plan')) {
        this.plan = initObj.plan
      }
      else {
        this.plan = new nav_msgs.msg.Path();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCoveragePathResponse
    // Serialize message field [plan]
    bufferOffset = nav_msgs.msg.Path.serialize(obj.plan, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCoveragePathResponse
    let len;
    let data = new GetCoveragePathResponse(null);
    // Deserialize message field [plan]
    data.plan = nav_msgs.msg.Path.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += nav_msgs.msg.Path.getMessageSize(object.plan);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'coverage_path_planning/GetCoveragePathResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0002bc113c0259d71f6cf8cbc9430e18';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    nav_msgs/Path plan
    
    
    ================================================================================
    MSG: nav_msgs/Path
    #An array of poses that represents a Path for a robot to follow
    Header header
    geometry_msgs/PoseStamped[] poses
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
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
    const resolved = new GetCoveragePathResponse(null);
    if (msg.plan !== undefined) {
      resolved.plan = nav_msgs.msg.Path.Resolve(msg.plan)
    }
    else {
      resolved.plan = new nav_msgs.msg.Path()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetCoveragePathRequest,
  Response: GetCoveragePathResponse,
  md5sum() { return '942df67a0deefe8bfb22ef78e1504921'; },
  datatype() { return 'coverage_path_planning/GetCoveragePath'; }
};
