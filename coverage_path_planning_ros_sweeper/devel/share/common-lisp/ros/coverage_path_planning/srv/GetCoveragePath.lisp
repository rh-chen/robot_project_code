; Auto-generated. Do not edit!


(cl:in-package coverage_path_planning-srv)


;//! \htmlinclude GetCoveragePath-request.msg.html

(cl:defclass <GetCoveragePath-request> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (goal
    :reader goal
    :initarg :goal
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (erosion_radius
    :reader erosion_radius
    :initarg :erosion_radius
    :type cl:float
    :initform 0.0)
   (robot_radius
    :reader robot_radius
    :initarg :robot_radius
    :type cl:float
    :initform 0.0)
   (occupancy_threshold
    :reader occupancy_threshold
    :initarg :occupancy_threshold
    :type cl:fixnum
    :initform 0)
   (map
    :reader map
    :initarg :map
    :type nav_msgs-msg:OccupancyGrid
    :initform (cl:make-instance 'nav_msgs-msg:OccupancyGrid)))
)

(cl:defclass GetCoveragePath-request (<GetCoveragePath-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCoveragePath-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCoveragePath-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name coverage_path_planning-srv:<GetCoveragePath-request> is deprecated: use coverage_path_planning-srv:GetCoveragePath-request instead.")))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <GetCoveragePath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coverage_path_planning-srv:start-val is deprecated.  Use coverage_path_planning-srv:start instead.")
  (start m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <GetCoveragePath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coverage_path_planning-srv:goal-val is deprecated.  Use coverage_path_planning-srv:goal instead.")
  (goal m))

(cl:ensure-generic-function 'erosion_radius-val :lambda-list '(m))
(cl:defmethod erosion_radius-val ((m <GetCoveragePath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coverage_path_planning-srv:erosion_radius-val is deprecated.  Use coverage_path_planning-srv:erosion_radius instead.")
  (erosion_radius m))

(cl:ensure-generic-function 'robot_radius-val :lambda-list '(m))
(cl:defmethod robot_radius-val ((m <GetCoveragePath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coverage_path_planning-srv:robot_radius-val is deprecated.  Use coverage_path_planning-srv:robot_radius instead.")
  (robot_radius m))

(cl:ensure-generic-function 'occupancy_threshold-val :lambda-list '(m))
(cl:defmethod occupancy_threshold-val ((m <GetCoveragePath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coverage_path_planning-srv:occupancy_threshold-val is deprecated.  Use coverage_path_planning-srv:occupancy_threshold instead.")
  (occupancy_threshold m))

(cl:ensure-generic-function 'map-val :lambda-list '(m))
(cl:defmethod map-val ((m <GetCoveragePath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coverage_path_planning-srv:map-val is deprecated.  Use coverage_path_planning-srv:map instead.")
  (map m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCoveragePath-request>) ostream)
  "Serializes a message object of type '<GetCoveragePath-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'erosion_radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'robot_radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'occupancy_threshold)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'map) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCoveragePath-request>) istream)
  "Deserializes a message object of type '<GetCoveragePath-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'erosion_radius) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'robot_radius) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'occupancy_threshold) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'map) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCoveragePath-request>)))
  "Returns string type for a service object of type '<GetCoveragePath-request>"
  "coverage_path_planning/GetCoveragePathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCoveragePath-request)))
  "Returns string type for a service object of type 'GetCoveragePath-request"
  "coverage_path_planning/GetCoveragePathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCoveragePath-request>)))
  "Returns md5sum for a message object of type '<GetCoveragePath-request>"
  "942df67a0deefe8bfb22ef78e1504921")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCoveragePath-request)))
  "Returns md5sum for a message object of type 'GetCoveragePath-request"
  "942df67a0deefe8bfb22ef78e1504921")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCoveragePath-request>)))
  "Returns full string definition for message of type '<GetCoveragePath-request>"
  (cl:format cl:nil "~%~%~%geometry_msgs/PoseStamped start~%~%~%geometry_msgs/PoseStamped goal~%~%~%~%float64 erosion_radius~%~%~%~%float64 robot_radius~%~%~%~%int8 occupancy_threshold~%~%nav_msgs/OccupancyGrid map~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: nav_msgs/OccupancyGrid~%# This represents a 2-D grid map, in which each cell represents the probability of~%# occupancy.~%~%Header header ~%~%#MetaData for the map~%MapMetaData info~%~%# The map data, in row-major order, starting with (0,0).  Occupancy~%# probabilities are in the range [0,100].  Unknown is -1.~%int8[] data~%~%================================================================================~%MSG: nav_msgs/MapMetaData~%# This hold basic information about the characterists of the OccupancyGrid~%~%# The time at which the map was loaded~%time map_load_time~%# The map resolution [m/cell]~%float32 resolution~%# Map width [cells]~%uint32 width~%# Map height [cells]~%uint32 height~%# The origin of the map [m, m, rad].  This is the real-world pose of the~%# cell (0,0) in the map.~%geometry_msgs/Pose origin~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCoveragePath-request)))
  "Returns full string definition for message of type 'GetCoveragePath-request"
  (cl:format cl:nil "~%~%~%geometry_msgs/PoseStamped start~%~%~%geometry_msgs/PoseStamped goal~%~%~%~%float64 erosion_radius~%~%~%~%float64 robot_radius~%~%~%~%int8 occupancy_threshold~%~%nav_msgs/OccupancyGrid map~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: nav_msgs/OccupancyGrid~%# This represents a 2-D grid map, in which each cell represents the probability of~%# occupancy.~%~%Header header ~%~%#MetaData for the map~%MapMetaData info~%~%# The map data, in row-major order, starting with (0,0).  Occupancy~%# probabilities are in the range [0,100].  Unknown is -1.~%int8[] data~%~%================================================================================~%MSG: nav_msgs/MapMetaData~%# This hold basic information about the characterists of the OccupancyGrid~%~%# The time at which the map was loaded~%time map_load_time~%# The map resolution [m/cell]~%float32 resolution~%# Map width [cells]~%uint32 width~%# Map height [cells]~%uint32 height~%# The origin of the map [m, m, rad].  This is the real-world pose of the~%# cell (0,0) in the map.~%geometry_msgs/Pose origin~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCoveragePath-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
     8
     8
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'map))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCoveragePath-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCoveragePath-request
    (cl:cons ':start (start msg))
    (cl:cons ':goal (goal msg))
    (cl:cons ':erosion_radius (erosion_radius msg))
    (cl:cons ':robot_radius (robot_radius msg))
    (cl:cons ':occupancy_threshold (occupancy_threshold msg))
    (cl:cons ':map (map msg))
))
;//! \htmlinclude GetCoveragePath-response.msg.html

(cl:defclass <GetCoveragePath-response> (roslisp-msg-protocol:ros-message)
  ((plan
    :reader plan
    :initarg :plan
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path)))
)

(cl:defclass GetCoveragePath-response (<GetCoveragePath-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetCoveragePath-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetCoveragePath-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name coverage_path_planning-srv:<GetCoveragePath-response> is deprecated: use coverage_path_planning-srv:GetCoveragePath-response instead.")))

(cl:ensure-generic-function 'plan-val :lambda-list '(m))
(cl:defmethod plan-val ((m <GetCoveragePath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader coverage_path_planning-srv:plan-val is deprecated.  Use coverage_path_planning-srv:plan instead.")
  (plan m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetCoveragePath-response>) ostream)
  "Serializes a message object of type '<GetCoveragePath-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'plan) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetCoveragePath-response>) istream)
  "Deserializes a message object of type '<GetCoveragePath-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'plan) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetCoveragePath-response>)))
  "Returns string type for a service object of type '<GetCoveragePath-response>"
  "coverage_path_planning/GetCoveragePathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCoveragePath-response)))
  "Returns string type for a service object of type 'GetCoveragePath-response"
  "coverage_path_planning/GetCoveragePathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetCoveragePath-response>)))
  "Returns md5sum for a message object of type '<GetCoveragePath-response>"
  "942df67a0deefe8bfb22ef78e1504921")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetCoveragePath-response)))
  "Returns md5sum for a message object of type 'GetCoveragePath-response"
  "942df67a0deefe8bfb22ef78e1504921")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetCoveragePath-response>)))
  "Returns full string definition for message of type '<GetCoveragePath-response>"
  (cl:format cl:nil "nav_msgs/Path plan~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetCoveragePath-response)))
  "Returns full string definition for message of type 'GetCoveragePath-response"
  (cl:format cl:nil "nav_msgs/Path plan~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetCoveragePath-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'plan))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetCoveragePath-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetCoveragePath-response
    (cl:cons ':plan (plan msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetCoveragePath)))
  'GetCoveragePath-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetCoveragePath)))
  'GetCoveragePath-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetCoveragePath)))
  "Returns string type for a service object of type '<GetCoveragePath>"
  "coverage_path_planning/GetCoveragePath")