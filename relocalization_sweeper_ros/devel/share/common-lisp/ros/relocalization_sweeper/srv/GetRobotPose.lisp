; Auto-generated. Do not edit!


(cl:in-package relocalization_sweeper-srv)


;//! \htmlinclude GetRobotPose-request.msg.html

(cl:defclass <GetRobotPose-request> (roslisp-msg-protocol:ros-message)
  ((img_data
    :reader img_data
    :initarg :img_data
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass GetRobotPose-request (<GetRobotPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRobotPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRobotPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name relocalization_sweeper-srv:<GetRobotPose-request> is deprecated: use relocalization_sweeper-srv:GetRobotPose-request instead.")))

(cl:ensure-generic-function 'img_data-val :lambda-list '(m))
(cl:defmethod img_data-val ((m <GetRobotPose-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader relocalization_sweeper-srv:img_data-val is deprecated.  Use relocalization_sweeper-srv:img_data instead.")
  (img_data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRobotPose-request>) ostream)
  "Serializes a message object of type '<GetRobotPose-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'img_data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRobotPose-request>) istream)
  "Deserializes a message object of type '<GetRobotPose-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'img_data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRobotPose-request>)))
  "Returns string type for a service object of type '<GetRobotPose-request>"
  "relocalization_sweeper/GetRobotPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotPose-request)))
  "Returns string type for a service object of type 'GetRobotPose-request"
  "relocalization_sweeper/GetRobotPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRobotPose-request>)))
  "Returns md5sum for a message object of type '<GetRobotPose-request>"
  "e2d88b1ab1ddd4e7df90d778b40332bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRobotPose-request)))
  "Returns md5sum for a message object of type 'GetRobotPose-request"
  "e2d88b1ab1ddd4e7df90d778b40332bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRobotPose-request>)))
  "Returns full string definition for message of type '<GetRobotPose-request>"
  (cl:format cl:nil "~%sensor_msgs/Image img_data~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRobotPose-request)))
  "Returns full string definition for message of type 'GetRobotPose-request"
  (cl:format cl:nil "~%sensor_msgs/Image img_data~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRobotPose-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'img_data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRobotPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRobotPose-request
    (cl:cons ':img_data (img_data msg))
))
;//! \htmlinclude GetRobotPose-response.msg.html

(cl:defclass <GetRobotPose-response> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass GetRobotPose-response (<GetRobotPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetRobotPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetRobotPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name relocalization_sweeper-srv:<GetRobotPose-response> is deprecated: use relocalization_sweeper-srv:GetRobotPose-response instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <GetRobotPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader relocalization_sweeper-srv:pose-val is deprecated.  Use relocalization_sweeper-srv:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetRobotPose-response>) ostream)
  "Serializes a message object of type '<GetRobotPose-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetRobotPose-response>) istream)
  "Deserializes a message object of type '<GetRobotPose-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetRobotPose-response>)))
  "Returns string type for a service object of type '<GetRobotPose-response>"
  "relocalization_sweeper/GetRobotPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotPose-response)))
  "Returns string type for a service object of type 'GetRobotPose-response"
  "relocalization_sweeper/GetRobotPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetRobotPose-response>)))
  "Returns md5sum for a message object of type '<GetRobotPose-response>"
  "e2d88b1ab1ddd4e7df90d778b40332bd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetRobotPose-response)))
  "Returns md5sum for a message object of type 'GetRobotPose-response"
  "e2d88b1ab1ddd4e7df90d778b40332bd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetRobotPose-response>)))
  "Returns full string definition for message of type '<GetRobotPose-response>"
  (cl:format cl:nil "geometry_msgs/PoseStamped pose~%~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetRobotPose-response)))
  "Returns full string definition for message of type 'GetRobotPose-response"
  (cl:format cl:nil "geometry_msgs/PoseStamped pose~%~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetRobotPose-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetRobotPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetRobotPose-response
    (cl:cons ':pose (pose msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetRobotPose)))
  'GetRobotPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetRobotPose)))
  'GetRobotPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetRobotPose)))
  "Returns string type for a service object of type '<GetRobotPose>"
  "relocalization_sweeper/GetRobotPose")