;; Auto-generated. Do not edit!


(when (boundp 'relocalization_sweeper::GetRobotPose)
  (if (not (find-package "RELOCALIZATION_SWEEPER"))
    (make-package "RELOCALIZATION_SWEEPER"))
  (shadow 'GetRobotPose (find-package "RELOCALIZATION_SWEEPER")))
(unless (find-package "RELOCALIZATION_SWEEPER::GETROBOTPOSE")
  (make-package "RELOCALIZATION_SWEEPER::GETROBOTPOSE"))
(unless (find-package "RELOCALIZATION_SWEEPER::GETROBOTPOSEREQUEST")
  (make-package "RELOCALIZATION_SWEEPER::GETROBOTPOSEREQUEST"))
(unless (find-package "RELOCALIZATION_SWEEPER::GETROBOTPOSERESPONSE")
  (make-package "RELOCALIZATION_SWEEPER::GETROBOTPOSERESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass relocalization_sweeper::GetRobotPoseRequest
  :super ros::object
  :slots (_img_data ))

(defmethod relocalization_sweeper::GetRobotPoseRequest
  (:init
   (&key
    ((:img_data __img_data) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _img_data __img_data)
   self)
  (:img_data
   (&rest __img_data)
   (if (keywordp (car __img_data))
       (send* _img_data __img_data)
     (progn
       (if __img_data (setq _img_data (car __img_data)))
       _img_data)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Image _img_data
    (send _img_data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Image _img_data
       (send _img_data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Image _img_data
     (send _img_data :deserialize buf ptr-) (incf ptr- (send _img_data :serialization-length))
   ;;
   self)
  )

(defclass relocalization_sweeper::GetRobotPoseResponse
  :super ros::object
  :slots (_pose ))

(defmethod relocalization_sweeper::GetRobotPoseResponse
  (:init
   (&key
    ((:pose __pose) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _pose __pose)
   self)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(defclass relocalization_sweeper::GetRobotPose
  :super ros::object
  :slots ())

(setf (get relocalization_sweeper::GetRobotPose :md5sum-) "e2d88b1ab1ddd4e7df90d778b40332bd")
(setf (get relocalization_sweeper::GetRobotPose :datatype-) "relocalization_sweeper/GetRobotPose")
(setf (get relocalization_sweeper::GetRobotPose :request) relocalization_sweeper::GetRobotPoseRequest)
(setf (get relocalization_sweeper::GetRobotPose :response) relocalization_sweeper::GetRobotPoseResponse)

(defmethod relocalization_sweeper::GetRobotPoseRequest
  (:response () (instance relocalization_sweeper::GetRobotPoseResponse :init)))

(setf (get relocalization_sweeper::GetRobotPoseRequest :md5sum-) "e2d88b1ab1ddd4e7df90d778b40332bd")
(setf (get relocalization_sweeper::GetRobotPoseRequest :datatype-) "relocalization_sweeper/GetRobotPoseRequest")
(setf (get relocalization_sweeper::GetRobotPoseRequest :definition-)
      "
sensor_msgs/Image img_data

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of cameara
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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
---
geometry_msgs/PoseStamped pose



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
")

(setf (get relocalization_sweeper::GetRobotPoseResponse :md5sum-) "e2d88b1ab1ddd4e7df90d778b40332bd")
(setf (get relocalization_sweeper::GetRobotPoseResponse :datatype-) "relocalization_sweeper/GetRobotPoseResponse")
(setf (get relocalization_sweeper::GetRobotPoseResponse :definition-)
      "
sensor_msgs/Image img_data

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of cameara
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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
---
geometry_msgs/PoseStamped pose



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
")



(provide :relocalization_sweeper/GetRobotPose "e2d88b1ab1ddd4e7df90d778b40332bd")


