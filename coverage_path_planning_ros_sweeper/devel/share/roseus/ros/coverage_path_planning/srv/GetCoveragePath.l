;; Auto-generated. Do not edit!


(when (boundp 'coverage_path_planning::GetCoveragePath)
  (if (not (find-package "COVERAGE_PATH_PLANNING"))
    (make-package "COVERAGE_PATH_PLANNING"))
  (shadow 'GetCoveragePath (find-package "COVERAGE_PATH_PLANNING")))
(unless (find-package "COVERAGE_PATH_PLANNING::GETCOVERAGEPATH")
  (make-package "COVERAGE_PATH_PLANNING::GETCOVERAGEPATH"))
(unless (find-package "COVERAGE_PATH_PLANNING::GETCOVERAGEPATHREQUEST")
  (make-package "COVERAGE_PATH_PLANNING::GETCOVERAGEPATHREQUEST"))
(unless (find-package "COVERAGE_PATH_PLANNING::GETCOVERAGEPATHRESPONSE")
  (make-package "COVERAGE_PATH_PLANNING::GETCOVERAGEPATHRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(defclass coverage_path_planning::GetCoveragePathRequest
  :super ros::object
  :slots (_start _goal _erosion_radius _robot_radius _occupancy_threshold _map ))

(defmethod coverage_path_planning::GetCoveragePathRequest
  (:init
   (&key
    ((:start __start) (instance geometry_msgs::PoseStamped :init))
    ((:goal __goal) (instance geometry_msgs::PoseStamped :init))
    ((:erosion_radius __erosion_radius) 0.0)
    ((:robot_radius __robot_radius) 0.0)
    ((:occupancy_threshold __occupancy_threshold) 0)
    ((:map __map) (instance nav_msgs::OccupancyGrid :init))
    )
   (send-super :init)
   (setq _start __start)
   (setq _goal __goal)
   (setq _erosion_radius (float __erosion_radius))
   (setq _robot_radius (float __robot_radius))
   (setq _occupancy_threshold (round __occupancy_threshold))
   (setq _map __map)
   self)
  (:start
   (&rest __start)
   (if (keywordp (car __start))
       (send* _start __start)
     (progn
       (if __start (setq _start (car __start)))
       _start)))
  (:goal
   (&rest __goal)
   (if (keywordp (car __goal))
       (send* _goal __goal)
     (progn
       (if __goal (setq _goal (car __goal)))
       _goal)))
  (:erosion_radius
   (&optional __erosion_radius)
   (if __erosion_radius (setq _erosion_radius __erosion_radius)) _erosion_radius)
  (:robot_radius
   (&optional __robot_radius)
   (if __robot_radius (setq _robot_radius __robot_radius)) _robot_radius)
  (:occupancy_threshold
   (&optional __occupancy_threshold)
   (if __occupancy_threshold (setq _occupancy_threshold __occupancy_threshold)) _occupancy_threshold)
  (:map
   (&rest __map)
   (if (keywordp (car __map))
       (send* _map __map)
     (progn
       (if __map (setq _map (car __map)))
       _map)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped _start
    (send _start :serialization-length)
    ;; geometry_msgs/PoseStamped _goal
    (send _goal :serialization-length)
    ;; float64 _erosion_radius
    8
    ;; float64 _robot_radius
    8
    ;; int8 _occupancy_threshold
    1
    ;; nav_msgs/OccupancyGrid _map
    (send _map :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped _start
       (send _start :serialize s)
     ;; geometry_msgs/PoseStamped _goal
       (send _goal :serialize s)
     ;; float64 _erosion_radius
       (sys::poke _erosion_radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _robot_radius
       (sys::poke _robot_radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int8 _occupancy_threshold
       (write-byte _occupancy_threshold s)
     ;; nav_msgs/OccupancyGrid _map
       (send _map :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped _start
     (send _start :deserialize buf ptr-) (incf ptr- (send _start :serialization-length))
   ;; geometry_msgs/PoseStamped _goal
     (send _goal :deserialize buf ptr-) (incf ptr- (send _goal :serialization-length))
   ;; float64 _erosion_radius
     (setq _erosion_radius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _robot_radius
     (setq _robot_radius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int8 _occupancy_threshold
     (setq _occupancy_threshold (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _occupancy_threshold 127) (setq _occupancy_threshold (- _occupancy_threshold 256)))
   ;; nav_msgs/OccupancyGrid _map
     (send _map :deserialize buf ptr-) (incf ptr- (send _map :serialization-length))
   ;;
   self)
  )

(defclass coverage_path_planning::GetCoveragePathResponse
  :super ros::object
  :slots (_plan ))

(defmethod coverage_path_planning::GetCoveragePathResponse
  (:init
   (&key
    ((:plan __plan) (instance nav_msgs::Path :init))
    )
   (send-super :init)
   (setq _plan __plan)
   self)
  (:plan
   (&rest __plan)
   (if (keywordp (car __plan))
       (send* _plan __plan)
     (progn
       (if __plan (setq _plan (car __plan)))
       _plan)))
  (:serialization-length
   ()
   (+
    ;; nav_msgs/Path _plan
    (send _plan :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nav_msgs/Path _plan
       (send _plan :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nav_msgs/Path _plan
     (send _plan :deserialize buf ptr-) (incf ptr- (send _plan :serialization-length))
   ;;
   self)
  )

(defclass coverage_path_planning::GetCoveragePath
  :super ros::object
  :slots ())

(setf (get coverage_path_planning::GetCoveragePath :md5sum-) "942df67a0deefe8bfb22ef78e1504921")
(setf (get coverage_path_planning::GetCoveragePath :datatype-) "coverage_path_planning/GetCoveragePath")
(setf (get coverage_path_planning::GetCoveragePath :request) coverage_path_planning::GetCoveragePathRequest)
(setf (get coverage_path_planning::GetCoveragePath :response) coverage_path_planning::GetCoveragePathResponse)

(defmethod coverage_path_planning::GetCoveragePathRequest
  (:response () (instance coverage_path_planning::GetCoveragePathResponse :init)))

(setf (get coverage_path_planning::GetCoveragePathRequest :md5sum-) "942df67a0deefe8bfb22ef78e1504921")
(setf (get coverage_path_planning::GetCoveragePathRequest :datatype-) "coverage_path_planning/GetCoveragePathRequest")
(setf (get coverage_path_planning::GetCoveragePathRequest :definition-)
      "


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
---
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
")

(setf (get coverage_path_planning::GetCoveragePathResponse :md5sum-) "942df67a0deefe8bfb22ef78e1504921")
(setf (get coverage_path_planning::GetCoveragePathResponse :datatype-) "coverage_path_planning/GetCoveragePathResponse")
(setf (get coverage_path_planning::GetCoveragePathResponse :definition-)
      "


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
---
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
")



(provide :coverage_path_planning/GetCoveragePath "942df67a0deefe8bfb22ef78e1504921")


