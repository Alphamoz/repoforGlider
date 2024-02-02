;; Auto-generated. Do not edit!


(when (boundp 'glider_msg::imumsg)
  (if (not (find-package "GLIDER_MSG"))
    (make-package "GLIDER_MSG"))
  (shadow 'imumsg (find-package "GLIDER_MSG")))
(unless (find-package "GLIDER_MSG::IMUMSG")
  (make-package "GLIDER_MSG::IMUMSG"))

(in-package "ROS")
;;//! \htmlinclude imumsg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass glider_msg::imumsg
  :super ros::object
  :slots (_header _gpsStatus _latitude _longitude _altitude _linearX _linearY _linearZ _angularX _angularY _angularZ _orientationX _orientationY _orientationZ _orientationW _angularVeloX _angularVeloY _angularVeloZ _linearAccelerationX _linearAccelerationY _linearAccelerationZ _position_covariance _orientationCovariance _linearAccelerationCovariance _RPY _systemStatusMessage _statusLevel ))

(defmethod glider_msg::imumsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:gpsStatus __gpsStatus) 0)
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:altitude __altitude) 0.0)
    ((:linearX __linearX) 0.0)
    ((:linearY __linearY) 0.0)
    ((:linearZ __linearZ) 0.0)
    ((:angularX __angularX) 0.0)
    ((:angularY __angularY) 0.0)
    ((:angularZ __angularZ) 0.0)
    ((:orientationX __orientationX) 0.0)
    ((:orientationY __orientationY) 0.0)
    ((:orientationZ __orientationZ) 0.0)
    ((:orientationW __orientationW) 0.0)
    ((:angularVeloX __angularVeloX) 0.0)
    ((:angularVeloY __angularVeloY) 0.0)
    ((:angularVeloZ __angularVeloZ) 0.0)
    ((:linearAccelerationX __linearAccelerationX) 0.0)
    ((:linearAccelerationY __linearAccelerationY) 0.0)
    ((:linearAccelerationZ __linearAccelerationZ) 0.0)
    ((:position_covariance __position_covariance) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:orientationCovariance __orientationCovariance) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:linearAccelerationCovariance __linearAccelerationCovariance) (make-array 9 :initial-element 0.0 :element-type :float))
    ((:RPY __RPY) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:systemStatusMessage __systemStatusMessage) "")
    ((:statusLevel __statusLevel) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _gpsStatus (round __gpsStatus))
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _altitude (float __altitude))
   (setq _linearX (float __linearX))
   (setq _linearY (float __linearY))
   (setq _linearZ (float __linearZ))
   (setq _angularX (float __angularX))
   (setq _angularY (float __angularY))
   (setq _angularZ (float __angularZ))
   (setq _orientationX (float __orientationX))
   (setq _orientationY (float __orientationY))
   (setq _orientationZ (float __orientationZ))
   (setq _orientationW (float __orientationW))
   (setq _angularVeloX (float __angularVeloX))
   (setq _angularVeloY (float __angularVeloY))
   (setq _angularVeloZ (float __angularVeloZ))
   (setq _linearAccelerationX (float __linearAccelerationX))
   (setq _linearAccelerationY (float __linearAccelerationY))
   (setq _linearAccelerationZ (float __linearAccelerationZ))
   (setq _position_covariance __position_covariance)
   (setq _orientationCovariance __orientationCovariance)
   (setq _linearAccelerationCovariance __linearAccelerationCovariance)
   (setq _RPY __RPY)
   (setq _systemStatusMessage (string __systemStatusMessage))
   (setq _statusLevel (round __statusLevel))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:gpsStatus
   (&optional __gpsStatus)
   (if __gpsStatus (setq _gpsStatus __gpsStatus)) _gpsStatus)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:linearX
   (&optional __linearX)
   (if __linearX (setq _linearX __linearX)) _linearX)
  (:linearY
   (&optional __linearY)
   (if __linearY (setq _linearY __linearY)) _linearY)
  (:linearZ
   (&optional __linearZ)
   (if __linearZ (setq _linearZ __linearZ)) _linearZ)
  (:angularX
   (&optional __angularX)
   (if __angularX (setq _angularX __angularX)) _angularX)
  (:angularY
   (&optional __angularY)
   (if __angularY (setq _angularY __angularY)) _angularY)
  (:angularZ
   (&optional __angularZ)
   (if __angularZ (setq _angularZ __angularZ)) _angularZ)
  (:orientationX
   (&optional __orientationX)
   (if __orientationX (setq _orientationX __orientationX)) _orientationX)
  (:orientationY
   (&optional __orientationY)
   (if __orientationY (setq _orientationY __orientationY)) _orientationY)
  (:orientationZ
   (&optional __orientationZ)
   (if __orientationZ (setq _orientationZ __orientationZ)) _orientationZ)
  (:orientationW
   (&optional __orientationW)
   (if __orientationW (setq _orientationW __orientationW)) _orientationW)
  (:angularVeloX
   (&optional __angularVeloX)
   (if __angularVeloX (setq _angularVeloX __angularVeloX)) _angularVeloX)
  (:angularVeloY
   (&optional __angularVeloY)
   (if __angularVeloY (setq _angularVeloY __angularVeloY)) _angularVeloY)
  (:angularVeloZ
   (&optional __angularVeloZ)
   (if __angularVeloZ (setq _angularVeloZ __angularVeloZ)) _angularVeloZ)
  (:linearAccelerationX
   (&optional __linearAccelerationX)
   (if __linearAccelerationX (setq _linearAccelerationX __linearAccelerationX)) _linearAccelerationX)
  (:linearAccelerationY
   (&optional __linearAccelerationY)
   (if __linearAccelerationY (setq _linearAccelerationY __linearAccelerationY)) _linearAccelerationY)
  (:linearAccelerationZ
   (&optional __linearAccelerationZ)
   (if __linearAccelerationZ (setq _linearAccelerationZ __linearAccelerationZ)) _linearAccelerationZ)
  (:position_covariance
   (&optional __position_covariance)
   (if __position_covariance (setq _position_covariance __position_covariance)) _position_covariance)
  (:orientationCovariance
   (&optional __orientationCovariance)
   (if __orientationCovariance (setq _orientationCovariance __orientationCovariance)) _orientationCovariance)
  (:linearAccelerationCovariance
   (&optional __linearAccelerationCovariance)
   (if __linearAccelerationCovariance (setq _linearAccelerationCovariance __linearAccelerationCovariance)) _linearAccelerationCovariance)
  (:RPY
   (&optional __RPY)
   (if __RPY (setq _RPY __RPY)) _RPY)
  (:systemStatusMessage
   (&optional __systemStatusMessage)
   (if __systemStatusMessage (setq _systemStatusMessage __systemStatusMessage)) _systemStatusMessage)
  (:statusLevel
   (&optional __statusLevel)
   (if __statusLevel (setq _statusLevel __statusLevel)) _statusLevel)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int16 _gpsStatus
    2
    ;; float32 _latitude
    4
    ;; float32 _longitude
    4
    ;; float32 _altitude
    4
    ;; float32 _linearX
    4
    ;; float32 _linearY
    4
    ;; float32 _linearZ
    4
    ;; float32 _angularX
    4
    ;; float32 _angularY
    4
    ;; float32 _angularZ
    4
    ;; float32 _orientationX
    4
    ;; float32 _orientationY
    4
    ;; float32 _orientationZ
    4
    ;; float32 _orientationW
    4
    ;; float32 _angularVeloX
    4
    ;; float32 _angularVeloY
    4
    ;; float32 _angularVeloZ
    4
    ;; float32 _linearAccelerationX
    4
    ;; float32 _linearAccelerationY
    4
    ;; float32 _linearAccelerationZ
    4
    ;; float64[9] _position_covariance
    (* 8    9)
    ;; float64[9] _orientationCovariance
    (* 8    9)
    ;; float64[9] _linearAccelerationCovariance
    (* 8    9)
    ;; float32[3] _RPY
    (* 4    3)
    ;; string _systemStatusMessage
    4 (length _systemStatusMessage)
    ;; int16 _statusLevel
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int16 _gpsStatus
       (write-word _gpsStatus s)
     ;; float32 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _linearX
       (sys::poke _linearX (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _linearY
       (sys::poke _linearY (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _linearZ
       (sys::poke _linearZ (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angularX
       (sys::poke _angularX (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angularY
       (sys::poke _angularY (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angularZ
       (sys::poke _angularZ (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _orientationX
       (sys::poke _orientationX (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _orientationY
       (sys::poke _orientationY (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _orientationZ
       (sys::poke _orientationZ (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _orientationW
       (sys::poke _orientationW (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angularVeloX
       (sys::poke _angularVeloX (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angularVeloY
       (sys::poke _angularVeloY (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angularVeloZ
       (sys::poke _angularVeloZ (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _linearAccelerationX
       (sys::poke _linearAccelerationX (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _linearAccelerationY
       (sys::poke _linearAccelerationY (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _linearAccelerationZ
       (sys::poke _linearAccelerationZ (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64[9] _position_covariance
     (dotimes (i 9)
       (sys::poke (elt _position_covariance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[9] _orientationCovariance
     (dotimes (i 9)
       (sys::poke (elt _orientationCovariance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[9] _linearAccelerationCovariance
     (dotimes (i 9)
       (sys::poke (elt _linearAccelerationCovariance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float32[3] _RPY
     (dotimes (i 3)
       (sys::poke (elt _RPY i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; string _systemStatusMessage
       (write-long (length _systemStatusMessage) s) (princ _systemStatusMessage s)
     ;; int16 _statusLevel
       (write-word _statusLevel s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int16 _gpsStatus
     (setq _gpsStatus (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _latitude
     (setq _latitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _longitude
     (setq _longitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _altitude
     (setq _altitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _linearX
     (setq _linearX (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _linearY
     (setq _linearY (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _linearZ
     (setq _linearZ (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angularX
     (setq _angularX (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angularY
     (setq _angularY (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angularZ
     (setq _angularZ (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _orientationX
     (setq _orientationX (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _orientationY
     (setq _orientationY (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _orientationZ
     (setq _orientationZ (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _orientationW
     (setq _orientationW (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angularVeloX
     (setq _angularVeloX (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angularVeloY
     (setq _angularVeloY (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angularVeloZ
     (setq _angularVeloZ (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _linearAccelerationX
     (setq _linearAccelerationX (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _linearAccelerationY
     (setq _linearAccelerationY (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _linearAccelerationZ
     (setq _linearAccelerationZ (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64[9] _position_covariance
   (dotimes (i (length _position_covariance))
     (setf (elt _position_covariance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[9] _orientationCovariance
   (dotimes (i (length _orientationCovariance))
     (setf (elt _orientationCovariance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[9] _linearAccelerationCovariance
   (dotimes (i (length _linearAccelerationCovariance))
     (setf (elt _linearAccelerationCovariance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float32[3] _RPY
   (dotimes (i (length _RPY))
     (setf (elt _RPY i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; string _systemStatusMessage
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _systemStatusMessage (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int16 _statusLevel
     (setq _statusLevel (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get glider_msg::imumsg :md5sum-) "d2f0d176490184beac4b63cba6827925")
(setf (get glider_msg::imumsg :datatype-) "glider_msg/imumsg")
(setf (get glider_msg::imumsg :definition-)
      "Header header
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

")



(provide :glider_msg/imumsg "d2f0d176490184beac4b63cba6827925")


