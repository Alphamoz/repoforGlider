;; Auto-generated. Do not edit!


(when (boundp 'glider_msg::dvlmsg)
  (if (not (find-package "GLIDER_MSG"))
    (make-package "GLIDER_MSG"))
  (shadow 'dvlmsg (find-package "GLIDER_MSG")))
(unless (find-package "GLIDER_MSG::DVLMSG")
  (make-package "GLIDER_MSG::DVLMSG"))

(in-package "ROS")
;;//! \htmlinclude dvlmsg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass glider_msg::dvlmsg
  :super ros::object
  :slots (_header _errCode _dataGood _altitudeBeam _bottomVelocityBeam _waterVelocityBeam _waterVelocityCredit _velocityInst _velocityInstFlag _velocityEarth _velocityEarthFlag _waterVelocityInst _waterVelocityInstFlag _waterVelocityEarth _waterVelocityEarthFlag _roll _pitch _heading _altitude _temperature _pressure _salinity _soundSpeed _rawData ))

(defmethod glider_msg::dvlmsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:errCode __errCode) "")
    ((:dataGood __dataGood) (make-array 4 :initial-element 0 :element-type :integer))
    ((:altitudeBeam __altitudeBeam) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:bottomVelocityBeam __bottomVelocityBeam) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:waterVelocityBeam __waterVelocityBeam) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:waterVelocityCredit __waterVelocityCredit) (make-array 4 :initial-element 0.0 :element-type :float))
    ((:velocityInst __velocityInst) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:velocityInstFlag __velocityInstFlag) 0)
    ((:velocityEarth __velocityEarth) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:velocityEarthFlag __velocityEarthFlag) 0)
    ((:waterVelocityInst __waterVelocityInst) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:waterVelocityInstFlag __waterVelocityInstFlag) 0)
    ((:waterVelocityEarth __waterVelocityEarth) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:waterVelocityEarthFlag __waterVelocityEarthFlag) 0)
    ((:roll __roll) 0.0)
    ((:pitch __pitch) 0.0)
    ((:heading __heading) 0.0)
    ((:altitude __altitude) 0.0)
    ((:temperature __temperature) 0.0)
    ((:pressure __pressure) 0.0)
    ((:salinity __salinity) 0.0)
    ((:soundSpeed __soundSpeed) 0.0)
    ((:rawData __rawData) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _errCode (string __errCode))
   (setq _dataGood __dataGood)
   (setq _altitudeBeam __altitudeBeam)
   (setq _bottomVelocityBeam __bottomVelocityBeam)
   (setq _waterVelocityBeam __waterVelocityBeam)
   (setq _waterVelocityCredit __waterVelocityCredit)
   (setq _velocityInst __velocityInst)
   (setq _velocityInstFlag (round __velocityInstFlag))
   (setq _velocityEarth __velocityEarth)
   (setq _velocityEarthFlag (round __velocityEarthFlag))
   (setq _waterVelocityInst __waterVelocityInst)
   (setq _waterVelocityInstFlag (round __waterVelocityInstFlag))
   (setq _waterVelocityEarth __waterVelocityEarth)
   (setq _waterVelocityEarthFlag (round __waterVelocityEarthFlag))
   (setq _roll (float __roll))
   (setq _pitch (float __pitch))
   (setq _heading (float __heading))
   (setq _altitude (float __altitude))
   (setq _temperature (float __temperature))
   (setq _pressure (float __pressure))
   (setq _salinity (float __salinity))
   (setq _soundSpeed (float __soundSpeed))
   (setq _rawData (string __rawData))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:errCode
   (&optional __errCode)
   (if __errCode (setq _errCode __errCode)) _errCode)
  (:dataGood
   (&optional __dataGood)
   (if __dataGood (setq _dataGood __dataGood)) _dataGood)
  (:altitudeBeam
   (&optional __altitudeBeam)
   (if __altitudeBeam (setq _altitudeBeam __altitudeBeam)) _altitudeBeam)
  (:bottomVelocityBeam
   (&optional __bottomVelocityBeam)
   (if __bottomVelocityBeam (setq _bottomVelocityBeam __bottomVelocityBeam)) _bottomVelocityBeam)
  (:waterVelocityBeam
   (&optional __waterVelocityBeam)
   (if __waterVelocityBeam (setq _waterVelocityBeam __waterVelocityBeam)) _waterVelocityBeam)
  (:waterVelocityCredit
   (&optional __waterVelocityCredit)
   (if __waterVelocityCredit (setq _waterVelocityCredit __waterVelocityCredit)) _waterVelocityCredit)
  (:velocityInst
   (&optional __velocityInst)
   (if __velocityInst (setq _velocityInst __velocityInst)) _velocityInst)
  (:velocityInstFlag
   (&optional __velocityInstFlag)
   (if __velocityInstFlag (setq _velocityInstFlag __velocityInstFlag)) _velocityInstFlag)
  (:velocityEarth
   (&optional __velocityEarth)
   (if __velocityEarth (setq _velocityEarth __velocityEarth)) _velocityEarth)
  (:velocityEarthFlag
   (&optional __velocityEarthFlag)
   (if __velocityEarthFlag (setq _velocityEarthFlag __velocityEarthFlag)) _velocityEarthFlag)
  (:waterVelocityInst
   (&optional __waterVelocityInst)
   (if __waterVelocityInst (setq _waterVelocityInst __waterVelocityInst)) _waterVelocityInst)
  (:waterVelocityInstFlag
   (&optional __waterVelocityInstFlag)
   (if __waterVelocityInstFlag (setq _waterVelocityInstFlag __waterVelocityInstFlag)) _waterVelocityInstFlag)
  (:waterVelocityEarth
   (&optional __waterVelocityEarth)
   (if __waterVelocityEarth (setq _waterVelocityEarth __waterVelocityEarth)) _waterVelocityEarth)
  (:waterVelocityEarthFlag
   (&optional __waterVelocityEarthFlag)
   (if __waterVelocityEarthFlag (setq _waterVelocityEarthFlag __waterVelocityEarthFlag)) _waterVelocityEarthFlag)
  (:roll
   (&optional __roll)
   (if __roll (setq _roll __roll)) _roll)
  (:pitch
   (&optional __pitch)
   (if __pitch (setq _pitch __pitch)) _pitch)
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:altitude
   (&optional __altitude)
   (if __altitude (setq _altitude __altitude)) _altitude)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:pressure
   (&optional __pressure)
   (if __pressure (setq _pressure __pressure)) _pressure)
  (:salinity
   (&optional __salinity)
   (if __salinity (setq _salinity __salinity)) _salinity)
  (:soundSpeed
   (&optional __soundSpeed)
   (if __soundSpeed (setq _soundSpeed __soundSpeed)) _soundSpeed)
  (:rawData
   (&optional __rawData)
   (if __rawData (setq _rawData __rawData)) _rawData)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _errCode
    4 (length _errCode)
    ;; int16[4] _dataGood
    (* 2    4)
    ;; float32[4] _altitudeBeam
    (* 4    4)
    ;; float32[4] _bottomVelocityBeam
    (* 4    4)
    ;; float32[4] _waterVelocityBeam
    (* 4    4)
    ;; float32[4] _waterVelocityCredit
    (* 4    4)
    ;; float32[3] _velocityInst
    (* 4    3)
    ;; int16 _velocityInstFlag
    2
    ;; float32[3] _velocityEarth
    (* 4    3)
    ;; int16 _velocityEarthFlag
    2
    ;; float32[3] _waterVelocityInst
    (* 4    3)
    ;; int16 _waterVelocityInstFlag
    2
    ;; float32[3] _waterVelocityEarth
    (* 4    3)
    ;; int16 _waterVelocityEarthFlag
    2
    ;; float32 _roll
    4
    ;; float32 _pitch
    4
    ;; float32 _heading
    4
    ;; float32 _altitude
    4
    ;; float32 _temperature
    4
    ;; float32 _pressure
    4
    ;; float32 _salinity
    4
    ;; float32 _soundSpeed
    4
    ;; string _rawData
    4 (length _rawData)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _errCode
       (write-long (length _errCode) s) (princ _errCode s)
     ;; int16[4] _dataGood
     (dotimes (i 4)
       (write-word (elt _dataGood i) s)
       )
     ;; float32[4] _altitudeBeam
     (dotimes (i 4)
       (sys::poke (elt _altitudeBeam i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[4] _bottomVelocityBeam
     (dotimes (i 4)
       (sys::poke (elt _bottomVelocityBeam i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[4] _waterVelocityBeam
     (dotimes (i 4)
       (sys::poke (elt _waterVelocityBeam i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[4] _waterVelocityCredit
     (dotimes (i 4)
       (sys::poke (elt _waterVelocityCredit i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32[3] _velocityInst
     (dotimes (i 3)
       (sys::poke (elt _velocityInst i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int16 _velocityInstFlag
       (write-word _velocityInstFlag s)
     ;; float32[3] _velocityEarth
     (dotimes (i 3)
       (sys::poke (elt _velocityEarth i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int16 _velocityEarthFlag
       (write-word _velocityEarthFlag s)
     ;; float32[3] _waterVelocityInst
     (dotimes (i 3)
       (sys::poke (elt _waterVelocityInst i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int16 _waterVelocityInstFlag
       (write-word _waterVelocityInstFlag s)
     ;; float32[3] _waterVelocityEarth
     (dotimes (i 3)
       (sys::poke (elt _waterVelocityEarth i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; int16 _waterVelocityEarthFlag
       (write-word _waterVelocityEarthFlag s)
     ;; float32 _roll
       (sys::poke _roll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pitch
       (sys::poke _pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _heading
       (sys::poke _heading (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _altitude
       (sys::poke _altitude (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _temperature
       (sys::poke _temperature (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _pressure
       (sys::poke _pressure (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _salinity
       (sys::poke _salinity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _soundSpeed
       (sys::poke _soundSpeed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _rawData
       (write-long (length _rawData) s) (princ _rawData s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _errCode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _errCode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int16[4] _dataGood
   (dotimes (i (length _dataGood))
     (setf (elt _dataGood i) (sys::peek buf ptr- :short)) (incf ptr- 2)
     )
   ;; float32[4] _altitudeBeam
   (dotimes (i (length _altitudeBeam))
     (setf (elt _altitudeBeam i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[4] _bottomVelocityBeam
   (dotimes (i (length _bottomVelocityBeam))
     (setf (elt _bottomVelocityBeam i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[4] _waterVelocityBeam
   (dotimes (i (length _waterVelocityBeam))
     (setf (elt _waterVelocityBeam i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[4] _waterVelocityCredit
   (dotimes (i (length _waterVelocityCredit))
     (setf (elt _waterVelocityCredit i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; float32[3] _velocityInst
   (dotimes (i (length _velocityInst))
     (setf (elt _velocityInst i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; int16 _velocityInstFlag
     (setq _velocityInstFlag (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32[3] _velocityEarth
   (dotimes (i (length _velocityEarth))
     (setf (elt _velocityEarth i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; int16 _velocityEarthFlag
     (setq _velocityEarthFlag (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32[3] _waterVelocityInst
   (dotimes (i (length _waterVelocityInst))
     (setf (elt _waterVelocityInst i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; int16 _waterVelocityInstFlag
     (setq _waterVelocityInstFlag (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32[3] _waterVelocityEarth
   (dotimes (i (length _waterVelocityEarth))
     (setf (elt _waterVelocityEarth i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; int16 _waterVelocityEarthFlag
     (setq _waterVelocityEarthFlag (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float32 _roll
     (setq _roll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pitch
     (setq _pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _heading
     (setq _heading (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _altitude
     (setq _altitude (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _temperature
     (setq _temperature (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _pressure
     (setq _pressure (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _salinity
     (setq _salinity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _soundSpeed
     (setq _soundSpeed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _rawData
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _rawData (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get glider_msg::dvlmsg :md5sum-) "c18145d5becba32686a61649fd23cb97")
(setf (get glider_msg::dvlmsg :datatype-) "glider_msg/dvlmsg")
(setf (get glider_msg::dvlmsg :definition-)
      "Header header
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

")



(provide :glider_msg/dvlmsg "c18145d5becba32686a61649fd23cb97")


