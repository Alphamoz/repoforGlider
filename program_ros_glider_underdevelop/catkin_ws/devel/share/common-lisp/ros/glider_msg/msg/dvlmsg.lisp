; Auto-generated. Do not edit!


(cl:in-package glider_msg-msg)


;//! \htmlinclude dvlmsg.msg.html

(cl:defclass <dvlmsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (errCode
    :reader errCode
    :initarg :errCode
    :type cl:string
    :initform "")
   (dataGood
    :reader dataGood
    :initarg :dataGood
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 4 :element-type 'cl:fixnum :initial-element 0))
   (altitudeBeam
    :reader altitudeBeam
    :initarg :altitudeBeam
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (bottomVelocityBeam
    :reader bottomVelocityBeam
    :initarg :bottomVelocityBeam
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (waterVelocityBeam
    :reader waterVelocityBeam
    :initarg :waterVelocityBeam
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (waterVelocityCredit
    :reader waterVelocityCredit
    :initarg :waterVelocityCredit
    :type (cl:vector cl:float)
   :initform (cl:make-array 4 :element-type 'cl:float :initial-element 0.0))
   (velocityInst
    :reader velocityInst
    :initarg :velocityInst
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (velocityInstFlag
    :reader velocityInstFlag
    :initarg :velocityInstFlag
    :type cl:fixnum
    :initform 0)
   (velocityEarth
    :reader velocityEarth
    :initarg :velocityEarth
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (velocityEarthFlag
    :reader velocityEarthFlag
    :initarg :velocityEarthFlag
    :type cl:fixnum
    :initform 0)
   (waterVelocityInst
    :reader waterVelocityInst
    :initarg :waterVelocityInst
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (waterVelocityInstFlag
    :reader waterVelocityInstFlag
    :initarg :waterVelocityInstFlag
    :type cl:fixnum
    :initform 0)
   (waterVelocityEarth
    :reader waterVelocityEarth
    :initarg :waterVelocityEarth
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (waterVelocityEarthFlag
    :reader waterVelocityEarthFlag
    :initarg :waterVelocityEarthFlag
    :type cl:fixnum
    :initform 0)
   (roll
    :reader roll
    :initarg :roll
    :type cl:float
    :initform 0.0)
   (pitch
    :reader pitch
    :initarg :pitch
    :type cl:float
    :initform 0.0)
   (heading
    :reader heading
    :initarg :heading
    :type cl:float
    :initform 0.0)
   (altitude
    :reader altitude
    :initarg :altitude
    :type cl:float
    :initform 0.0)
   (temperature
    :reader temperature
    :initarg :temperature
    :type cl:float
    :initform 0.0)
   (pressure
    :reader pressure
    :initarg :pressure
    :type cl:float
    :initform 0.0)
   (salinity
    :reader salinity
    :initarg :salinity
    :type cl:float
    :initform 0.0)
   (soundSpeed
    :reader soundSpeed
    :initarg :soundSpeed
    :type cl:float
    :initform 0.0)
   (rawData
    :reader rawData
    :initarg :rawData
    :type cl:string
    :initform ""))
)

(cl:defclass dvlmsg (<dvlmsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <dvlmsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'dvlmsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name glider_msg-msg:<dvlmsg> is deprecated: use glider_msg-msg:dvlmsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:header-val is deprecated.  Use glider_msg-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'errCode-val :lambda-list '(m))
(cl:defmethod errCode-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:errCode-val is deprecated.  Use glider_msg-msg:errCode instead.")
  (errCode m))

(cl:ensure-generic-function 'dataGood-val :lambda-list '(m))
(cl:defmethod dataGood-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:dataGood-val is deprecated.  Use glider_msg-msg:dataGood instead.")
  (dataGood m))

(cl:ensure-generic-function 'altitudeBeam-val :lambda-list '(m))
(cl:defmethod altitudeBeam-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:altitudeBeam-val is deprecated.  Use glider_msg-msg:altitudeBeam instead.")
  (altitudeBeam m))

(cl:ensure-generic-function 'bottomVelocityBeam-val :lambda-list '(m))
(cl:defmethod bottomVelocityBeam-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:bottomVelocityBeam-val is deprecated.  Use glider_msg-msg:bottomVelocityBeam instead.")
  (bottomVelocityBeam m))

(cl:ensure-generic-function 'waterVelocityBeam-val :lambda-list '(m))
(cl:defmethod waterVelocityBeam-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:waterVelocityBeam-val is deprecated.  Use glider_msg-msg:waterVelocityBeam instead.")
  (waterVelocityBeam m))

(cl:ensure-generic-function 'waterVelocityCredit-val :lambda-list '(m))
(cl:defmethod waterVelocityCredit-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:waterVelocityCredit-val is deprecated.  Use glider_msg-msg:waterVelocityCredit instead.")
  (waterVelocityCredit m))

(cl:ensure-generic-function 'velocityInst-val :lambda-list '(m))
(cl:defmethod velocityInst-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:velocityInst-val is deprecated.  Use glider_msg-msg:velocityInst instead.")
  (velocityInst m))

(cl:ensure-generic-function 'velocityInstFlag-val :lambda-list '(m))
(cl:defmethod velocityInstFlag-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:velocityInstFlag-val is deprecated.  Use glider_msg-msg:velocityInstFlag instead.")
  (velocityInstFlag m))

(cl:ensure-generic-function 'velocityEarth-val :lambda-list '(m))
(cl:defmethod velocityEarth-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:velocityEarth-val is deprecated.  Use glider_msg-msg:velocityEarth instead.")
  (velocityEarth m))

(cl:ensure-generic-function 'velocityEarthFlag-val :lambda-list '(m))
(cl:defmethod velocityEarthFlag-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:velocityEarthFlag-val is deprecated.  Use glider_msg-msg:velocityEarthFlag instead.")
  (velocityEarthFlag m))

(cl:ensure-generic-function 'waterVelocityInst-val :lambda-list '(m))
(cl:defmethod waterVelocityInst-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:waterVelocityInst-val is deprecated.  Use glider_msg-msg:waterVelocityInst instead.")
  (waterVelocityInst m))

(cl:ensure-generic-function 'waterVelocityInstFlag-val :lambda-list '(m))
(cl:defmethod waterVelocityInstFlag-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:waterVelocityInstFlag-val is deprecated.  Use glider_msg-msg:waterVelocityInstFlag instead.")
  (waterVelocityInstFlag m))

(cl:ensure-generic-function 'waterVelocityEarth-val :lambda-list '(m))
(cl:defmethod waterVelocityEarth-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:waterVelocityEarth-val is deprecated.  Use glider_msg-msg:waterVelocityEarth instead.")
  (waterVelocityEarth m))

(cl:ensure-generic-function 'waterVelocityEarthFlag-val :lambda-list '(m))
(cl:defmethod waterVelocityEarthFlag-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:waterVelocityEarthFlag-val is deprecated.  Use glider_msg-msg:waterVelocityEarthFlag instead.")
  (waterVelocityEarthFlag m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:roll-val is deprecated.  Use glider_msg-msg:roll instead.")
  (roll m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:pitch-val is deprecated.  Use glider_msg-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'heading-val :lambda-list '(m))
(cl:defmethod heading-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:heading-val is deprecated.  Use glider_msg-msg:heading instead.")
  (heading m))

(cl:ensure-generic-function 'altitude-val :lambda-list '(m))
(cl:defmethod altitude-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:altitude-val is deprecated.  Use glider_msg-msg:altitude instead.")
  (altitude m))

(cl:ensure-generic-function 'temperature-val :lambda-list '(m))
(cl:defmethod temperature-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:temperature-val is deprecated.  Use glider_msg-msg:temperature instead.")
  (temperature m))

(cl:ensure-generic-function 'pressure-val :lambda-list '(m))
(cl:defmethod pressure-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:pressure-val is deprecated.  Use glider_msg-msg:pressure instead.")
  (pressure m))

(cl:ensure-generic-function 'salinity-val :lambda-list '(m))
(cl:defmethod salinity-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:salinity-val is deprecated.  Use glider_msg-msg:salinity instead.")
  (salinity m))

(cl:ensure-generic-function 'soundSpeed-val :lambda-list '(m))
(cl:defmethod soundSpeed-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:soundSpeed-val is deprecated.  Use glider_msg-msg:soundSpeed instead.")
  (soundSpeed m))

(cl:ensure-generic-function 'rawData-val :lambda-list '(m))
(cl:defmethod rawData-val ((m <dvlmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:rawData-val is deprecated.  Use glider_msg-msg:rawData instead.")
  (rawData m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <dvlmsg>) ostream)
  "Serializes a message object of type '<dvlmsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'errCode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'errCode))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'dataGood))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'altitudeBeam))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'bottomVelocityBeam))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'waterVelocityBeam))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'waterVelocityCredit))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'velocityInst))
  (cl:let* ((signed (cl:slot-value msg 'velocityInstFlag)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'velocityEarth))
  (cl:let* ((signed (cl:slot-value msg 'velocityEarthFlag)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'waterVelocityInst))
  (cl:let* ((signed (cl:slot-value msg 'waterVelocityInstFlag)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'waterVelocityEarth))
  (cl:let* ((signed (cl:slot-value msg 'waterVelocityEarthFlag)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'heading))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'altitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'pressure))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'salinity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'soundSpeed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'rawData))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'rawData))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <dvlmsg>) istream)
  "Deserializes a message object of type '<dvlmsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'errCode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'errCode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:setf (cl:slot-value msg 'dataGood) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'dataGood)))
    (cl:dotimes (i 4)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  (cl:setf (cl:slot-value msg 'altitudeBeam) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'altitudeBeam)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'bottomVelocityBeam) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'bottomVelocityBeam)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'waterVelocityBeam) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'waterVelocityBeam)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'waterVelocityCredit) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'waterVelocityCredit)))
    (cl:dotimes (i 4)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'velocityInst) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'velocityInst)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'velocityInstFlag) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:setf (cl:slot-value msg 'velocityEarth) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'velocityEarth)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'velocityEarthFlag) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:setf (cl:slot-value msg 'waterVelocityInst) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'waterVelocityInst)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'waterVelocityInstFlag) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:setf (cl:slot-value msg 'waterVelocityEarth) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'waterVelocityEarth)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'waterVelocityEarthFlag) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'roll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'heading) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'altitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temperature) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pressure) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'salinity) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'soundSpeed) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'rawData) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'rawData) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<dvlmsg>)))
  "Returns string type for a message object of type '<dvlmsg>"
  "glider_msg/dvlmsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'dvlmsg)))
  "Returns string type for a message object of type 'dvlmsg"
  "glider_msg/dvlmsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<dvlmsg>)))
  "Returns md5sum for a message object of type '<dvlmsg>"
  "c18145d5becba32686a61649fd23cb97")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'dvlmsg)))
  "Returns md5sum for a message object of type 'dvlmsg"
  "c18145d5becba32686a61649fd23cb97")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<dvlmsg>)))
  "Returns full string definition for message of type '<dvlmsg>"
  (cl:format cl:nil "Header header~%string errCode~%int16[4] dataGood~%float32[4] altitudeBeam~%float32[4] bottomVelocityBeam~%float32[4] waterVelocityBeam~%float32[4] waterVelocityCredit~%float32[3] velocityInst~%int16 velocityInstFlag~%float32[3] velocityEarth~%int16 velocityEarthFlag~%float32[3] waterVelocityInst~%int16 waterVelocityInstFlag~%float32[3] waterVelocityEarth~%int16 waterVelocityEarthFlag~%float32 roll~%float32 pitch~%float32 heading~%float32 altitude~%float32 temperature~%float32 pressure~%float32 salinity~%float32 soundSpeed~%string rawData~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'dvlmsg)))
  "Returns full string definition for message of type 'dvlmsg"
  (cl:format cl:nil "Header header~%string errCode~%int16[4] dataGood~%float32[4] altitudeBeam~%float32[4] bottomVelocityBeam~%float32[4] waterVelocityBeam~%float32[4] waterVelocityCredit~%float32[3] velocityInst~%int16 velocityInstFlag~%float32[3] velocityEarth~%int16 velocityEarthFlag~%float32[3] waterVelocityInst~%int16 waterVelocityInstFlag~%float32[3] waterVelocityEarth~%int16 waterVelocityEarthFlag~%float32 roll~%float32 pitch~%float32 heading~%float32 altitude~%float32 temperature~%float32 pressure~%float32 salinity~%float32 soundSpeed~%string rawData~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <dvlmsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'errCode))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'dataGood) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'altitudeBeam) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'bottomVelocityBeam) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'waterVelocityBeam) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'waterVelocityCredit) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'velocityInst) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     2
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'velocityEarth) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     2
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'waterVelocityInst) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     2
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'waterVelocityEarth) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     2
     4
     4
     4
     4
     4
     4
     4
     4
     4 (cl:length (cl:slot-value msg 'rawData))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <dvlmsg>))
  "Converts a ROS message object to a list"
  (cl:list 'dvlmsg
    (cl:cons ':header (header msg))
    (cl:cons ':errCode (errCode msg))
    (cl:cons ':dataGood (dataGood msg))
    (cl:cons ':altitudeBeam (altitudeBeam msg))
    (cl:cons ':bottomVelocityBeam (bottomVelocityBeam msg))
    (cl:cons ':waterVelocityBeam (waterVelocityBeam msg))
    (cl:cons ':waterVelocityCredit (waterVelocityCredit msg))
    (cl:cons ':velocityInst (velocityInst msg))
    (cl:cons ':velocityInstFlag (velocityInstFlag msg))
    (cl:cons ':velocityEarth (velocityEarth msg))
    (cl:cons ':velocityEarthFlag (velocityEarthFlag msg))
    (cl:cons ':waterVelocityInst (waterVelocityInst msg))
    (cl:cons ':waterVelocityInstFlag (waterVelocityInstFlag msg))
    (cl:cons ':waterVelocityEarth (waterVelocityEarth msg))
    (cl:cons ':waterVelocityEarthFlag (waterVelocityEarthFlag msg))
    (cl:cons ':roll (roll msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':heading (heading msg))
    (cl:cons ':altitude (altitude msg))
    (cl:cons ':temperature (temperature msg))
    (cl:cons ':pressure (pressure msg))
    (cl:cons ':salinity (salinity msg))
    (cl:cons ':soundSpeed (soundSpeed msg))
    (cl:cons ':rawData (rawData msg))
))
