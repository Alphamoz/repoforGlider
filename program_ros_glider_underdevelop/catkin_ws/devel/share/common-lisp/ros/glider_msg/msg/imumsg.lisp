; Auto-generated. Do not edit!


(cl:in-package glider_msg-msg)


;//! \htmlinclude imumsg.msg.html

(cl:defclass <imumsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (gpsStatus
    :reader gpsStatus
    :initarg :gpsStatus
    :type cl:fixnum
    :initform 0)
   (latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:float
    :initform 0.0)
   (altitude
    :reader altitude
    :initarg :altitude
    :type cl:float
    :initform 0.0)
   (linearX
    :reader linearX
    :initarg :linearX
    :type cl:float
    :initform 0.0)
   (linearY
    :reader linearY
    :initarg :linearY
    :type cl:float
    :initform 0.0)
   (linearZ
    :reader linearZ
    :initarg :linearZ
    :type cl:float
    :initform 0.0)
   (angularX
    :reader angularX
    :initarg :angularX
    :type cl:float
    :initform 0.0)
   (angularY
    :reader angularY
    :initarg :angularY
    :type cl:float
    :initform 0.0)
   (angularZ
    :reader angularZ
    :initarg :angularZ
    :type cl:float
    :initform 0.0)
   (orientationX
    :reader orientationX
    :initarg :orientationX
    :type cl:float
    :initform 0.0)
   (orientationY
    :reader orientationY
    :initarg :orientationY
    :type cl:float
    :initform 0.0)
   (orientationZ
    :reader orientationZ
    :initarg :orientationZ
    :type cl:float
    :initform 0.0)
   (orientationW
    :reader orientationW
    :initarg :orientationW
    :type cl:float
    :initform 0.0)
   (angularVeloX
    :reader angularVeloX
    :initarg :angularVeloX
    :type cl:float
    :initform 0.0)
   (angularVeloY
    :reader angularVeloY
    :initarg :angularVeloY
    :type cl:float
    :initform 0.0)
   (angularVeloZ
    :reader angularVeloZ
    :initarg :angularVeloZ
    :type cl:float
    :initform 0.0)
   (linearAccelerationX
    :reader linearAccelerationX
    :initarg :linearAccelerationX
    :type cl:float
    :initform 0.0)
   (linearAccelerationY
    :reader linearAccelerationY
    :initarg :linearAccelerationY
    :type cl:float
    :initform 0.0)
   (linearAccelerationZ
    :reader linearAccelerationZ
    :initarg :linearAccelerationZ
    :type cl:float
    :initform 0.0)
   (position_covariance
    :reader position_covariance
    :initarg :position_covariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0))
   (orientationCovariance
    :reader orientationCovariance
    :initarg :orientationCovariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0))
   (linearAccelerationCovariance
    :reader linearAccelerationCovariance
    :initarg :linearAccelerationCovariance
    :type (cl:vector cl:float)
   :initform (cl:make-array 9 :element-type 'cl:float :initial-element 0.0))
   (RPY
    :reader RPY
    :initarg :RPY
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (systemStatusMessage
    :reader systemStatusMessage
    :initarg :systemStatusMessage
    :type cl:string
    :initform "")
   (statusLevel
    :reader statusLevel
    :initarg :statusLevel
    :type cl:fixnum
    :initform 0))
)

(cl:defclass imumsg (<imumsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <imumsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'imumsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name glider_msg-msg:<imumsg> is deprecated: use glider_msg-msg:imumsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:header-val is deprecated.  Use glider_msg-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'gpsStatus-val :lambda-list '(m))
(cl:defmethod gpsStatus-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:gpsStatus-val is deprecated.  Use glider_msg-msg:gpsStatus instead.")
  (gpsStatus m))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:latitude-val is deprecated.  Use glider_msg-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:longitude-val is deprecated.  Use glider_msg-msg:longitude instead.")
  (longitude m))

(cl:ensure-generic-function 'altitude-val :lambda-list '(m))
(cl:defmethod altitude-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:altitude-val is deprecated.  Use glider_msg-msg:altitude instead.")
  (altitude m))

(cl:ensure-generic-function 'linearX-val :lambda-list '(m))
(cl:defmethod linearX-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:linearX-val is deprecated.  Use glider_msg-msg:linearX instead.")
  (linearX m))

(cl:ensure-generic-function 'linearY-val :lambda-list '(m))
(cl:defmethod linearY-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:linearY-val is deprecated.  Use glider_msg-msg:linearY instead.")
  (linearY m))

(cl:ensure-generic-function 'linearZ-val :lambda-list '(m))
(cl:defmethod linearZ-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:linearZ-val is deprecated.  Use glider_msg-msg:linearZ instead.")
  (linearZ m))

(cl:ensure-generic-function 'angularX-val :lambda-list '(m))
(cl:defmethod angularX-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:angularX-val is deprecated.  Use glider_msg-msg:angularX instead.")
  (angularX m))

(cl:ensure-generic-function 'angularY-val :lambda-list '(m))
(cl:defmethod angularY-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:angularY-val is deprecated.  Use glider_msg-msg:angularY instead.")
  (angularY m))

(cl:ensure-generic-function 'angularZ-val :lambda-list '(m))
(cl:defmethod angularZ-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:angularZ-val is deprecated.  Use glider_msg-msg:angularZ instead.")
  (angularZ m))

(cl:ensure-generic-function 'orientationX-val :lambda-list '(m))
(cl:defmethod orientationX-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:orientationX-val is deprecated.  Use glider_msg-msg:orientationX instead.")
  (orientationX m))

(cl:ensure-generic-function 'orientationY-val :lambda-list '(m))
(cl:defmethod orientationY-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:orientationY-val is deprecated.  Use glider_msg-msg:orientationY instead.")
  (orientationY m))

(cl:ensure-generic-function 'orientationZ-val :lambda-list '(m))
(cl:defmethod orientationZ-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:orientationZ-val is deprecated.  Use glider_msg-msg:orientationZ instead.")
  (orientationZ m))

(cl:ensure-generic-function 'orientationW-val :lambda-list '(m))
(cl:defmethod orientationW-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:orientationW-val is deprecated.  Use glider_msg-msg:orientationW instead.")
  (orientationW m))

(cl:ensure-generic-function 'angularVeloX-val :lambda-list '(m))
(cl:defmethod angularVeloX-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:angularVeloX-val is deprecated.  Use glider_msg-msg:angularVeloX instead.")
  (angularVeloX m))

(cl:ensure-generic-function 'angularVeloY-val :lambda-list '(m))
(cl:defmethod angularVeloY-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:angularVeloY-val is deprecated.  Use glider_msg-msg:angularVeloY instead.")
  (angularVeloY m))

(cl:ensure-generic-function 'angularVeloZ-val :lambda-list '(m))
(cl:defmethod angularVeloZ-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:angularVeloZ-val is deprecated.  Use glider_msg-msg:angularVeloZ instead.")
  (angularVeloZ m))

(cl:ensure-generic-function 'linearAccelerationX-val :lambda-list '(m))
(cl:defmethod linearAccelerationX-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:linearAccelerationX-val is deprecated.  Use glider_msg-msg:linearAccelerationX instead.")
  (linearAccelerationX m))

(cl:ensure-generic-function 'linearAccelerationY-val :lambda-list '(m))
(cl:defmethod linearAccelerationY-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:linearAccelerationY-val is deprecated.  Use glider_msg-msg:linearAccelerationY instead.")
  (linearAccelerationY m))

(cl:ensure-generic-function 'linearAccelerationZ-val :lambda-list '(m))
(cl:defmethod linearAccelerationZ-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:linearAccelerationZ-val is deprecated.  Use glider_msg-msg:linearAccelerationZ instead.")
  (linearAccelerationZ m))

(cl:ensure-generic-function 'position_covariance-val :lambda-list '(m))
(cl:defmethod position_covariance-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:position_covariance-val is deprecated.  Use glider_msg-msg:position_covariance instead.")
  (position_covariance m))

(cl:ensure-generic-function 'orientationCovariance-val :lambda-list '(m))
(cl:defmethod orientationCovariance-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:orientationCovariance-val is deprecated.  Use glider_msg-msg:orientationCovariance instead.")
  (orientationCovariance m))

(cl:ensure-generic-function 'linearAccelerationCovariance-val :lambda-list '(m))
(cl:defmethod linearAccelerationCovariance-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:linearAccelerationCovariance-val is deprecated.  Use glider_msg-msg:linearAccelerationCovariance instead.")
  (linearAccelerationCovariance m))

(cl:ensure-generic-function 'RPY-val :lambda-list '(m))
(cl:defmethod RPY-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:RPY-val is deprecated.  Use glider_msg-msg:RPY instead.")
  (RPY m))

(cl:ensure-generic-function 'systemStatusMessage-val :lambda-list '(m))
(cl:defmethod systemStatusMessage-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:systemStatusMessage-val is deprecated.  Use glider_msg-msg:systemStatusMessage instead.")
  (systemStatusMessage m))

(cl:ensure-generic-function 'statusLevel-val :lambda-list '(m))
(cl:defmethod statusLevel-val ((m <imumsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:statusLevel-val is deprecated.  Use glider_msg-msg:statusLevel instead.")
  (statusLevel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <imumsg>) ostream)
  "Serializes a message object of type '<imumsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'gpsStatus)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'longitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'altitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'linearX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'linearY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'linearZ))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angularX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angularY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angularZ))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'orientationX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'orientationY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'orientationZ))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'orientationW))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angularVeloX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angularVeloY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angularVeloZ))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'linearAccelerationX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'linearAccelerationY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'linearAccelerationZ))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'position_covariance))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'orientationCovariance))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'linearAccelerationCovariance))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'RPY))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'systemStatusMessage))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'systemStatusMessage))
  (cl:let* ((signed (cl:slot-value msg 'statusLevel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <imumsg>) istream)
  "Deserializes a message object of type '<imumsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'gpsStatus) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude) (roslisp-utils:decode-single-float-bits bits)))
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
    (cl:setf (cl:slot-value msg 'linearX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linearY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linearZ) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angularX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angularY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angularZ) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'orientationX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'orientationY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'orientationZ) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'orientationW) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angularVeloX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angularVeloY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angularVeloZ) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linearAccelerationX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linearAccelerationY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linearAccelerationZ) (roslisp-utils:decode-single-float-bits bits)))
  (cl:setf (cl:slot-value msg 'position_covariance) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'position_covariance)))
    (cl:dotimes (i 9)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'orientationCovariance) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'orientationCovariance)))
    (cl:dotimes (i 9)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'linearAccelerationCovariance) (cl:make-array 9))
  (cl:let ((vals (cl:slot-value msg 'linearAccelerationCovariance)))
    (cl:dotimes (i 9)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'RPY) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'RPY)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'systemStatusMessage) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'systemStatusMessage) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'statusLevel) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<imumsg>)))
  "Returns string type for a message object of type '<imumsg>"
  "glider_msg/imumsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'imumsg)))
  "Returns string type for a message object of type 'imumsg"
  "glider_msg/imumsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<imumsg>)))
  "Returns md5sum for a message object of type '<imumsg>"
  "d2f0d176490184beac4b63cba6827925")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'imumsg)))
  "Returns md5sum for a message object of type 'imumsg"
  "d2f0d176490184beac4b63cba6827925")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<imumsg>)))
  "Returns full string definition for message of type '<imumsg>"
  (cl:format cl:nil "Header header~%int16 gpsStatus~%float32 latitude~%float32 longitude~%float32 altitude~%float32 linearX~%float32 linearY~%float32 linearZ~%float32 angularX~%float32 angularY~%float32 angularZ~%float32 orientationX~%float32 orientationY~%float32 orientationZ~%float32 orientationW~%float32 angularVeloX~%float32 angularVeloY~%float32 angularVeloZ~%float32 linearAccelerationX~%float32 linearAccelerationY~%float32 linearAccelerationZ~%float64[9] position_covariance~%float64[9] orientationCovariance~%float64[9] linearAccelerationCovariance~%float32[3] RPY~%string systemStatusMessage~%int16 statusLevel~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'imumsg)))
  "Returns full string definition for message of type 'imumsg"
  (cl:format cl:nil "Header header~%int16 gpsStatus~%float32 latitude~%float32 longitude~%float32 altitude~%float32 linearX~%float32 linearY~%float32 linearZ~%float32 angularX~%float32 angularY~%float32 angularZ~%float32 orientationX~%float32 orientationY~%float32 orientationZ~%float32 orientationW~%float32 angularVeloX~%float32 angularVeloY~%float32 angularVeloZ~%float32 linearAccelerationX~%float32 linearAccelerationY~%float32 linearAccelerationZ~%float64[9] position_covariance~%float64[9] orientationCovariance~%float64[9] linearAccelerationCovariance~%float32[3] RPY~%string systemStatusMessage~%int16 statusLevel~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <imumsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'position_covariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'orientationCovariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'linearAccelerationCovariance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'RPY) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:length (cl:slot-value msg 'systemStatusMessage))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <imumsg>))
  "Converts a ROS message object to a list"
  (cl:list 'imumsg
    (cl:cons ':header (header msg))
    (cl:cons ':gpsStatus (gpsStatus msg))
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
    (cl:cons ':altitude (altitude msg))
    (cl:cons ':linearX (linearX msg))
    (cl:cons ':linearY (linearY msg))
    (cl:cons ':linearZ (linearZ msg))
    (cl:cons ':angularX (angularX msg))
    (cl:cons ':angularY (angularY msg))
    (cl:cons ':angularZ (angularZ msg))
    (cl:cons ':orientationX (orientationX msg))
    (cl:cons ':orientationY (orientationY msg))
    (cl:cons ':orientationZ (orientationZ msg))
    (cl:cons ':orientationW (orientationW msg))
    (cl:cons ':angularVeloX (angularVeloX msg))
    (cl:cons ':angularVeloY (angularVeloY msg))
    (cl:cons ':angularVeloZ (angularVeloZ msg))
    (cl:cons ':linearAccelerationX (linearAccelerationX msg))
    (cl:cons ':linearAccelerationY (linearAccelerationY msg))
    (cl:cons ':linearAccelerationZ (linearAccelerationZ msg))
    (cl:cons ':position_covariance (position_covariance msg))
    (cl:cons ':orientationCovariance (orientationCovariance msg))
    (cl:cons ':linearAccelerationCovariance (linearAccelerationCovariance msg))
    (cl:cons ':RPY (RPY msg))
    (cl:cons ':systemStatusMessage (systemStatusMessage msg))
    (cl:cons ':statusLevel (statusLevel msg))
))
