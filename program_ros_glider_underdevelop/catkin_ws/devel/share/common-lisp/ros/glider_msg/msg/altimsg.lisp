; Auto-generated. Do not edit!


(cl:in-package glider_msg-msg)


;//! \htmlinclude altimsg.msg.html

(cl:defclass <altimsg> (roslisp-msg-protocol:ros-message)
  ((range
    :reader range
    :initarg :range
    :type cl:float
    :initform 0.0)
   (depth
    :reader depth
    :initarg :depth
    :type cl:float
    :initform 0.0)
   (minRange
    :reader minRange
    :initarg :minRange
    :type cl:float
    :initform 0.0)
   (maxRange
    :reader maxRange
    :initarg :maxRange
    :type cl:float
    :initform 0.0))
)

(cl:defclass altimsg (<altimsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <altimsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'altimsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name glider_msg-msg:<altimsg> is deprecated: use glider_msg-msg:altimsg instead.")))

(cl:ensure-generic-function 'range-val :lambda-list '(m))
(cl:defmethod range-val ((m <altimsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:range-val is deprecated.  Use glider_msg-msg:range instead.")
  (range m))

(cl:ensure-generic-function 'depth-val :lambda-list '(m))
(cl:defmethod depth-val ((m <altimsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:depth-val is deprecated.  Use glider_msg-msg:depth instead.")
  (depth m))

(cl:ensure-generic-function 'minRange-val :lambda-list '(m))
(cl:defmethod minRange-val ((m <altimsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:minRange-val is deprecated.  Use glider_msg-msg:minRange instead.")
  (minRange m))

(cl:ensure-generic-function 'maxRange-val :lambda-list '(m))
(cl:defmethod maxRange-val ((m <altimsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:maxRange-val is deprecated.  Use glider_msg-msg:maxRange instead.")
  (maxRange m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <altimsg>) ostream)
  "Serializes a message object of type '<altimsg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'range))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'depth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'minRange))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'maxRange))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <altimsg>) istream)
  "Deserializes a message object of type '<altimsg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'range) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'depth) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'minRange) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'maxRange) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<altimsg>)))
  "Returns string type for a message object of type '<altimsg>"
  "glider_msg/altimsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'altimsg)))
  "Returns string type for a message object of type 'altimsg"
  "glider_msg/altimsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<altimsg>)))
  "Returns md5sum for a message object of type '<altimsg>"
  "b1a9f9d6caa5b73af43f68a831c5c9fe")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'altimsg)))
  "Returns md5sum for a message object of type 'altimsg"
  "b1a9f9d6caa5b73af43f68a831c5c9fe")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<altimsg>)))
  "Returns full string definition for message of type '<altimsg>"
  (cl:format cl:nil "float32 range~%float32 depth~%float32 minRange~%float32 maxRange~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'altimsg)))
  "Returns full string definition for message of type 'altimsg"
  (cl:format cl:nil "float32 range~%float32 depth~%float32 minRange~%float32 maxRange~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <altimsg>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <altimsg>))
  "Converts a ROS message object to a list"
  (cl:list 'altimsg
    (cl:cons ':range (range msg))
    (cl:cons ':depth (depth msg))
    (cl:cons ':minRange (minRange msg))
    (cl:cons ':maxRange (maxRange msg))
))
