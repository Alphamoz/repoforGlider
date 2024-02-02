; Auto-generated. Do not edit!


(cl:in-package glider_msg-msg)


;//! \htmlinclude miniCTmsg.msg.html

(cl:defclass <miniCTmsg> (roslisp-msg-protocol:ros-message)
  ((temperature
    :reader temperature
    :initarg :temperature
    :type cl:float
    :initform 0.0)
   (conductivity
    :reader conductivity
    :initarg :conductivity
    :type cl:float
    :initform 0.0))
)

(cl:defclass miniCTmsg (<miniCTmsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <miniCTmsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'miniCTmsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name glider_msg-msg:<miniCTmsg> is deprecated: use glider_msg-msg:miniCTmsg instead.")))

(cl:ensure-generic-function 'temperature-val :lambda-list '(m))
(cl:defmethod temperature-val ((m <miniCTmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:temperature-val is deprecated.  Use glider_msg-msg:temperature instead.")
  (temperature m))

(cl:ensure-generic-function 'conductivity-val :lambda-list '(m))
(cl:defmethod conductivity-val ((m <miniCTmsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader glider_msg-msg:conductivity-val is deprecated.  Use glider_msg-msg:conductivity instead.")
  (conductivity m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <miniCTmsg>) ostream)
  "Serializes a message object of type '<miniCTmsg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'conductivity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <miniCTmsg>) istream)
  "Deserializes a message object of type '<miniCTmsg>"
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
    (cl:setf (cl:slot-value msg 'conductivity) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<miniCTmsg>)))
  "Returns string type for a message object of type '<miniCTmsg>"
  "glider_msg/miniCTmsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'miniCTmsg)))
  "Returns string type for a message object of type 'miniCTmsg"
  "glider_msg/miniCTmsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<miniCTmsg>)))
  "Returns md5sum for a message object of type '<miniCTmsg>"
  "a6f0344464bd3a01fae94a0f61d3ba94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'miniCTmsg)))
  "Returns md5sum for a message object of type 'miniCTmsg"
  "a6f0344464bd3a01fae94a0f61d3ba94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<miniCTmsg>)))
  "Returns full string definition for message of type '<miniCTmsg>"
  (cl:format cl:nil "float32 temperature~%float32 conductivity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'miniCTmsg)))
  "Returns full string definition for message of type 'miniCTmsg"
  (cl:format cl:nil "float32 temperature~%float32 conductivity~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <miniCTmsg>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <miniCTmsg>))
  "Converts a ROS message object to a list"
  (cl:list 'miniCTmsg
    (cl:cons ':temperature (temperature msg))
    (cl:cons ':conductivity (conductivity msg))
))
