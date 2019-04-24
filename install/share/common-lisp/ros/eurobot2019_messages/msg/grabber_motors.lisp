; Auto-generated. Do not edit!


(cl:in-package eurobot2019_messages-msg)


;//! \htmlinclude grabber_motors.msg.html

(cl:defclass <grabber_motors> (roslisp-msg-protocol:ros-message)
  ((z_pos_mm
    :reader z_pos_mm
    :initarg :z_pos_mm
    :type cl:float
    :initform 0.0)
   (open_pos_mm
    :reader open_pos_mm
    :initarg :open_pos_mm
    :type cl:float
    :initform 0.0)
   (z_twist_rad
    :reader z_twist_rad
    :initarg :z_twist_rad
    :type cl:float
    :initform 0.0)
   (servo_state
    :reader servo_state
    :initarg :servo_state
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass grabber_motors (<grabber_motors>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <grabber_motors>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'grabber_motors)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eurobot2019_messages-msg:<grabber_motors> is deprecated: use eurobot2019_messages-msg:grabber_motors instead.")))

(cl:ensure-generic-function 'z_pos_mm-val :lambda-list '(m))
(cl:defmethod z_pos_mm-val ((m <grabber_motors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:z_pos_mm-val is deprecated.  Use eurobot2019_messages-msg:z_pos_mm instead.")
  (z_pos_mm m))

(cl:ensure-generic-function 'open_pos_mm-val :lambda-list '(m))
(cl:defmethod open_pos_mm-val ((m <grabber_motors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:open_pos_mm-val is deprecated.  Use eurobot2019_messages-msg:open_pos_mm instead.")
  (open_pos_mm m))

(cl:ensure-generic-function 'z_twist_rad-val :lambda-list '(m))
(cl:defmethod z_twist_rad-val ((m <grabber_motors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:z_twist_rad-val is deprecated.  Use eurobot2019_messages-msg:z_twist_rad instead.")
  (z_twist_rad m))

(cl:ensure-generic-function 'servo_state-val :lambda-list '(m))
(cl:defmethod servo_state-val ((m <grabber_motors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:servo_state-val is deprecated.  Use eurobot2019_messages-msg:servo_state instead.")
  (servo_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <grabber_motors>) ostream)
  "Serializes a message object of type '<grabber_motors>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z_pos_mm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'open_pos_mm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z_twist_rad))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'servo_state) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <grabber_motors>) istream)
  "Deserializes a message object of type '<grabber_motors>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_pos_mm) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'open_pos_mm) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_twist_rad) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'servo_state) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<grabber_motors>)))
  "Returns string type for a message object of type '<grabber_motors>"
  "eurobot2019_messages/grabber_motors")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'grabber_motors)))
  "Returns string type for a message object of type 'grabber_motors"
  "eurobot2019_messages/grabber_motors")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<grabber_motors>)))
  "Returns md5sum for a message object of type '<grabber_motors>"
  "1cacf4e8a711d50ed3a69eafde8b2ff7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'grabber_motors)))
  "Returns md5sum for a message object of type 'grabber_motors"
  "1cacf4e8a711d50ed3a69eafde8b2ff7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<grabber_motors>)))
  "Returns full string definition for message of type '<grabber_motors>"
  (cl:format cl:nil "float32 z_pos_mm~%float32 open_pos_mm~%float32 z_twist_rad~%bool servo_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'grabber_motors)))
  "Returns full string definition for message of type 'grabber_motors"
  (cl:format cl:nil "float32 z_pos_mm~%float32 open_pos_mm~%float32 z_twist_rad~%bool servo_state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <grabber_motors>))
  (cl:+ 0
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <grabber_motors>))
  "Converts a ROS message object to a list"
  (cl:list 'grabber_motors
    (cl:cons ':z_pos_mm (z_pos_mm msg))
    (cl:cons ':open_pos_mm (open_pos_mm msg))
    (cl:cons ':z_twist_rad (z_twist_rad msg))
    (cl:cons ':servo_state (servo_state msg))
))
