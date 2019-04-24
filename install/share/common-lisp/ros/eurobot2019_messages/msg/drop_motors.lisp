; Auto-generated. Do not edit!


(cl:in-package eurobot2019_messages-msg)


;//! \htmlinclude drop_motors.msg.html

(cl:defclass <drop_motors> (roslisp-msg-protocol:ros-message)
  ((left_z
    :reader left_z
    :initarg :left_z
    :type cl:float
    :initform 0.0)
   (left_x
    :reader left_x
    :initarg :left_x
    :type cl:float
    :initform 0.0)
   (middle_z
    :reader middle_z
    :initarg :middle_z
    :type cl:float
    :initform 0.0)
   (middle_x
    :reader middle_x
    :initarg :middle_x
    :type cl:float
    :initform 0.0)
   (right_z
    :reader right_z
    :initarg :right_z
    :type cl:float
    :initform 0.0)
   (right_x
    :reader right_x
    :initarg :right_x
    :type cl:float
    :initform 0.0))
)

(cl:defclass drop_motors (<drop_motors>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <drop_motors>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'drop_motors)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eurobot2019_messages-msg:<drop_motors> is deprecated: use eurobot2019_messages-msg:drop_motors instead.")))

(cl:ensure-generic-function 'left_z-val :lambda-list '(m))
(cl:defmethod left_z-val ((m <drop_motors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:left_z-val is deprecated.  Use eurobot2019_messages-msg:left_z instead.")
  (left_z m))

(cl:ensure-generic-function 'left_x-val :lambda-list '(m))
(cl:defmethod left_x-val ((m <drop_motors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:left_x-val is deprecated.  Use eurobot2019_messages-msg:left_x instead.")
  (left_x m))

(cl:ensure-generic-function 'middle_z-val :lambda-list '(m))
(cl:defmethod middle_z-val ((m <drop_motors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:middle_z-val is deprecated.  Use eurobot2019_messages-msg:middle_z instead.")
  (middle_z m))

(cl:ensure-generic-function 'middle_x-val :lambda-list '(m))
(cl:defmethod middle_x-val ((m <drop_motors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:middle_x-val is deprecated.  Use eurobot2019_messages-msg:middle_x instead.")
  (middle_x m))

(cl:ensure-generic-function 'right_z-val :lambda-list '(m))
(cl:defmethod right_z-val ((m <drop_motors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:right_z-val is deprecated.  Use eurobot2019_messages-msg:right_z instead.")
  (right_z m))

(cl:ensure-generic-function 'right_x-val :lambda-list '(m))
(cl:defmethod right_x-val ((m <drop_motors>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:right_x-val is deprecated.  Use eurobot2019_messages-msg:right_x instead.")
  (right_x m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <drop_motors>) ostream)
  "Serializes a message object of type '<drop_motors>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'left_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'middle_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'middle_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'right_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <drop_motors>) istream)
  "Deserializes a message object of type '<drop_motors>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'middle_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'middle_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_x) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<drop_motors>)))
  "Returns string type for a message object of type '<drop_motors>"
  "eurobot2019_messages/drop_motors")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'drop_motors)))
  "Returns string type for a message object of type 'drop_motors"
  "eurobot2019_messages/drop_motors")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<drop_motors>)))
  "Returns md5sum for a message object of type '<drop_motors>"
  "da232c9309df34c64bd87bc76c440433")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'drop_motors)))
  "Returns md5sum for a message object of type 'drop_motors"
  "da232c9309df34c64bd87bc76c440433")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<drop_motors>)))
  "Returns full string definition for message of type '<drop_motors>"
  (cl:format cl:nil "# The z position of the left stepper motor~%float32 left_z~%# The x position of the left pusher~%float32 left_x~%# The z position of the middle stepper motor~%float32 middle_z~%# The x position of the middle pusher~%float32 middle_x~%# The z position of the right stepper motor~%float32 right_z~%# The x position of the right pusher~%float32 right_x~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'drop_motors)))
  "Returns full string definition for message of type 'drop_motors"
  (cl:format cl:nil "# The z position of the left stepper motor~%float32 left_z~%# The x position of the left pusher~%float32 left_x~%# The z position of the middle stepper motor~%float32 middle_z~%# The x position of the middle pusher~%float32 middle_x~%# The z position of the right stepper motor~%float32 right_z~%# The x position of the right pusher~%float32 right_x~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <drop_motors>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <drop_motors>))
  "Converts a ROS message object to a list"
  (cl:list 'drop_motors
    (cl:cons ':left_z (left_z msg))
    (cl:cons ':left_x (left_x msg))
    (cl:cons ':middle_z (middle_z msg))
    (cl:cons ':middle_x (middle_x msg))
    (cl:cons ':right_z (right_z msg))
    (cl:cons ':right_x (right_x msg))
))
