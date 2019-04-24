; Auto-generated. Do not edit!


(cl:in-package eurobot2019_messages-msg)


;//! \htmlinclude drop_status.msg.html

(cl:defclass <drop_status> (roslisp-msg-protocol:ros-message)
  ((left_tower_contents
    :reader left_tower_contents
    :initarg :left_tower_contents
    :type cl:fixnum
    :initform 0)
   (middle_tower_contents
    :reader middle_tower_contents
    :initarg :middle_tower_contents
    :type cl:fixnum
    :initform 0)
   (right_tower_contents
    :reader right_tower_contents
    :initarg :right_tower_contents
    :type cl:fixnum
    :initform 0))
)

(cl:defclass drop_status (<drop_status>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <drop_status>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'drop_status)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eurobot2019_messages-msg:<drop_status> is deprecated: use eurobot2019_messages-msg:drop_status instead.")))

(cl:ensure-generic-function 'left_tower_contents-val :lambda-list '(m))
(cl:defmethod left_tower_contents-val ((m <drop_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:left_tower_contents-val is deprecated.  Use eurobot2019_messages-msg:left_tower_contents instead.")
  (left_tower_contents m))

(cl:ensure-generic-function 'middle_tower_contents-val :lambda-list '(m))
(cl:defmethod middle_tower_contents-val ((m <drop_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:middle_tower_contents-val is deprecated.  Use eurobot2019_messages-msg:middle_tower_contents instead.")
  (middle_tower_contents m))

(cl:ensure-generic-function 'right_tower_contents-val :lambda-list '(m))
(cl:defmethod right_tower_contents-val ((m <drop_status>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:right_tower_contents-val is deprecated.  Use eurobot2019_messages-msg:right_tower_contents instead.")
  (right_tower_contents m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <drop_status>) ostream)
  "Serializes a message object of type '<drop_status>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'left_tower_contents)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'middle_tower_contents)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'right_tower_contents)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <drop_status>) istream)
  "Deserializes a message object of type '<drop_status>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'left_tower_contents)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'middle_tower_contents)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'right_tower_contents)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<drop_status>)))
  "Returns string type for a message object of type '<drop_status>"
  "eurobot2019_messages/drop_status")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'drop_status)))
  "Returns string type for a message object of type 'drop_status"
  "eurobot2019_messages/drop_status")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<drop_status>)))
  "Returns md5sum for a message object of type '<drop_status>"
  "86a774b860e3f87b013422373c4196ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'drop_status)))
  "Returns md5sum for a message object of type 'drop_status"
  "86a774b860e3f87b013422373c4196ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<drop_status>)))
  "Returns full string definition for message of type '<drop_status>"
  (cl:format cl:nil "# The number of pucks in the respective tower~%uint8 left_tower_contents~%uint8 middle_tower_contents~%uint8 right_tower_contents~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'drop_status)))
  "Returns full string definition for message of type 'drop_status"
  (cl:format cl:nil "# The number of pucks in the respective tower~%uint8 left_tower_contents~%uint8 middle_tower_contents~%uint8 right_tower_contents~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <drop_status>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <drop_status>))
  "Converts a ROS message object to a list"
  (cl:list 'drop_status
    (cl:cons ':left_tower_contents (left_tower_contents msg))
    (cl:cons ':middle_tower_contents (middle_tower_contents msg))
    (cl:cons ':right_tower_contents (right_tower_contents msg))
))
