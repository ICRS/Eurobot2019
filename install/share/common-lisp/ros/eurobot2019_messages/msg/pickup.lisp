; Auto-generated. Do not edit!


(cl:in-package eurobot2019_messages-msg)


;//! \htmlinclude pickup.msg.html

(cl:defclass <pickup> (roslisp-msg-protocol:ros-message)
  ((is_vertical
    :reader is_vertical
    :initarg :is_vertical
    :type cl:boolean
    :initform cl:nil)
   (pos_reached
    :reader pos_reached
    :initarg :pos_reached
    :type cl:boolean
    :initform cl:nil)
   (colour
    :reader colour
    :initarg :colour
    :type cl:fixnum
    :initform 0))
)

(cl:defclass pickup (<pickup>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pickup>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pickup)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eurobot2019_messages-msg:<pickup> is deprecated: use eurobot2019_messages-msg:pickup instead.")))

(cl:ensure-generic-function 'is_vertical-val :lambda-list '(m))
(cl:defmethod is_vertical-val ((m <pickup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:is_vertical-val is deprecated.  Use eurobot2019_messages-msg:is_vertical instead.")
  (is_vertical m))

(cl:ensure-generic-function 'pos_reached-val :lambda-list '(m))
(cl:defmethod pos_reached-val ((m <pickup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:pos_reached-val is deprecated.  Use eurobot2019_messages-msg:pos_reached instead.")
  (pos_reached m))

(cl:ensure-generic-function 'colour-val :lambda-list '(m))
(cl:defmethod colour-val ((m <pickup>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:colour-val is deprecated.  Use eurobot2019_messages-msg:colour instead.")
  (colour m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pickup>) ostream)
  "Serializes a message object of type '<pickup>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_vertical) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pos_reached) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'colour)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pickup>) istream)
  "Deserializes a message object of type '<pickup>"
    (cl:setf (cl:slot-value msg 'is_vertical) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'pos_reached) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'colour)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pickup>)))
  "Returns string type for a message object of type '<pickup>"
  "eurobot2019_messages/pickup")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pickup)))
  "Returns string type for a message object of type 'pickup"
  "eurobot2019_messages/pickup")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pickup>)))
  "Returns md5sum for a message object of type '<pickup>"
  "fc43b8165e5ce088844c3951cd1006fd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pickup)))
  "Returns md5sum for a message object of type 'pickup"
  "fc43b8165e5ce088844c3951cd1006fd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pickup>)))
  "Returns full string definition for message of type '<pickup>"
  (cl:format cl:nil "# Is the target puck vertical?~%bool is_vertical~%~%# Have we reached the correct position yet?~%# Note that the picker will be told to open up and get to the right z~%# Before continuing by setting this to false~%bool pos_reached~%~%# 0 is \"idle at bottom front\"~%# 1 is red~%# 2 is green~%# 3 is blue~%# 4 is gold~%# 5 is \"idle at top back\"~%uint8 colour~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pickup)))
  "Returns full string definition for message of type 'pickup"
  (cl:format cl:nil "# Is the target puck vertical?~%bool is_vertical~%~%# Have we reached the correct position yet?~%# Note that the picker will be told to open up and get to the right z~%# Before continuing by setting this to false~%bool pos_reached~%~%# 0 is \"idle at bottom front\"~%# 1 is red~%# 2 is green~%# 3 is blue~%# 4 is gold~%# 5 is \"idle at top back\"~%uint8 colour~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pickup>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pickup>))
  "Converts a ROS message object to a list"
  (cl:list 'pickup
    (cl:cons ':is_vertical (is_vertical msg))
    (cl:cons ':pos_reached (pos_reached msg))
    (cl:cons ':colour (colour msg))
))
