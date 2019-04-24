; Auto-generated. Do not edit!


(cl:in-package eurobot2019_messages-msg)


;//! \htmlinclude drop_command.msg.html

(cl:defclass <drop_command> (roslisp-msg-protocol:ros-message)
  ((left
    :reader left
    :initarg :left
    :type cl:boolean
    :initform cl:nil)
   (right
    :reader right
    :initarg :right
    :type cl:boolean
    :initform cl:nil)
   (middle
    :reader middle
    :initarg :middle
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass drop_command (<drop_command>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <drop_command>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'drop_command)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name eurobot2019_messages-msg:<drop_command> is deprecated: use eurobot2019_messages-msg:drop_command instead.")))

(cl:ensure-generic-function 'left-val :lambda-list '(m))
(cl:defmethod left-val ((m <drop_command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:left-val is deprecated.  Use eurobot2019_messages-msg:left instead.")
  (left m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <drop_command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:right-val is deprecated.  Use eurobot2019_messages-msg:right instead.")
  (right m))

(cl:ensure-generic-function 'middle-val :lambda-list '(m))
(cl:defmethod middle-val ((m <drop_command>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader eurobot2019_messages-msg:middle-val is deprecated.  Use eurobot2019_messages-msg:middle instead.")
  (middle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <drop_command>) ostream)
  "Serializes a message object of type '<drop_command>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'middle) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <drop_command>) istream)
  "Deserializes a message object of type '<drop_command>"
    (cl:setf (cl:slot-value msg 'left) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'right) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'middle) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<drop_command>)))
  "Returns string type for a message object of type '<drop_command>"
  "eurobot2019_messages/drop_command")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'drop_command)))
  "Returns string type for a message object of type 'drop_command"
  "eurobot2019_messages/drop_command")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<drop_command>)))
  "Returns md5sum for a message object of type '<drop_command>"
  "c498bd496c426b7314def7449c775a44")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'drop_command)))
  "Returns md5sum for a message object of type 'drop_command"
  "c498bd496c426b7314def7449c775a44")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<drop_command>)))
  "Returns full string definition for message of type '<drop_command>"
  (cl:format cl:nil "# Which tower to drop from~%# 0 is idle, 1 is unload~%bool left~%bool right~%bool middle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'drop_command)))
  "Returns full string definition for message of type 'drop_command"
  (cl:format cl:nil "# Which tower to drop from~%# 0 is idle, 1 is unload~%bool left~%bool right~%bool middle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <drop_command>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <drop_command>))
  "Converts a ROS message object to a list"
  (cl:list 'drop_command
    (cl:cons ':left (left msg))
    (cl:cons ':right (right msg))
    (cl:cons ':middle (middle msg))
))
