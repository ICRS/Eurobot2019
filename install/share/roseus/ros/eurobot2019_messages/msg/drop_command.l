;; Auto-generated. Do not edit!


(when (boundp 'eurobot2019_messages::drop_command)
  (if (not (find-package "EUROBOT2019_MESSAGES"))
    (make-package "EUROBOT2019_MESSAGES"))
  (shadow 'drop_command (find-package "EUROBOT2019_MESSAGES")))
(unless (find-package "EUROBOT2019_MESSAGES::DROP_COMMAND")
  (make-package "EUROBOT2019_MESSAGES::DROP_COMMAND"))

(in-package "ROS")
;;//! \htmlinclude drop_command.msg.html


(defclass eurobot2019_messages::drop_command
  :super ros::object
  :slots (_left _right _middle ))

(defmethod eurobot2019_messages::drop_command
  (:init
   (&key
    ((:left __left) nil)
    ((:right __right) nil)
    ((:middle __middle) nil)
    )
   (send-super :init)
   (setq _left __left)
   (setq _right __right)
   (setq _middle __middle)
   self)
  (:left
   (&optional __left)
   (if __left (setq _left __left)) _left)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:middle
   (&optional __middle)
   (if __middle (setq _middle __middle)) _middle)
  (:serialization-length
   ()
   (+
    ;; bool _left
    1
    ;; bool _right
    1
    ;; bool _middle
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _left
       (if _left (write-byte -1 s) (write-byte 0 s))
     ;; bool _right
       (if _right (write-byte -1 s) (write-byte 0 s))
     ;; bool _middle
       (if _middle (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _left
     (setq _left (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _right
     (setq _right (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _middle
     (setq _middle (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get eurobot2019_messages::drop_command :md5sum-) "c498bd496c426b7314def7449c775a44")
(setf (get eurobot2019_messages::drop_command :datatype-) "eurobot2019_messages/drop_command")
(setf (get eurobot2019_messages::drop_command :definition-)
      "# Which tower to drop from
# 0 is idle, 1 is unload
bool left
bool right
bool middle

")



(provide :eurobot2019_messages/drop_command "c498bd496c426b7314def7449c775a44")


