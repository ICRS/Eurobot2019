;; Auto-generated. Do not edit!


(when (boundp 'eurobot2019_messages::drop_status)
  (if (not (find-package "EUROBOT2019_MESSAGES"))
    (make-package "EUROBOT2019_MESSAGES"))
  (shadow 'drop_status (find-package "EUROBOT2019_MESSAGES")))
(unless (find-package "EUROBOT2019_MESSAGES::DROP_STATUS")
  (make-package "EUROBOT2019_MESSAGES::DROP_STATUS"))

(in-package "ROS")
;;//! \htmlinclude drop_status.msg.html


(defclass eurobot2019_messages::drop_status
  :super ros::object
  :slots (_left_tower_contents _middle_tower_contents _right_tower_contents ))

(defmethod eurobot2019_messages::drop_status
  (:init
   (&key
    ((:left_tower_contents __left_tower_contents) 0)
    ((:middle_tower_contents __middle_tower_contents) 0)
    ((:right_tower_contents __right_tower_contents) 0)
    )
   (send-super :init)
   (setq _left_tower_contents (round __left_tower_contents))
   (setq _middle_tower_contents (round __middle_tower_contents))
   (setq _right_tower_contents (round __right_tower_contents))
   self)
  (:left_tower_contents
   (&optional __left_tower_contents)
   (if __left_tower_contents (setq _left_tower_contents __left_tower_contents)) _left_tower_contents)
  (:middle_tower_contents
   (&optional __middle_tower_contents)
   (if __middle_tower_contents (setq _middle_tower_contents __middle_tower_contents)) _middle_tower_contents)
  (:right_tower_contents
   (&optional __right_tower_contents)
   (if __right_tower_contents (setq _right_tower_contents __right_tower_contents)) _right_tower_contents)
  (:serialization-length
   ()
   (+
    ;; uint8 _left_tower_contents
    1
    ;; uint8 _middle_tower_contents
    1
    ;; uint8 _right_tower_contents
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _left_tower_contents
       (write-byte _left_tower_contents s)
     ;; uint8 _middle_tower_contents
       (write-byte _middle_tower_contents s)
     ;; uint8 _right_tower_contents
       (write-byte _right_tower_contents s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _left_tower_contents
     (setq _left_tower_contents (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _middle_tower_contents
     (setq _middle_tower_contents (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _right_tower_contents
     (setq _right_tower_contents (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get eurobot2019_messages::drop_status :md5sum-) "86a774b860e3f87b013422373c4196ad")
(setf (get eurobot2019_messages::drop_status :datatype-) "eurobot2019_messages/drop_status")
(setf (get eurobot2019_messages::drop_status :definition-)
      "# The number of pucks in the respective tower
uint8 left_tower_contents
uint8 middle_tower_contents
uint8 right_tower_contents

")



(provide :eurobot2019_messages/drop_status "86a774b860e3f87b013422373c4196ad")


