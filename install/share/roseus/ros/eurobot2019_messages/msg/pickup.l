;; Auto-generated. Do not edit!


(when (boundp 'eurobot2019_messages::pickup)
  (if (not (find-package "EUROBOT2019_MESSAGES"))
    (make-package "EUROBOT2019_MESSAGES"))
  (shadow 'pickup (find-package "EUROBOT2019_MESSAGES")))
(unless (find-package "EUROBOT2019_MESSAGES::PICKUP")
  (make-package "EUROBOT2019_MESSAGES::PICKUP"))

(in-package "ROS")
;;//! \htmlinclude pickup.msg.html


(defclass eurobot2019_messages::pickup
  :super ros::object
  :slots (_is_vertical _pos_reached _colour ))

(defmethod eurobot2019_messages::pickup
  (:init
   (&key
    ((:is_vertical __is_vertical) nil)
    ((:pos_reached __pos_reached) nil)
    ((:colour __colour) 0)
    )
   (send-super :init)
   (setq _is_vertical __is_vertical)
   (setq _pos_reached __pos_reached)
   (setq _colour (round __colour))
   self)
  (:is_vertical
   (&optional __is_vertical)
   (if __is_vertical (setq _is_vertical __is_vertical)) _is_vertical)
  (:pos_reached
   (&optional __pos_reached)
   (if __pos_reached (setq _pos_reached __pos_reached)) _pos_reached)
  (:colour
   (&optional __colour)
   (if __colour (setq _colour __colour)) _colour)
  (:serialization-length
   ()
   (+
    ;; bool _is_vertical
    1
    ;; bool _pos_reached
    1
    ;; uint8 _colour
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _is_vertical
       (if _is_vertical (write-byte -1 s) (write-byte 0 s))
     ;; bool _pos_reached
       (if _pos_reached (write-byte -1 s) (write-byte 0 s))
     ;; uint8 _colour
       (write-byte _colour s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _is_vertical
     (setq _is_vertical (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _pos_reached
     (setq _pos_reached (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint8 _colour
     (setq _colour (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get eurobot2019_messages::pickup :md5sum-) "fc43b8165e5ce088844c3951cd1006fd")
(setf (get eurobot2019_messages::pickup :datatype-) "eurobot2019_messages/pickup")
(setf (get eurobot2019_messages::pickup :definition-)
      "# Is the target puck vertical?
bool is_vertical

# Have we reached the correct position yet?
# Note that the picker will be told to open up and get to the right z
# Before continuing by setting this to false
bool pos_reached

# 0 is \"idle at bottom front\"
# 1 is red
# 2 is green
# 3 is blue
# 4 is gold
# 5 is \"idle at top back\"
uint8 colour

")



(provide :eurobot2019_messages/pickup "fc43b8165e5ce088844c3951cd1006fd")


