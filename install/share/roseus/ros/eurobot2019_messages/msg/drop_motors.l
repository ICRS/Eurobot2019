;; Auto-generated. Do not edit!


(when (boundp 'eurobot2019_messages::drop_motors)
  (if (not (find-package "EUROBOT2019_MESSAGES"))
    (make-package "EUROBOT2019_MESSAGES"))
  (shadow 'drop_motors (find-package "EUROBOT2019_MESSAGES")))
(unless (find-package "EUROBOT2019_MESSAGES::DROP_MOTORS")
  (make-package "EUROBOT2019_MESSAGES::DROP_MOTORS"))

(in-package "ROS")
;;//! \htmlinclude drop_motors.msg.html


(defclass eurobot2019_messages::drop_motors
  :super ros::object
  :slots (_left_z _left_x _middle_z _middle_x _right_z _right_x ))

(defmethod eurobot2019_messages::drop_motors
  (:init
   (&key
    ((:left_z __left_z) 0.0)
    ((:left_x __left_x) 0.0)
    ((:middle_z __middle_z) 0.0)
    ((:middle_x __middle_x) 0.0)
    ((:right_z __right_z) 0.0)
    ((:right_x __right_x) 0.0)
    )
   (send-super :init)
   (setq _left_z (float __left_z))
   (setq _left_x (float __left_x))
   (setq _middle_z (float __middle_z))
   (setq _middle_x (float __middle_x))
   (setq _right_z (float __right_z))
   (setq _right_x (float __right_x))
   self)
  (:left_z
   (&optional __left_z)
   (if __left_z (setq _left_z __left_z)) _left_z)
  (:left_x
   (&optional __left_x)
   (if __left_x (setq _left_x __left_x)) _left_x)
  (:middle_z
   (&optional __middle_z)
   (if __middle_z (setq _middle_z __middle_z)) _middle_z)
  (:middle_x
   (&optional __middle_x)
   (if __middle_x (setq _middle_x __middle_x)) _middle_x)
  (:right_z
   (&optional __right_z)
   (if __right_z (setq _right_z __right_z)) _right_z)
  (:right_x
   (&optional __right_x)
   (if __right_x (setq _right_x __right_x)) _right_x)
  (:serialization-length
   ()
   (+
    ;; float32 _left_z
    4
    ;; float32 _left_x
    4
    ;; float32 _middle_z
    4
    ;; float32 _middle_x
    4
    ;; float32 _right_z
    4
    ;; float32 _right_x
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _left_z
       (sys::poke _left_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _left_x
       (sys::poke _left_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _middle_z
       (sys::poke _middle_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _middle_x
       (sys::poke _middle_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_z
       (sys::poke _right_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _right_x
       (sys::poke _right_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _left_z
     (setq _left_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _left_x
     (setq _left_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _middle_z
     (setq _middle_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _middle_x
     (setq _middle_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_z
     (setq _right_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _right_x
     (setq _right_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get eurobot2019_messages::drop_motors :md5sum-) "da232c9309df34c64bd87bc76c440433")
(setf (get eurobot2019_messages::drop_motors :datatype-) "eurobot2019_messages/drop_motors")
(setf (get eurobot2019_messages::drop_motors :definition-)
      "# The z position of the left stepper motor
float32 left_z
# The x position of the left pusher
float32 left_x
# The z position of the middle stepper motor
float32 middle_z
# The x position of the middle pusher
float32 middle_x
# The z position of the right stepper motor
float32 right_z
# The x position of the right pusher
float32 right_x

")



(provide :eurobot2019_messages/drop_motors "da232c9309df34c64bd87bc76c440433")


