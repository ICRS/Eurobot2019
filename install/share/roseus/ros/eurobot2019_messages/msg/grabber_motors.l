;; Auto-generated. Do not edit!


(when (boundp 'eurobot2019_messages::grabber_motors)
  (if (not (find-package "EUROBOT2019_MESSAGES"))
    (make-package "EUROBOT2019_MESSAGES"))
  (shadow 'grabber_motors (find-package "EUROBOT2019_MESSAGES")))
(unless (find-package "EUROBOT2019_MESSAGES::GRABBER_MOTORS")
  (make-package "EUROBOT2019_MESSAGES::GRABBER_MOTORS"))

(in-package "ROS")
;;//! \htmlinclude grabber_motors.msg.html


(defclass eurobot2019_messages::grabber_motors
  :super ros::object
  :slots (_z_pos_mm _open_pos_mm _z_twist_rad _servo_state ))

(defmethod eurobot2019_messages::grabber_motors
  (:init
   (&key
    ((:z_pos_mm __z_pos_mm) 0.0)
    ((:open_pos_mm __open_pos_mm) 0.0)
    ((:z_twist_rad __z_twist_rad) 0.0)
    ((:servo_state __servo_state) nil)
    )
   (send-super :init)
   (setq _z_pos_mm (float __z_pos_mm))
   (setq _open_pos_mm (float __open_pos_mm))
   (setq _z_twist_rad (float __z_twist_rad))
   (setq _servo_state __servo_state)
   self)
  (:z_pos_mm
   (&optional __z_pos_mm)
   (if __z_pos_mm (setq _z_pos_mm __z_pos_mm)) _z_pos_mm)
  (:open_pos_mm
   (&optional __open_pos_mm)
   (if __open_pos_mm (setq _open_pos_mm __open_pos_mm)) _open_pos_mm)
  (:z_twist_rad
   (&optional __z_twist_rad)
   (if __z_twist_rad (setq _z_twist_rad __z_twist_rad)) _z_twist_rad)
  (:servo_state
   (&optional __servo_state)
   (if __servo_state (setq _servo_state __servo_state)) _servo_state)
  (:serialization-length
   ()
   (+
    ;; float32 _z_pos_mm
    4
    ;; float32 _open_pos_mm
    4
    ;; float32 _z_twist_rad
    4
    ;; bool _servo_state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _z_pos_mm
       (sys::poke _z_pos_mm (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _open_pos_mm
       (sys::poke _open_pos_mm (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z_twist_rad
       (sys::poke _z_twist_rad (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; bool _servo_state
       (if _servo_state (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _z_pos_mm
     (setq _z_pos_mm (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _open_pos_mm
     (setq _open_pos_mm (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z_twist_rad
     (setq _z_twist_rad (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; bool _servo_state
     (setq _servo_state (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get eurobot2019_messages::grabber_motors :md5sum-) "1cacf4e8a711d50ed3a69eafde8b2ff7")
(setf (get eurobot2019_messages::grabber_motors :datatype-) "eurobot2019_messages/grabber_motors")
(setf (get eurobot2019_messages::grabber_motors :definition-)
      "float32 z_pos_mm
float32 open_pos_mm
float32 z_twist_rad
bool servo_state

")



(provide :eurobot2019_messages/grabber_motors "1cacf4e8a711d50ed3a69eafde8b2ff7")


