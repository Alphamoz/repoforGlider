;; Auto-generated. Do not edit!


(when (boundp 'glider_msg::altimsg)
  (if (not (find-package "GLIDER_MSG"))
    (make-package "GLIDER_MSG"))
  (shadow 'altimsg (find-package "GLIDER_MSG")))
(unless (find-package "GLIDER_MSG::ALTIMSG")
  (make-package "GLIDER_MSG::ALTIMSG"))

(in-package "ROS")
;;//! \htmlinclude altimsg.msg.html


(defclass glider_msg::altimsg
  :super ros::object
  :slots (_range _depth _minRange _maxRange ))

(defmethod glider_msg::altimsg
  (:init
   (&key
    ((:range __range) 0.0)
    ((:depth __depth) 0.0)
    ((:minRange __minRange) 0.0)
    ((:maxRange __maxRange) 0.0)
    )
   (send-super :init)
   (setq _range (float __range))
   (setq _depth (float __depth))
   (setq _minRange (float __minRange))
   (setq _maxRange (float __maxRange))
   self)
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:depth
   (&optional __depth)
   (if __depth (setq _depth __depth)) _depth)
  (:minRange
   (&optional __minRange)
   (if __minRange (setq _minRange __minRange)) _minRange)
  (:maxRange
   (&optional __maxRange)
   (if __maxRange (setq _maxRange __maxRange)) _maxRange)
  (:serialization-length
   ()
   (+
    ;; float32 _range
    4
    ;; float32 _depth
    4
    ;; float32 _minRange
    4
    ;; float32 _maxRange
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _range
       (sys::poke _range (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _depth
       (sys::poke _depth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _minRange
       (sys::poke _minRange (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _maxRange
       (sys::poke _maxRange (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _range
     (setq _range (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _depth
     (setq _depth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _minRange
     (setq _minRange (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _maxRange
     (setq _maxRange (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get glider_msg::altimsg :md5sum-) "b1a9f9d6caa5b73af43f68a831c5c9fe")
(setf (get glider_msg::altimsg :datatype-) "glider_msg/altimsg")
(setf (get glider_msg::altimsg :definition-)
      "float32 range
float32 depth
float32 minRange
float32 maxRange
")



(provide :glider_msg/altimsg "b1a9f9d6caa5b73af43f68a831c5c9fe")


