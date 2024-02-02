;; Auto-generated. Do not edit!


(when (boundp 'glider_msg::miniCTmsg)
  (if (not (find-package "GLIDER_MSG"))
    (make-package "GLIDER_MSG"))
  (shadow 'miniCTmsg (find-package "GLIDER_MSG")))
(unless (find-package "GLIDER_MSG::MINICTMSG")
  (make-package "GLIDER_MSG::MINICTMSG"))

(in-package "ROS")
;;//! \htmlinclude miniCTmsg.msg.html


(defclass glider_msg::miniCTmsg
  :super ros::object
  :slots (_temperature _conductivity ))

(defmethod glider_msg::miniCTmsg
  (:init
   (&key
    ((:temperature __temperature) 0.0)
    ((:conductivity __conductivity) 0.0)
    )
   (send-super :init)
   (setq _temperature (float __temperature))
   (setq _conductivity (float __conductivity))
   self)
  (:temperature
   (&optional __temperature)
   (if __temperature (setq _temperature __temperature)) _temperature)
  (:conductivity
   (&optional __conductivity)
   (if __conductivity (setq _conductivity __conductivity)) _conductivity)
  (:serialization-length
   ()
   (+
    ;; float32 _temperature
    4
    ;; float32 _conductivity
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _temperature
       (sys::poke _temperature (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _conductivity
       (sys::poke _conductivity (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _temperature
     (setq _temperature (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _conductivity
     (setq _conductivity (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get glider_msg::miniCTmsg :md5sum-) "a6f0344464bd3a01fae94a0f61d3ba94")
(setf (get glider_msg::miniCTmsg :datatype-) "glider_msg/miniCTmsg")
(setf (get glider_msg::miniCTmsg :definition-)
      "float32 temperature
float32 conductivity

")



(provide :glider_msg/miniCTmsg "a6f0344464bd3a01fae94a0f61d3ba94")


