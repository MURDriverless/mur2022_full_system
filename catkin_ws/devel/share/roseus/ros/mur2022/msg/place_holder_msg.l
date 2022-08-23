;; Auto-generated. Do not edit!


(when (boundp 'mur2022::place_holder_msg)
  (if (not (find-package "MUR2022"))
    (make-package "MUR2022"))
  (shadow 'place_holder_msg (find-package "MUR2022")))
(unless (find-package "MUR2022::PLACE_HOLDER_MSG")
  (make-package "MUR2022::PLACE_HOLDER_MSG"))

(in-package "ROS")
;;//! \htmlinclude place_holder_msg.msg.html


(defclass mur2022::place_holder_msg
  :super ros::object
  :slots (_temp_bool _temp_uint32 _temp_int32 _temp_float32 _temp_float64 _temp_string _temp_float64_array ))

(defmethod mur2022::place_holder_msg
  (:init
   (&key
    ((:temp_bool __temp_bool) nil)
    ((:temp_uint32 __temp_uint32) 0)
    ((:temp_int32 __temp_int32) 0)
    ((:temp_float32 __temp_float32) 0.0)
    ((:temp_float64 __temp_float64) 0.0)
    ((:temp_string __temp_string) "")
    ((:temp_float64_array __temp_float64_array) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _temp_bool __temp_bool)
   (setq _temp_uint32 (round __temp_uint32))
   (setq _temp_int32 (round __temp_int32))
   (setq _temp_float32 (float __temp_float32))
   (setq _temp_float64 (float __temp_float64))
   (setq _temp_string (string __temp_string))
   (setq _temp_float64_array __temp_float64_array)
   self)
  (:temp_bool
   (&optional __temp_bool)
   (if __temp_bool (setq _temp_bool __temp_bool)) _temp_bool)
  (:temp_uint32
   (&optional __temp_uint32)
   (if __temp_uint32 (setq _temp_uint32 __temp_uint32)) _temp_uint32)
  (:temp_int32
   (&optional __temp_int32)
   (if __temp_int32 (setq _temp_int32 __temp_int32)) _temp_int32)
  (:temp_float32
   (&optional __temp_float32)
   (if __temp_float32 (setq _temp_float32 __temp_float32)) _temp_float32)
  (:temp_float64
   (&optional __temp_float64)
   (if __temp_float64 (setq _temp_float64 __temp_float64)) _temp_float64)
  (:temp_string
   (&optional __temp_string)
   (if __temp_string (setq _temp_string __temp_string)) _temp_string)
  (:temp_float64_array
   (&optional __temp_float64_array)
   (if __temp_float64_array (setq _temp_float64_array __temp_float64_array)) _temp_float64_array)
  (:serialization-length
   ()
   (+
    ;; bool _temp_bool
    1
    ;; uint32 _temp_uint32
    4
    ;; int32 _temp_int32
    4
    ;; float32 _temp_float32
    4
    ;; float64 _temp_float64
    8
    ;; string _temp_string
    4 (length _temp_string)
    ;; float64[] _temp_float64_array
    (* 8    (length _temp_float64_array)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _temp_bool
       (if _temp_bool (write-byte -1 s) (write-byte 0 s))
     ;; uint32 _temp_uint32
       (write-long _temp_uint32 s)
     ;; int32 _temp_int32
       (write-long _temp_int32 s)
     ;; float32 _temp_float32
       (sys::poke _temp_float32 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float64 _temp_float64
       (sys::poke _temp_float64 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _temp_string
       (write-long (length _temp_string) s) (princ _temp_string s)
     ;; float64[] _temp_float64_array
     (write-long (length _temp_float64_array) s)
     (dotimes (i (length _temp_float64_array))
       (sys::poke (elt _temp_float64_array i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _temp_bool
     (setq _temp_bool (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; uint32 _temp_uint32
     (setq _temp_uint32 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _temp_int32
     (setq _temp_int32 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _temp_float32
     (setq _temp_float32 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float64 _temp_float64
     (setq _temp_float64 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _temp_string
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _temp_string (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64[] _temp_float64_array
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _temp_float64_array (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _temp_float64_array i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get mur2022::place_holder_msg :md5sum-) "13548be3cae6b9d8d8453c21a93e0345")
(setf (get mur2022::place_holder_msg :datatype-) "mur2022/place_holder_msg")
(setf (get mur2022::place_holder_msg :definition-)
      "bool temp_bool
uint32 temp_uint32
int32 temp_int32
float32 temp_float32
float64 temp_float64
string temp_string
float64[] temp_float64_array
")



(provide :mur2022/place_holder_msg "13548be3cae6b9d8d8453c21a93e0345")


