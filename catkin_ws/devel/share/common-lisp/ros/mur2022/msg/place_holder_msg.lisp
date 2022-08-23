; Auto-generated. Do not edit!


(cl:in-package mur2022-msg)


;//! \htmlinclude place_holder_msg.msg.html

(cl:defclass <place_holder_msg> (roslisp-msg-protocol:ros-message)
  ((temp_bool
    :reader temp_bool
    :initarg :temp_bool
    :type cl:boolean
    :initform cl:nil)
   (temp_uint32
    :reader temp_uint32
    :initarg :temp_uint32
    :type cl:integer
    :initform 0)
   (temp_int32
    :reader temp_int32
    :initarg :temp_int32
    :type cl:integer
    :initform 0)
   (temp_float32
    :reader temp_float32
    :initarg :temp_float32
    :type cl:float
    :initform 0.0)
   (temp_float64
    :reader temp_float64
    :initarg :temp_float64
    :type cl:float
    :initform 0.0)
   (temp_string
    :reader temp_string
    :initarg :temp_string
    :type cl:string
    :initform "")
   (temp_float64_array
    :reader temp_float64_array
    :initarg :temp_float64_array
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass place_holder_msg (<place_holder_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <place_holder_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'place_holder_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mur2022-msg:<place_holder_msg> is deprecated: use mur2022-msg:place_holder_msg instead.")))

(cl:ensure-generic-function 'temp_bool-val :lambda-list '(m))
(cl:defmethod temp_bool-val ((m <place_holder_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mur2022-msg:temp_bool-val is deprecated.  Use mur2022-msg:temp_bool instead.")
  (temp_bool m))

(cl:ensure-generic-function 'temp_uint32-val :lambda-list '(m))
(cl:defmethod temp_uint32-val ((m <place_holder_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mur2022-msg:temp_uint32-val is deprecated.  Use mur2022-msg:temp_uint32 instead.")
  (temp_uint32 m))

(cl:ensure-generic-function 'temp_int32-val :lambda-list '(m))
(cl:defmethod temp_int32-val ((m <place_holder_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mur2022-msg:temp_int32-val is deprecated.  Use mur2022-msg:temp_int32 instead.")
  (temp_int32 m))

(cl:ensure-generic-function 'temp_float32-val :lambda-list '(m))
(cl:defmethod temp_float32-val ((m <place_holder_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mur2022-msg:temp_float32-val is deprecated.  Use mur2022-msg:temp_float32 instead.")
  (temp_float32 m))

(cl:ensure-generic-function 'temp_float64-val :lambda-list '(m))
(cl:defmethod temp_float64-val ((m <place_holder_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mur2022-msg:temp_float64-val is deprecated.  Use mur2022-msg:temp_float64 instead.")
  (temp_float64 m))

(cl:ensure-generic-function 'temp_string-val :lambda-list '(m))
(cl:defmethod temp_string-val ((m <place_holder_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mur2022-msg:temp_string-val is deprecated.  Use mur2022-msg:temp_string instead.")
  (temp_string m))

(cl:ensure-generic-function 'temp_float64_array-val :lambda-list '(m))
(cl:defmethod temp_float64_array-val ((m <place_holder_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mur2022-msg:temp_float64_array-val is deprecated.  Use mur2022-msg:temp_float64_array instead.")
  (temp_float64_array m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <place_holder_msg>) ostream)
  "Serializes a message object of type '<place_holder_msg>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'temp_bool) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_uint32)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'temp_uint32)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'temp_uint32)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'temp_uint32)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'temp_int32)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'temp_float32))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'temp_float64))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'temp_string))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'temp_string))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'temp_float64_array))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'temp_float64_array))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <place_holder_msg>) istream)
  "Deserializes a message object of type '<place_holder_msg>"
    (cl:setf (cl:slot-value msg 'temp_bool) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'temp_uint32)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'temp_uint32)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'temp_uint32)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'temp_uint32)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'temp_int32) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temp_float32) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temp_float64) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'temp_string) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'temp_string) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'temp_float64_array) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'temp_float64_array)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<place_holder_msg>)))
  "Returns string type for a message object of type '<place_holder_msg>"
  "mur2022/place_holder_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'place_holder_msg)))
  "Returns string type for a message object of type 'place_holder_msg"
  "mur2022/place_holder_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<place_holder_msg>)))
  "Returns md5sum for a message object of type '<place_holder_msg>"
  "13548be3cae6b9d8d8453c21a93e0345")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'place_holder_msg)))
  "Returns md5sum for a message object of type 'place_holder_msg"
  "13548be3cae6b9d8d8453c21a93e0345")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<place_holder_msg>)))
  "Returns full string definition for message of type '<place_holder_msg>"
  (cl:format cl:nil "bool temp_bool~%uint32 temp_uint32~%int32 temp_int32~%float32 temp_float32~%float64 temp_float64~%string temp_string~%float64[] temp_float64_array~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'place_holder_msg)))
  "Returns full string definition for message of type 'place_holder_msg"
  (cl:format cl:nil "bool temp_bool~%uint32 temp_uint32~%int32 temp_int32~%float32 temp_float32~%float64 temp_float64~%string temp_string~%float64[] temp_float64_array~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <place_holder_msg>))
  (cl:+ 0
     1
     4
     4
     4
     8
     4 (cl:length (cl:slot-value msg 'temp_string))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'temp_float64_array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <place_holder_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'place_holder_msg
    (cl:cons ':temp_bool (temp_bool msg))
    (cl:cons ':temp_uint32 (temp_uint32 msg))
    (cl:cons ':temp_int32 (temp_int32 msg))
    (cl:cons ':temp_float32 (temp_float32 msg))
    (cl:cons ':temp_float64 (temp_float64 msg))
    (cl:cons ':temp_string (temp_string msg))
    (cl:cons ':temp_float64_array (temp_float64_array msg))
))
