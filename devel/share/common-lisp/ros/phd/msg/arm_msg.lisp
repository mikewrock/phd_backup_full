; Auto-generated. Do not edit!


(cl:in-package phd-msg)


;//! \htmlinclude arm_msg.msg.html

(cl:defclass <arm_msg> (roslisp-msg-protocol:ros-message)
  ((j1
    :reader j1
    :initarg :j1
    :type cl:float
    :initform 0.0)
   (j2
    :reader j2
    :initarg :j2
    :type cl:float
    :initform 0.0)
   (j3
    :reader j3
    :initarg :j3
    :type cl:float
    :initform 0.0)
   (j4
    :reader j4
    :initarg :j4
    :type cl:float
    :initform 0.0)
   (j5
    :reader j5
    :initarg :j5
    :type cl:float
    :initform 0.0)
   (j6
    :reader j6
    :initarg :j6
    :type cl:float
    :initform 0.0)
   (x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0)
   (rx
    :reader rx
    :initarg :rx
    :type cl:float
    :initform 0.0)
   (ry
    :reader ry
    :initarg :ry
    :type cl:float
    :initform 0.0)
   (rz
    :reader rz
    :initarg :rz
    :type cl:float
    :initform 0.0)
   (vel
    :reader vel
    :initarg :vel
    :type cl:float
    :initform 0.0)
   (acc
    :reader acc
    :initarg :acc
    :type cl:float
    :initform 0.0)
   (fig
    :reader fig
    :initarg :fig
    :type cl:integer
    :initform 0)
   (pose
    :reader pose
    :initarg :pose
    :type cl:boolean
    :initform cl:nil)
   (motion_type
    :reader motion_type
    :initarg :motion_type
    :type cl:integer
    :initform 0)
   (user_string
    :reader user_string
    :initarg :user_string
    :type cl:string
    :initform ""))
)

(cl:defclass arm_msg (<arm_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <arm_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'arm_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-msg:<arm_msg> is deprecated: use phd-msg:arm_msg instead.")))

(cl:ensure-generic-function 'j1-val :lambda-list '(m))
(cl:defmethod j1-val ((m <arm_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:j1-val is deprecated.  Use phd-msg:j1 instead.")
  (j1 m))

(cl:ensure-generic-function 'j2-val :lambda-list '(m))
(cl:defmethod j2-val ((m <arm_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:j2-val is deprecated.  Use phd-msg:j2 instead.")
  (j2 m))

(cl:ensure-generic-function 'j3-val :lambda-list '(m))
(cl:defmethod j3-val ((m <arm_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:j3-val is deprecated.  Use phd-msg:j3 instead.")
  (j3 m))

(cl:ensure-generic-function 'j4-val :lambda-list '(m))
(cl:defmethod j4-val ((m <arm_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:j4-val is deprecated.  Use phd-msg:j4 instead.")
  (j4 m))

(cl:ensure-generic-function 'j5-val :lambda-list '(m))
(cl:defmethod j5-val ((m <arm_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:j5-val is deprecated.  Use phd-msg:j5 instead.")
  (j5 m))

(cl:ensure-generic-function 'j6-val :lambda-list '(m))
(cl:defmethod j6-val ((m <arm_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:j6-val is deprecated.  Use phd-msg:j6 instead.")
  (j6 m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <arm_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:x-val is deprecated.  Use phd-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <arm_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:y-val is deprecated.  Use phd-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <arm_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:z-val is deprecated.  Use phd-msg:z instead.")
  (z m))

(cl:ensure-generic-function 'rx-val :lambda-list '(m))
(cl:defmethod rx-val ((m <arm_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:rx-val is deprecated.  Use phd-msg:rx instead.")
  (rx m))

(cl:ensure-generic-function 'ry-val :lambda-list '(m))
(cl:defmethod ry-val ((m <arm_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:ry-val is deprecated.  Use phd-msg:ry instead.")
  (ry m))

(cl:ensure-generic-function 'rz-val :lambda-list '(m))
(cl:defmethod rz-val ((m <arm_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:rz-val is deprecated.  Use phd-msg:rz instead.")
  (rz m))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <arm_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:vel-val is deprecated.  Use phd-msg:vel instead.")
  (vel m))

(cl:ensure-generic-function 'acc-val :lambda-list '(m))
(cl:defmethod acc-val ((m <arm_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:acc-val is deprecated.  Use phd-msg:acc instead.")
  (acc m))

(cl:ensure-generic-function 'fig-val :lambda-list '(m))
(cl:defmethod fig-val ((m <arm_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:fig-val is deprecated.  Use phd-msg:fig instead.")
  (fig m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <arm_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:pose-val is deprecated.  Use phd-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'motion_type-val :lambda-list '(m))
(cl:defmethod motion_type-val ((m <arm_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:motion_type-val is deprecated.  Use phd-msg:motion_type instead.")
  (motion_type m))

(cl:ensure-generic-function 'user_string-val :lambda-list '(m))
(cl:defmethod user_string-val ((m <arm_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader phd-msg:user_string-val is deprecated.  Use phd-msg:user_string instead.")
  (user_string m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <arm_msg>) ostream)
  "Serializes a message object of type '<arm_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'j1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'j2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'j3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'j4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'j5))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'j6))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ry))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'vel))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'acc))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'fig)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pose) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'motion_type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'user_string))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'user_string))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <arm_msg>) istream)
  "Deserializes a message object of type '<arm_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'j1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'j2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'j3) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'j4) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'j5) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'j6) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rx) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ry) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rz) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'vel) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acc) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'fig) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'pose) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'motion_type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'user_string) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'user_string) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<arm_msg>)))
  "Returns string type for a message object of type '<arm_msg>"
  "phd/arm_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'arm_msg)))
  "Returns string type for a message object of type 'arm_msg"
  "phd/arm_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<arm_msg>)))
  "Returns md5sum for a message object of type '<arm_msg>"
  "60ecd3f39bc022e6e6d98a74d7b832ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'arm_msg)))
  "Returns md5sum for a message object of type 'arm_msg"
  "60ecd3f39bc022e6e6d98a74d7b832ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<arm_msg>)))
  "Returns full string definition for message of type '<arm_msg>"
  (cl:format cl:nil "float32 j1~%float32 j2~%float32 j3~%float32 j4~%float32 j5~%float32 j6~%float32 x~%float32 y~%float32 z~%float32 rx~%float32 ry~%float32 rz~%float32 vel~%float32 acc~%int32 fig~%bool pose~%int32 motion_type~%string user_string~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'arm_msg)))
  "Returns full string definition for message of type 'arm_msg"
  (cl:format cl:nil "float32 j1~%float32 j2~%float32 j3~%float32 j4~%float32 j5~%float32 j6~%float32 x~%float32 y~%float32 z~%float32 rx~%float32 ry~%float32 rz~%float32 vel~%float32 acc~%int32 fig~%bool pose~%int32 motion_type~%string user_string~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <arm_msg>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     4
     1
     4
     4 (cl:length (cl:slot-value msg 'user_string))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <arm_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'arm_msg
    (cl:cons ':j1 (j1 msg))
    (cl:cons ':j2 (j2 msg))
    (cl:cons ':j3 (j3 msg))
    (cl:cons ':j4 (j4 msg))
    (cl:cons ':j5 (j5 msg))
    (cl:cons ':j6 (j6 msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
    (cl:cons ':rx (rx msg))
    (cl:cons ':ry (ry msg))
    (cl:cons ':rz (rz msg))
    (cl:cons ':vel (vel msg))
    (cl:cons ':acc (acc msg))
    (cl:cons ':fig (fig msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':motion_type (motion_type msg))
    (cl:cons ':user_string (user_string msg))
))
