; Auto-generated. Do not edit!


(cl:in-package phd-srv)


;//! \htmlinclude empty-request.msg.html

(cl:defclass <empty-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass empty-request (<empty-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <empty-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'empty-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-srv:<empty-request> is deprecated: use phd-srv:empty-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <empty-request>) ostream)
  "Serializes a message object of type '<empty-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <empty-request>) istream)
  "Deserializes a message object of type '<empty-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<empty-request>)))
  "Returns string type for a service object of type '<empty-request>"
  "phd/emptyRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'empty-request)))
  "Returns string type for a service object of type 'empty-request"
  "phd/emptyRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<empty-request>)))
  "Returns md5sum for a message object of type '<empty-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'empty-request)))
  "Returns md5sum for a message object of type 'empty-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<empty-request>)))
  "Returns full string definition for message of type '<empty-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'empty-request)))
  "Returns full string definition for message of type 'empty-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <empty-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <empty-request>))
  "Converts a ROS message object to a list"
  (cl:list 'empty-request
))
;//! \htmlinclude empty-response.msg.html

(cl:defclass <empty-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass empty-response (<empty-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <empty-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'empty-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name phd-srv:<empty-response> is deprecated: use phd-srv:empty-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <empty-response>) ostream)
  "Serializes a message object of type '<empty-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <empty-response>) istream)
  "Deserializes a message object of type '<empty-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<empty-response>)))
  "Returns string type for a service object of type '<empty-response>"
  "phd/emptyResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'empty-response)))
  "Returns string type for a service object of type 'empty-response"
  "phd/emptyResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<empty-response>)))
  "Returns md5sum for a message object of type '<empty-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'empty-response)))
  "Returns md5sum for a message object of type 'empty-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<empty-response>)))
  "Returns full string definition for message of type '<empty-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'empty-response)))
  "Returns full string definition for message of type 'empty-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <empty-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <empty-response>))
  "Converts a ROS message object to a list"
  (cl:list 'empty-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'empty)))
  'empty-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'empty)))
  'empty-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'empty)))
  "Returns string type for a service object of type '<empty>"
  "phd/empty")