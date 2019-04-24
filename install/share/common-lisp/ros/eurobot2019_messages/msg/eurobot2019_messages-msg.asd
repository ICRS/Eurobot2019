
(cl:in-package :asdf)

(defsystem "eurobot2019_messages-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "drop_command" :depends-on ("_package_drop_command"))
    (:file "_package_drop_command" :depends-on ("_package"))
    (:file "drop_motors" :depends-on ("_package_drop_motors"))
    (:file "_package_drop_motors" :depends-on ("_package"))
    (:file "grabber_motors" :depends-on ("_package_grabber_motors"))
    (:file "_package_grabber_motors" :depends-on ("_package"))
    (:file "pickup" :depends-on ("_package_pickup"))
    (:file "_package_pickup" :depends-on ("_package"))
  ))