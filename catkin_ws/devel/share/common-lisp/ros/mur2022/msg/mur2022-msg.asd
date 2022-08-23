
(cl:in-package :asdf)

(defsystem "mur2022-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "place_holder_msg" :depends-on ("_package_place_holder_msg"))
    (:file "_package_place_holder_msg" :depends-on ("_package"))
  ))