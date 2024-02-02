
(cl:in-package :asdf)

(defsystem "glider_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "altimsg" :depends-on ("_package_altimsg"))
    (:file "_package_altimsg" :depends-on ("_package"))
    (:file "dvlmsg" :depends-on ("_package_dvlmsg"))
    (:file "_package_dvlmsg" :depends-on ("_package"))
    (:file "imumsg" :depends-on ("_package_imumsg"))
    (:file "_package_imumsg" :depends-on ("_package"))
    (:file "miniCTmsg" :depends-on ("_package_miniCTmsg"))
    (:file "_package_miniCTmsg" :depends-on ("_package"))
  ))