
(in-package :asdf)

(defsystem "sr_display-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "display_check" :depends-on ("_package"))
    (:file "_package_display_check" :depends-on ("_package"))
    ))
