
(in-package :asdf)

(defsystem "sr_edc_ethercat_drivers-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "SimpleMotorFlasher" :depends-on ("_package"))
    (:file "_package_SimpleMotorFlasher" :depends-on ("_package"))
    ))
