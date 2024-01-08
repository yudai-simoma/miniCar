
(cl:in-package :asdf)

(defsystem "traxxas_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "servo_esc_coeffs" :depends-on ("_package_servo_esc_coeffs"))
    (:file "_package_servo_esc_coeffs" :depends-on ("_package"))
  ))