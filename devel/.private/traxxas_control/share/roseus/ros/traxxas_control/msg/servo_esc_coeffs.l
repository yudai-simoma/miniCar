;; Auto-generated. Do not edit!


(when (boundp 'traxxas_control::servo_esc_coeffs)
  (if (not (find-package "TRAXXAS_CONTROL"))
    (make-package "TRAXXAS_CONTROL"))
  (shadow 'servo_esc_coeffs (find-package "TRAXXAS_CONTROL")))
(unless (find-package "TRAXXAS_CONTROL::SERVO_ESC_COEFFS")
  (make-package "TRAXXAS_CONTROL::SERVO_ESC_COEFFS"))

(in-package "ROS")
;;//! \htmlinclude servo_esc_coeffs.msg.html


(defclass traxxas_control::servo_esc_coeffs
  :super ros::object
  :slots (_servo_neutral _esc_neutral _angle_mult _throt_mult ))

(defmethod traxxas_control::servo_esc_coeffs
  (:init
   (&key
    ((:servo_neutral __servo_neutral) 0)
    ((:esc_neutral __esc_neutral) 0)
    ((:angle_mult __angle_mult) 0)
    ((:throt_mult __throt_mult) 0)
    )
   (send-super :init)
   (setq _servo_neutral (round __servo_neutral))
   (setq _esc_neutral (round __esc_neutral))
   (setq _angle_mult (round __angle_mult))
   (setq _throt_mult (round __throt_mult))
   self)
  (:servo_neutral
   (&optional __servo_neutral)
   (if __servo_neutral (setq _servo_neutral __servo_neutral)) _servo_neutral)
  (:esc_neutral
   (&optional __esc_neutral)
   (if __esc_neutral (setq _esc_neutral __esc_neutral)) _esc_neutral)
  (:angle_mult
   (&optional __angle_mult)
   (if __angle_mult (setq _angle_mult __angle_mult)) _angle_mult)
  (:throt_mult
   (&optional __throt_mult)
   (if __throt_mult (setq _throt_mult __throt_mult)) _throt_mult)
  (:serialization-length
   ()
   (+
    ;; int32 _servo_neutral
    4
    ;; int32 _esc_neutral
    4
    ;; int32 _angle_mult
    4
    ;; int32 _throt_mult
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _servo_neutral
       (write-long _servo_neutral s)
     ;; int32 _esc_neutral
       (write-long _esc_neutral s)
     ;; int32 _angle_mult
       (write-long _angle_mult s)
     ;; int32 _throt_mult
       (write-long _throt_mult s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _servo_neutral
     (setq _servo_neutral (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _esc_neutral
     (setq _esc_neutral (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _angle_mult
     (setq _angle_mult (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _throt_mult
     (setq _throt_mult (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get traxxas_control::servo_esc_coeffs :md5sum-) "2d202238d5f284960838988ffb4a6570")
(setf (get traxxas_control::servo_esc_coeffs :datatype-) "traxxas_control/servo_esc_coeffs")
(setf (get traxxas_control::servo_esc_coeffs :definition-)
      "int32 servo_neutral
int32 esc_neutral

int32 angle_mult
int32 throt_mult

")



(provide :traxxas_control/servo_esc_coeffs "2d202238d5f284960838988ffb4a6570")


