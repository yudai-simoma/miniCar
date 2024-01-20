#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Range
import Adafruit_PCA9685

class MultiSensorServoController:
    def __init__(self):
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)

        self.servo_channel = 8
        self.servo_min = 285
        self.servo_max = 405

        # センサーデータ受け取りのためのサブスクライバー
        self.sub_front_left = rospy.Subscriber('/ultrasonic/front_left', Range, self.front_left_callback)
        self.sub_front_center = rospy.Subscriber('/ultrasonic/front_center', Range, self.front_center_callback)
        self.sub_front_right = rospy.Subscriber('/ultrasonic/front_right', Range, self.front_right_callback)
        self.sub_left = rospy.Subscriber('/ultrasonic/left', Range, self.left_callback)
        self.sub_right = rospy.Subscriber('/ultrasonic/right', Range, self.right_callback)

        # 各センサーからの距離データ
        self.front_left_distance = None
        self.front_center_distance = None
        self.front_right_distance = None
        self.left_distance = None
        self.right_distance = None

    def front_left_callback(self, msg):
        self.front_left_distance = msg.range * 100
        rospy.loginfo(f"Front left distance: {self.front_left_distance} cm")

    def front_center_callback(self, msg):
        self.front_center_distance = msg.range * 100
        rospy.loginfo(f"Front center distance: {self.front_center_distance} cm")

    def front_right_callback(self, msg):
        self.front_right_distance = msg.range * 100
        rospy.loginfo(f"Front right distance: {self.front_right_distance} cm")

    def left_callback(self, msg):
        self.left_distance = msg.range * 100
        rospy.loginfo(f"Left side distance: {self.left_distance} cm")

    def right_callback(self, msg):
        self.right_distance = msg.range * 100
        rospy.loginfo(f"Right side distance: {self.right_distance} cm")

    def calculate_servo_angle(self):
        # ここにセンサーデータに基づくサーボモータの角度計算ロジックを実装します。
        # 例えば、最も近い障害物に応じて角度を調整するなど。
        # 現在は仮の値として直進(90度)を返しています。
        return 90

    def update(self):
        servo_angle = self.calculate_servo_angle()
        rospy.loginfo(f"サーボ角度を {servo_angle} 度に設定します。")
        pulse_length = self.servo_min + int((self.servo_max - self.servo_min) / 180 * servo_angle)
        self.pwm.set_pwm(self.servo_channel, 0, pulse_length)

def main():
    rospy.init_node('multi_sensor_servo_controller')
    controller = MultiSensorServoController()
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        controller.update()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
