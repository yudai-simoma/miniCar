#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Range
import Adafruit_PCA9685

class AutoController:
    def __init__(self):
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)

        self.esc_channel = 2
        self.esc_neutral = 307
        self.esc_max = 511
        self.esc_min = 102
        self.current_esc_pulse = self.esc_neutral

        # 超音波センサーのデータを受け取るためのサブスクライバー
        self.sub_front_left = rospy.Subscriber('/ultrasonic/front_left', Range, self.front_left_callback)
        self.sub_front_center = rospy.Subscriber('/ultrasonic/front_center', Range, self.front_center_callback)
        self.sub_front_right = rospy.Subscriber('/ultrasonic/front_right', Range, self.front_right_callback)
        self.sub_left = rospy.Subscriber('/ultrasonic/left', Range, self.left_callback)
        self.sub_right = rospy.Subscriber('/ultrasonic/right', Range, self.right_callback)

        self.front_left_distance = None
        self.front_center_distance = None
        self.front_right_distance = None
        self.left_distance = None
        self.right_distance = None

        # サーボモーター制御用のパブリッシャーの追加
        self.pub_servo_cmd = rospy.Publisher('/servo_cmd', Int16, queue_size=10)

    def front_left_callback(self, msg):
        self.front_left_distance = msg.range * 100
        rospy.loginfo(f"Front left distance: {msg.range} cm")

    def front_center_callback(self, msg):
        self.front_center_distance = msg.range * 100
        rospy.loginfo(f"Front center distance: {msg.range} cm")

    def front_right_callback(self, msg):
        self.front_right_distance = msg.range * 100
        rospy.loginfo(f"Front right distance: {msg.range} cm")

    def left_callback(self, msg):
        self.left_distance = msg.range * 100
        rospy.loginfo(f"Left side distance: {msg.range} cm")

    def right_callback(self, msg):
        self.right_distance = msg.range * 100
        rospy.loginfo(f"Right side distance: {msg.range} cm")

    def update(self):
        if all([self.front_left_distance, self.front_center_distance, self.front_right_distance, self.left_distance, self.right_distance]):
            # 距離に基づいてサーボの制御
            if self.front_center_distance < 30:  # 30cm未満の場合停止
                self.current_esc_pulse = self.esc_neutral
                rospy.loginfo("Stopping vehicle due to close obstacle")
            else:
                self.current_esc_pulse = min(self.current_esc_pulse + 10, self.esc_max)
                rospy.loginfo(f"Setting ESC pulse to {self.current_esc_pulse}")

            # サーボの向きを調整
            # サーボモーターの角度を計算
            if self.left_distance < self.right_distance:
                servo_angle = 125 + 20  # 右に曲がる
            elif self.left_distance > self.right_distance:
                servo_angle = 125 - 20  # 左に曲がる
            else:
                servo_angle = 125  # 直進

            self.pwm.set_pwm(self.esc_channel, 0, self.current_esc_pulse)
            # Arduinoにサーボの角度を送信
            # rospy.loginfo(f"servo_angle = {servo_angle}")
            self.pub_servo_cmd.publish(servo_angle)

def main():
    rospy.init_node('auto_controller')
    controller = AutoController()
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        controller.update()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
