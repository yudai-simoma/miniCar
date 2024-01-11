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

        # サーボモーターの設定
        self.servo_channel = 8  # サーボモーターのチャンネル
        self.servo_min = 150    # 最小パルス長
        self.servo_max = 600    # 最大パルス長

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
        # rospy.loginfo(f"Front left distance: {msg.range * 100} cm")

    def front_center_callback(self, msg):
        self.front_center_distance = msg.range * 100
        rospy.loginfo(f"Front center distance: {msg.range * 100} cm")

    def front_right_callback(self, msg):
        self.front_right_distance = msg.range * 100
        # rospy.loginfo(f"Front right distance: {msg.range * 100} cm")

    def left_callback(self, msg):
        self.left_distance = msg.range * 100
        # rospy.loginfo(f"Left side distance: {msg.range * 100} cm")

    def right_callback(self, msg):
        self.right_distance = msg.range * 100
        # rospy.loginfo(f"Right side distance: {msg.range * 100} cm")

    def update(self):
        if all([self.front_left_distance, self.front_center_distance, self.front_right_distance, self.left_distance, self.right_distance]):
            # # 距離に基づいてサーボの制御
            # if self.front_center_distance < 30:  # 30cm未満の場合停止
            #     self.current_esc_pulse = self.esc_neutral
            #     rospy.loginfo("Stopping vehicle due to close obstacle")
            # else:
            #     self.current_esc_pulse = min(self.current_esc_pulse + 10, self.esc_max)
            #     rospy.loginfo(f"Setting ESC pulse to {self.current_esc_pulse}")

            if self.front_center_distance > 100:
                # 距離が40cm以上の場合、スロットルを最も高く設定
                self.current_esc_pulse = min(self.esc_neutral + 100, self.esc_max)
                self.current_esc_pulse = min(self.esc_neutral + 80, self.esc_max)
            elif self.front_center_distance > 60:
                # 距離が30cm以上40cm未満の場合、スロットルを少し低く設定
#                self.current_esc_pulse = min(self.esc_neutral + 105, self.esc_max)
                self.current_esc_pulse = min(self.esc_neutral + 80, self.esc_max)
            elif self.front_center_distance > 50:
                # 距離が30cm以上40cm未満の場合、スロットルを少し低く設定
#                self.current_esc_pulse = min(self.esc_neutral + 95, self.esc_max)
                self.current_esc_pulse = min(self.esc_neutral + 80, self.esc_max)
            elif self.front_center_distance > 40:
                # 距離が15cm以上30cm未満の場合、スロットルをさらに低く設定
#                self.current_esc_pulse = min(self.esc_neutral + 90, self.esc_max)
                self.current_esc_pulse = min(self.esc_neutral + 75, self.esc_max)
            elif self.front_center_distance > 30:
                # 距離が6cm以上15cm未満の場合、スロットルを非常に低く設定
#                self.current_esc_pulse = min(self.esc_neutral + 80, self.esc_max)
                self.current_esc_pulse = min(self.esc_neutral + 75, self.esc_max)
            elif self.front_center_distance > 20:
                # 距離が3cm以上6cm未満の場合、最低のスロットルを設定
                self.current_esc_pulse = min(self.esc_neutral + 70, self.esc_max)
            elif self.front_center_distance > 15:
                # 距離が3cm以上6cm未満の場合、最低のスロットルを設定
                self.current_esc_pulse = min(self.esc_neutral + 70, self.esc_max)
            else:
                # 距離が3cm未満の場合、停止
                self.current_esc_pulse = self.esc_neutral
            self.pwm.set_pwm(self.esc_channel, 0, self.current_esc_pulse)

            # サーボの向きを調整
            distance_diff = self.left_distance - self.right_distance  # 左右の距離差

            if distance_diff > 20:  # 左の方が右より20cm以上遠い場合
                servo_angle = 125 + 40  # 最も左に曲がる
            elif distance_diff > 10:  # 左の方が右より10cm以上遠い場合
                servo_angle = 125 + 20  # 左に曲がる
            elif distance_diff < -20:  # 右の方が左より20cm以上遠い場合
                servo_angle = 125 - 40  # 最も右に曲がる
            elif distance_diff < -10:  # 右の方が左より10cm以上遠い場合
                servo_angle = 125 - 20  # 右に曲がる
            else:
                servo_angle = 125  # 直進

            # サーボモーターのパルス長を計算して設定
            pulse_length = self.servo_min + int((self.servo_max - self.servo_min) / 180 * servo_angle)
            self.pwm.set_pwm(self.servo_channel, 0, pulse_length)


            # アルディノボードでの制御で使用
            # # サーボモーターの角度を計算
            # if self.left_distance < self.right_distance:
            #     servo_angle = 125 + 20  # 右に曲がる
            # elif self.left_distance > self.right_distance:
            #     servo_angle = 125 - 20  # 左に曲がる
            # else:
            #     servo_angle = 125  # 直進

            # # Arduinoにサーボの角度を送信
            # # rospy.loginfo(f"servo_angle = {servo_angle}")
            # self.pub_servo_cmd.publish(servo_angle)

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
