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
        self.servo_min = 285    # 最小パルス長
        self.servo_max = 405    # 最大パルス長

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

    def front_left_callback(self, msg):
        self.front_left_distance = msg.range * 100
        # rospy.loginfo(f"Front left distance: {msg.range * 100} cm")

    def front_center_callback(self, msg):
        self.front_center_distance = msg.range * 100
        # rospy.loginfo(f"Front center distance: {msg.range * 100} cm")

    def front_right_callback(self, msg):
        self.front_right_distance = msg.range * 100
        # rospy.loginfo(f"Front right distance: {msg.range * 100} cm")

    def left_callback(self, msg):
        self.left_distance = msg.range * 100
        # rospy.loginfo(f"Left side distance: {msg.range * 100} cm")

    def right_callback(self, msg):
        self.right_distance = msg.range * 100
        # rospy.loginfo(f"Right side distance: {msg.range * 100} cm")

    # def calculate_servo_angle(self):
    #     # 左右センサーの距離が20cm以上、前方センサーの距離が40cm以上離れているか確認
    #     side_safe_distance = 10  # 左右のしきい値
    #     front_safe_distance = 20  # 前方のしきい値
    #     if self.left_distance > side_safe_distance and self.right_distance > side_safe_distance and \
    #     self.front_left_distance > front_safe_distance and \
    #     self.front_center_distance > front_safe_distance and \
    #     self.front_right_distance > front_safe_distance:
    #         rospy.loginfo("全センサー安全距離範囲内。直進します。")
    #         return 90  # 直進

    #     # 5つのセンサーからの距離データを元にサーボの角度を決定
    #     min_distance = min(self.front_left_distance, self.front_center_distance,
    #                        self.front_right_distance, self.left_distance,
    #                        self.right_distance)

    #     # 最小距離に対応するサーボ角度を決定
    #     servo_angle = 90  # 初期値は直進
    #     if min_distance == self.front_left_distance:
    #         servo_angle = 150 if self.front_left_distance > 50 else 180
    #     elif min_distance == self.front_center_distance:
    #         servo_angle = 60 if self.front_center_distance > 50 else 120
    #     elif min_distance == self.front_right_distance:
    #         servo_angle = 30 if self.front_right_distance > 50 else 0
    #     elif min_distance == self.left_distance:
    #         servo_angle = 120
    #     elif min_distance == self.right_distance:
    #         servo_angle = 60

    #     rospy.loginfo(f"サーボ角度を {servo_angle} 度に設定します。")
    #     return servo_angle

    def calculate_servo_angle(self):
        # 左右センサーの距離が10cm以上、前方センサーの距離が20cm以上離れているか確認
        side_safe_distance = 10
        front_safe_distance = 20
        if self.left_distance > side_safe_distance and self.right_distance > side_safe_distance and \
        self.front_left_distance > front_safe_distance and \
        self.front_center_distance > front_safe_distance and \
        self.front_right_distance > front_safe_distance:
            rospy.loginfo("全センサー安全距離範囲内。直進します。")
            return 90  # 直進

        # 5つのセンサーからの最小距離を取得
        min_distance = min(self.front_left_distance, self.front_center_distance,
                        self.front_right_distance, self.left_distance,
                        self.right_distance)

        # 前方のセンサーから最も遠い場所を探す
        max_front_distance = max(self.front_left_distance, self.front_center_distance, self.front_right_distance)

        # 最小距離が短い場合、最も近い方向から避ける
        if min_distance < 30:
            if min_distance == self.front_left_distance:
                return 180  # 極端に右に
            elif min_distance == self.front_right_distance:
                return 0  # 極端に左に
            elif min_distance == self.front_center_distance:
                # 前方中央が最も近い場合、左右どちらが空いているか判断
                return 150 if self.left_distance > self.right_distance else 30
        else:
            # そうでない場合は、前方の最も遠い方向に向けて舵を取る
            if max_front_distance == self.front_left_distance:
                return 30  # わずかに左に
            elif max_front_distance == self.front_right_distance:
                return 150  # わずかに右に
            # 前方中央が最も遠い、または3つのセンサーが同じ距離の場合は直進

        return 90  # デフォルトは直進

    def update(self):
        if all([self.front_left_distance, self.front_center_distance, self.front_right_distance, self.left_distance, self.right_distance]):
            # 前方の3つのセンサーから最小の距離を取得
            min_front_distance = min(self.front_left_distance, self.front_center_distance, self.front_right_distance)
            # 右と左のセンサーから最小の距離を取得
            min_side_distance = min(self.left_distance, self.right_distance)

            # ログを出すためのコード
            if min_front_distance == self.front_left_distance:
                sensor_used = "前方左"
            elif min_front_distance == self.front_center_distance:
                sensor_used = "前方中央"
            else:
                sensor_used = "前方右"
            rospy.loginfo(f"\t{sensor_used}センサーの最小距離: {min_front_distance} cm。")

            # 前方のセンサーが15cm未満、または左右のセンサーが2cm未満の場合、停止
            if min_front_distance <= 10:
                rospy.loginfo(f"前方のセンサーが10cm未満 ({min_front_distance} cm) なので停止します。")
                self.current_esc_pulse = self.esc_neutral
            elif min_side_distance <= 3:
                rospy.loginfo(f"左右のセンサーのうち、最小距離が2cm未満 ({min_side_distance} cm) なので停止します。")
                self.current_esc_pulse = self.esc_neutral
            else:
                # 距離に基づいてサーボの制御
                if min_front_distance > 75:
                    self.current_esc_pulse = min(self.esc_neutral + 26, self.esc_max)
                elif min_front_distance > 50:
                    self.current_esc_pulse = min(self.esc_neutral + 26, self.esc_max)
                elif min_front_distance > 40:
                    self.current_esc_pulse = min(self.esc_neutral + 25, self.esc_max)
                elif min_front_distance > 30:
                    self.current_esc_pulse = min(self.esc_neutral + 24, self.esc_max)
                elif min_front_distance > 25:
                    self.current_esc_pulse = min(self.esc_neutral + 22, self.esc_max)
                elif min_front_distance > 20:
                    self.current_esc_pulse = min(self.esc_neutral + 20, self.esc_max)
                elif min_front_distance > 15:
                    self.current_esc_pulse = min(self.esc_neutral + 20, self.esc_max)
                else:
                    self.current_esc_pulse = min(self.esc_neutral, self.esc_max)

            self.pwm.set_pwm(self.esc_channel, 0, self.current_esc_pulse)


            # サーボの向きを調整
            servo_angle = self.calculate_servo_angle()

            # サーボモーターのパルス長を計算して設定
            pulse_length = self.servo_min + int((self.servo_max - self.servo_min) / 180 * servo_angle)
            self.pwm.set_pwm(self.servo_channel, 0, pulse_length)

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
