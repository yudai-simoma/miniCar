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

        # PID parameters
        self.Kp = 0.4       # P: 目的の値に差分があったときにどの比率で舵を切るかの比率
        self.Ki = 0.001     # I: Pだけだと誤差が出るため、誤差を直す時に証する値
        self.Kd = 0.01      # D: PやIだけだと、目的地を通り過ぎてしまい舵切りがカクカクするのを防ぐ値
        self.err_total = 0
        self.err_prev = 0

    def front_left_callback(self, msg):
        self.front_left_distance = msg.range * 100

    def front_center_callback(self, msg):
        self.front_center_distance = msg.range * 100

    def front_right_callback(self, msg):
        self.front_right_distance = msg.range * 100

    def left_callback(self, msg):
        self.left_distance = msg.range * 100

    def right_callback(self, msg):
        self.right_distance = msg.range * 100

    # モータの制御を行う関数
    def control_motor(self):
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
            # if min_front_distance > 75:
            #     self.current_esc_pulse = min(self.esc_neutral + 26, self.esc_max)
            # elif min_front_distance > 50:
            #     self.current_esc_pulse = min(self.esc_neutral + 26, self.esc_max)
            # elif min_front_distance > 40:
            #     self.current_esc_pulse = min(self.esc_neutral + 25, self.esc_max)
            # elif min_front_distance > 30:
            #     self.current_esc_pulse = min(self.esc_neutral + 24, self.esc_max)
            # elif min_front_distance > 25:
            #     self.current_esc_pulse = min(self.esc_neutral + 22, self.esc_max)
            if min_front_distance > 20:
                self.current_esc_pulse = min(self.esc_neutral + 23, self.esc_max)
            elif min_front_distance > 10:
                self.current_esc_pulse = min(self.esc_neutral + 23, self.esc_max)
            else:
                self.current_esc_pulse = min(self.esc_neutral, self.esc_max)

        self.pwm.set_pwm(self.esc_channel, 0, self.current_esc_pulse)

    # サーボモータの制御を行う関数
    def control_servo(self):
        err = self.front_right_distance - self.front_left_distance
        self.err_total += err
        P = err * self.Kp
        I = self.err_total * self.Ki
        D = (err - self.err_prev) * self.Kd
        self.err_prev = err

        steer_pwm = self.servo_min + int((self.servo_max - self.servo_min) / 180 * (90 + P + I + D))
        steer_pwm = max(min(steer_pwm, self.servo_max), self.servo_min)

        self.pwm.set_pwm(self.servo_channel, 0, steer_pwm)

    def update(self):
        if all([self.front_left_distance, self.front_center_distance, self.front_right_distance, self.left_distance, self.right_distance]):

            # ータの制御を行う関数を呼び出す
            self.control_motor()

            # サーボモータの制御を行う関数を呼び出す
            self.control_servo()

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
