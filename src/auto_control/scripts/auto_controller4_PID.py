#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Range
import Adafruit_PCA9685
import atexit

# 両壁の中央を走るプログラム
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
        self.servo_max = 410    # 最大パルス長
        self.servo_medium = ((self.servo_max - self.servo_min) / 2) + self.servo_min    # 中央パルス長

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
        self.Kp = 0.0       # P: 目的の値に差分があったときにどの比率で舵を切るかの比率
        self.Ki = 0.00     # I: Pだけだと誤差が出るため、誤差を直す時に証する値
        self.Kd = 0.0      # D: PやIだけだと、目的地を通り過ぎてしまい舵切りがカクカクするのを防ぐ値
        # self.Kp = 3.0       # P: 目的の値に差分があったときにどの比率で舵を切るかの比率
        # self.Ki = 0.000     # I: Pだけだと誤差が出るため、誤差を直す時に証する値
        # self.Kd = 0.00      # D: PやIだけだと、目的地を通り過ぎてしまい舵切りがカクカクするのを防ぐ値

        self.err_total = 0
        self.err_prev = 0
        self.err = 0

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

    def stop_motors(self):
        rospy.loginfo("Stopping motors...")
        # ESCとサーボモータを中立位置に設定
        self.pwm.set_pwm(self.esc_channel, 0, int(self.esc_neutral))
        self.pwm.set_pwm(self.servo_channel, 0, int(self.servo_medium))

    # モータの制御を行う関数
    def control_motor(self):
        # 前方の3つのセンサーから最小の距離を取得
        min_front_distance = min(self.front_left_distance, self.front_center_distance, self.front_right_distance)
        # 右と左のセンサーから最小の距離を取得
        min_side_distance = min(self.left_distance, self.right_distance)

        # 前方のセンサーが15cm未満、または左右のセンサーが2cm未満の場合、停止
        if min_front_distance < 5:
            rospy.loginfo(f"前方のセンサーが5cm未満 ({min_front_distance} cm) なので後進します。")
            self.current_esc_pulse = max(self.esc_neutral - 30, self.esc_min)  # 後進のパルス幅を設定
        elif min_side_distance <= 3:
            rospy.loginfo(f"左右のセンサーのうち、最小距離が3cm未満 ({min_side_distance} cm) なので停止します。")
            self.current_esc_pulse = self.esc_neutral
        else:
            # 距離に基づいてサーボの制御
            if min_front_distance > 20:
                self.current_esc_pulse = min(self.esc_neutral + 23, self.esc_max)
            elif min_front_distance > 5:
                self.current_esc_pulse = min(self.esc_neutral + 22, self.esc_max)
            else:
                self.current_esc_pulse = min(self.esc_neutral, self.esc_max)

        self.pwm.set_pwm(self.esc_channel, 0, self.current_esc_pulse)

    # サーボモータの制御を行う関数
    def control_servo(self):
        self.err_prev = self.err
        self.err = self.front_right_distance - self.front_left_distance
        self.err_total += self.err
        P = self.err * self.Kp
        I = self.err_total * self.Ki
        D = (self.err_prev - self.err) * self.Kd

        rospy.loginfo(f"Front_right_distance = {self.front_right_distance}, Front_left_distance = {self.front_left_distance}")
        rospy.loginfo(f"P = {P}, I = {I}, D = {D}, all = {P + I + D}")

        # 左側の壁からの距離に基づいてステアリングを調整する
        # ここでは、左側の距離が特定の閾値より小さい場合、右にステアリングを切る
        left_wall_threshold = 30  # 15cm以下の場合、左の壁に近づいていると判断
        right_wall_threshold = 60
        if self.front_left_distance < left_wall_threshold or \
            self.left_distance < left_wall_threshold:
            rospy.loginfo(f"左の壁に近づいているため、右にステアリングを切ります。")
            P += 40  # この値は実際の挙動を見て調整する
        elif self.front_right_distance < right_wall_threshold or \
            self.right_distance < right_wall_threshold:
            rospy.loginfo(f"右の壁に近づいているため、左にステアリングを切ります。")
            P -= 40  # この値は実際の挙動を見て調整する

		# TODO:おそらく不要
        # steer_pwm = self.servo_min + int((self.servo_max - self.servo_min) / 180 * (90 + P + I + D))
        # steer_pwm = self.servo_min + int(90 + P + I + D)
        steer_pwm = int(P + I + D)

        steer_pwm = max(min(self.servo_medium + steer_pwm, self.servo_max), self.servo_min)

        rospy.loginfo(f"servo_pwm = '{int(steer_pwm)}'\n")
        self.pwm.set_pwm(self.servo_channel, 0, int(steer_pwm))

    def update(self):
        if all([self.front_left_distance, self.front_center_distance, self.front_right_distance, self.left_distance, self.right_distance]):

            # ータの制御を行う関数を呼び出す
            self.control_motor()

            # サーボモータの制御を行う関数を呼び出す
            self.control_servo()

def main():
    rospy.init_node('auto_controller')
    controller = AutoController()
    rate = rospy.Rate(30)
    # プログラム終了時にモータ停止関数を呼び出す
    atexit.register(controller.stop_motors)
    while not rospy.is_shutdown():
        controller.update()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
