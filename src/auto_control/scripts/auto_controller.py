#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Range
import Adafruit_PCA9685
import atexit

# 両壁の中央を走るプログラム
class AutoController:

    UPDATE_RATE = 25  # 更新頻度（1秒間に何回処理を行うか）

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
        self.sub_left_far = rospy.Subscriber('/ultrasonic/left_far', Range, self.left_far_callback)
        self.sub_center = rospy.Subscriber('/ultrasonic/center', Range, self.center_callback)
        self.sub_right_far = rospy.Subscriber('/ultrasonic/right_far', Range, self.right_far_callback)
        self.sub_left_near = rospy.Subscriber('/ultrasonic/left_near', Range, self.left_near_callback)
        self.sub_right_near = rospy.Subscriber('/ultrasonic/right_near', Range, self.right_near_callback)

        self.left_far_distance = None
        self.center_distance = None
        self.right_far_distance = None
        self.left_near_distance = None
        self.right_near_distance = None

        # PID parameters
        # self.Kp = 2.2       # P: 目的の値に差分があったときにどの比率で舵を切るかの比率
        # self.Ki = 0.01     # I: Pだけだと誤差が出るため、誤差を直す時に証する値
        # self.Kd = 0.5      # D: PやIだけだと、目的地を通り過ぎてしまい舵切りがカクカクするのを防ぐ値
        self.Kp = 1.2      # P: 目的の値に差分があったときにどの比率で舵を切るかの比率
        self.Ki = 0.001     # I: Pだけだと誤差が出るため、誤差を直す時に証する値
        self.Kd = 0.2      # D: PやIだけだと、目的地を通り過ぎてしまい舵切りがカクカクするのを防ぐ値

        self.err_total = 0
        self.err_prev = 0
        self.err = 0

        self.log_counter = 0  # ログ出力用のカウンター

    def left_far_callback(self, msg):
        self.left_far_distance = msg.range * 100
        # if self.log_counter % self.UPDATE_RATE == 0:  # 1秒ごとにログを出力
        #     rospy.loginfo(f"\t\t\t左外側センサー距離: {self.left_far_distance} cm")

    def center_callback(self, msg):
        self.center_distance = msg.range * 100
        # if self.log_counter % self.UPDATE_RATE == 0:  # 1秒ごとにログを出力
        #     rospy.loginfo(f"\t\t中央センサー距離: {self.center_distance} cm")

    def right_far_callback(self, msg):
        self.right_far_distance = msg.range * 100
        # if self.log_counter % self.UPDATE_RATE == 0:  # 1秒ごとにログを出力
        #     rospy.loginfo(f"\t右外側センサー距離: {self.right_far_distance} cm")

    def left_near_callback(self, msg):
        self.left_near_distance = msg.range * 100
        # if self.log_counter % self.UPDATE_RATE == 0:  # 1秒ごとにログを出力
        #     rospy.loginfo(f"\t\t\t\t左内側センサー距離: {self.left_near_distance} cm")

    def right_near_callback(self, msg):
        self.right_near_distance = msg.range * 100
        # if self.log_counter % self.UPDATE_RATE == 0:  # 1秒ごとにログを出力
        #     rospy.loginfo(f"右内側センサー距離: {self.right_near_distance} cm")

    def stop_motors(self):
        rospy.loginfo("Stopping motors...")
        # ESCとサーボモータを中立位置に設定
        self.pwm.set_pwm(self.esc_channel, 0, int(self.esc_neutral))
        self.pwm.set_pwm(self.servo_channel, 0, int(self.servo_medium))

    # モータの制御を行う関数
    def control_motor(self):
        # 前方の3つのセンサーから最小の距離を取得
        min_front_distance = min(self.left_far_distance, self.center_distance, self.right_far_distance)

        if min_front_distance > 1:
            self.current_esc_pulse = min(self.esc_neutral + 26, self.esc_max)
        else:
            self.current_esc_pulse = min(self.esc_neutral, self.esc_max)

        self.pwm.set_pwm(self.esc_channel, 0, self.current_esc_pulse)

    # サーボモータの制御を行う関数
    def control_servo(self):
        self.err_prev = self.err
        # 左右センサーの値を使用して誤差を計算
        left_average = (self.left_near_distance + self.left_far_distance) / 2
        right_average = (self.right_near_distance + self.right_far_distance) / 2
        self.err = right_average - left_average
        self.err_total += self.err
        P = self.err * self.Kp
        I = self.err_total * self.Ki
        D = (self.err_prev - self.err) * self.Kd

        # if self.center_distance < 40:
        #     if self.center_distance < 20:
        #         rospy.loginfo("----->>>>")
        #         P += 50
        #     elif self.left_far_distance < self.right_far_distance:
        #         rospy.loginfo("----->>>>")
        #         P += 50
        #     else:
        #         rospy.loginfo("<<<<-----")
        #         P -= 50
        if self.left_far_distance < 20:
            P += 50  # ステアリングを大きく右に切る
            # rospy.loginfo("左外側前方センサーが近いため、右にステアリングを切ります。")
        elif self.left_near_distance < 20:
            P += 50  # ステアリングを大きく右に切る
            # rospy.loginfo("左内側センサーが近いため、右にステアリングを切ります。")
        elif self.right_far_distance < 20:
            P -= 50  # ステアリングを大きく左に切る
            # rospy.loginfo("右外側前方センサーが近いため、左にステアリングを切ります。")
        elif self.right_near_distance < 20:
            P -= 50  # ステアリングを大きく左に切る
            # rospy.loginfo("右内側センサーが近いため、左にステアリングを切ります。")
        # else:
        #     rospy.loginfo("！！！安全！！！")

        steer_pwm = int(P + I + D)

        steer_pwm = max(min(self.servo_medium + steer_pwm, self.servo_max), self.servo_min)

        # rospy.loginfo(f"servo_pwm = '{int(steer_pwm)}'\n")
        print(f'中央.{self.center_distance:.1f} 外右.{self.right_far_distance:.1f} 内右.{self.right_near_distance:.1f} 外左.{self.left_far_distance:.1f} 内左.{self.left_near_distance:.1f} pwm:{steer_pwm}')
        self.pwm.set_pwm(self.servo_channel, 0, int(steer_pwm))

    def update(self):
        if all([self.left_far_distance, self.center_distance, self.right_far_distance, self.left_near_distance, self.right_near_distance]):

            # モータの制御を行う関数を呼び出す
            self.control_motor()

            # サーボモータの制御を行う関数を呼び出す
            self.control_servo()

            self.log_counter += 1  # カウンターをインクリメント
            if self.log_counter >= self.UPDATE_RATE:
                self.log_counter = 0  # カウンターをリセット

def main():
    rospy.init_node('auto_controller')
    controller = AutoController()
    rate = rospy.Rate(controller.UPDATE_RATE)
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
