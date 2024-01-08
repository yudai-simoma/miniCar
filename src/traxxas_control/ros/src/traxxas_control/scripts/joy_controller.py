#!/usr/bin/env python3

# 必要なライブラリのインポート
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from traxxas_control.msg import servo_esc_coeffs
import Adafruit_PCA9685  # Adafruit PCA9685ライブラリのインポート

# ジョイスティックコントローラーのクラス定義
class JoyController:
    # 初期化メソッド
    def __init__(self):
        # ROSのパブリッシャーの定義
        self.pub_servo_cmd = rospy.Publisher('/servo_cmd', Int16, queue_size=10)
        self.pub_esc = rospy.Publisher('/esc', Int16, queue_size=10)
        self.pub_servo_esc_coeffs = rospy.Publisher('/servo_esc_coeffs', servo_esc_coeffs, queue_size=10)

        # PCA9685の初期化
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)  # PWM周波数を60Hzに設定

        # ESCのチャンネル設定
        self.esc_channel = 2    # ESCのチャンネル

        # ESCモーターのパルス長設定を調整
        self.esc_neutral = 307  # 1.5ms（ニュートラル位置）
        self.esc_min = 102      # 1.0ms（最小スロットル位置）より広い範囲に設定
        self.esc_max = 511      # 2.0ms（最大スロットル位置）より広い範囲に設定
        # スロットルの倍数設定を増加
        self.throt_mult = 400  # 以前は200だった
        self.current_esc_pulse = self.esc_neutral

        # ROSのサブスクライバーの定義
        rospy.Subscriber('/joy', Joy, self._joy_cb, queue_size=10)

        # ROSノードの初期化
        rospy.init_node('joy_controller')

    # コールバックメソッド
    def _joy_cb(self, msg):
        rospy.loginfo("Joy message received: axes: %s, buttons: %s", str(msg.axes), str(msg.buttons))

        # ジョイスティックの右スティック縦軸でサーボモータの位置を制御する
        right_stick_vertical = msg.axes[3]  # 右スティック縦軸の値を取得
        # servo_position = int(90 + (right_stick_vertical * 90))  # 0度から180度の範囲で計算
        servo_position = int(125 + (right_stick_vertical * 55))

        if msg.axes[3]:
            rospy.loginfo(f"Right Stick Horizontal Value: {right_stick_vertical}")
            rospy.loginfo(f"Calculated Servo Position: {servo_position}")
        self.pub_servo_cmd.publish(servo_position)  # サーボモータの位置をパブリッシュ

        # 左スティックの縦方向の入力を取得
        if msg.buttons[0]:  # A button
            self.current_esc_pulse = min(self.current_esc_pulse + 10, self.esc_max)
        elif msg.buttons[1]:  # B button
            self.current_esc_pulse = max(self.current_esc_pulse - 10, self.esc_min)
        self.pwm.set_pwm(self.esc_channel, 0, self.current_esc_pulse)

        # ESCの現在の状態をパブリッシュ
        msg = servo_esc_coeffs()
        msg.esc_neutral = self.esc_neutral
        msg.throt_mult = self.throt_mult
        self.pub_servo_esc_coeffs.publish(msg)

# メイン関数
if __name__ == '__main__':
    controller = JoyController()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr('JoyController failed')
