#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Range
import Adafruit_PCA9685

class WallFollowerController:
    def __init__(self):
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(60)

        self.esc_channel = 2
        self.esc_neutral = 307
        self.esc_max = 511
        self.esc_min = 102
        self.current_esc_pulse = self.esc_neutral

        # サーボモーターの設定
        self.servo_channel = 8
        self.servo_min = 285
        self.servo_max = 405

        # センサーデータ受け取りのためのサブスクライバー
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

    def front_center_callback(self, msg):
        self.front_center_distance = msg.range * 100

    def front_right_callback(self, msg):
        self.front_right_distance = msg.range * 100

    def left_callback(self, msg):
        self.left_distance = msg.range * 100

    def right_callback(self, msg):
        self.right_distance = msg.range * 100

    def calculate_servo_angle(self):
        desired_distance = 15  # 壁からの望ましい距離
        front_safe_distance = 20  # 前方の安全距離

        # 左の壁に対する制御
        if self.left_distance > desired_distance:
            servo_angle = 60  # 左に曲がる
        elif self.left_distance < desired_distance:
            servo_angle = 120  # 右に曲がる
        else:
            servo_angle = 90  # 直進

        # 前方とその他の方向に対する制御
        if self.front_center_distance < front_safe_distance or \
        self.front_left_distance < front_safe_distance or \
        self.front_right_distance < front_safe_distance:
            servo_angle = 140  # 前方障害物があれば右に回避

        if self.right_distance < desired_distance:
            servo_angle = 40  # 右側に障害物があれば左に回避

        return servo_angle

    def update(self):
        if all([self.front_left_distance, self.front_center_distance, self.front_right_distance, self.left_distance, self.right_distance]):
            # 前方の3つのセンサーから最小の距離を取得
            min_front_distance = min(self.front_left_distance, self.front_center_distance, self.front_right_distance)
            # ログを出すためのコード
            if min_front_distance == self.front_left_distance:
                sensor_used = "前方左"
            elif min_front_distance == self.front_center_distance:
                sensor_used = "前方中央"
            else:
                sensor_used = "前方右"
            rospy.loginfo(f"\t{sensor_used}センサーの最小距離: {min_front_distance} cm。")

            # 距離に基づいてサーボの制御
            if min_front_distance > 100:
                self.current_esc_pulse = min(self.esc_neutral + 28, self.esc_max)
            elif min_front_distance > 60:
                self.current_esc_pulse = min(self.esc_neutral + 28, self.esc_max)
            elif min_front_distance > 50:
                self.current_esc_pulse = min(self.esc_neutral + 28, self.esc_max)
            elif min_front_distance > 40:
                self.current_esc_pulse = min(self.esc_neutral + 26, self.esc_max)
            elif min_front_distance > 30:
                self.current_esc_pulse = min(self.esc_neutral + 26, self.esc_max)
            elif min_front_distance > 20:
                self.current_esc_pulse = min(self.esc_neutral + 23, self.esc_max)
            elif min_front_distance > 15:
                self.current_esc_pulse = min(self.esc_neutral + 23, self.esc_max)
            else:
                # 距離が3cm未満の場合、停止
                self.current_esc_pulse = self.esc_neutral
            self.pwm.set_pwm(self.esc_channel, 0, self.current_esc_pulse)

            servo_angle = self.calculate_servo_angle()
            rospy.loginfo(f"サーボ角度を {servo_angle} 度に設定します。")

            pulse_length = self.servo_min + int((self.servo_max - self.servo_min) / 180 * servo_angle)
            self.pwm.set_pwm(self.servo_channel, 0, pulse_length)

def main():
    rospy.init_node('wall_follower_controller')
    controller = WallFollowerController()
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        controller.update()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
