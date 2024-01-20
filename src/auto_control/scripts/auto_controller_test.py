#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Range
import Adafruit_PCA9685

class ServoTestController:
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
        self.sub_front_center = rospy.Subscriber('/ultrasonic/front_center', Range, self.front_center_callback)

        self.front_center_distance = None

    def front_center_callback(self, msg):
        self.front_center_distance = msg.range * 100

    def calculate_servo_angle(self):
        if self.front_center_distance is None:
            rospy.loginfo("前方中央センサーのデータがありません。サーボを中立位置に設定します。")
            return 90  # データがない場合は中立位置

        # 前方中央センサーのデータに基づくサーボ角度の設定
        if self.front_center_distance > 30:
            rospy.loginfo(f"距離: {self.front_center_distance} cm, サーボ角度: 0度")
            return 0 # 左
        elif self.front_center_distance > 25:
            rospy.loginfo(f"距離: {self.front_center_distance} cm, サーボ角度: 30度")
            return 30
        elif self.front_center_distance > 20:
            rospy.loginfo(f"距離: {self.front_center_distance} cm, サーボ角度: 60度")
            return 60
        elif self.front_center_distance > 15:
            rospy.loginfo(f"距離: {self.front_center_distance} cm, サーボ角度: 90度")
            return 90 # 真ん中
        elif self.front_center_distance > 10:
            rospy.loginfo(f"距離: {self.front_center_distance} cm, サーボ角度: 120度")
            return 120
        elif self.front_center_distance > 5:
            rospy.loginfo(f"距離: {self.front_center_distance} cm, サーボ角度: 150度")
            return 150
        else:
            rospy.loginfo(f"距離: {self.front_center_distance} cm, サーボ角度: 180度")
            return 180 # 右


    def update(self):
        servo_angle = self.calculate_servo_angle()
        rospy.loginfo(f"サーボ角度を {servo_angle} 度に設定します。")
        pulse_length = self.servo_min + int((self.servo_max - self.servo_min) / 180 * servo_angle)
        self.pwm.set_pwm(self.servo_channel, 0, pulse_length)

def main():
    rospy.init_node('servo_test_controller')
    controller = ServoTestController()
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        controller.update()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
