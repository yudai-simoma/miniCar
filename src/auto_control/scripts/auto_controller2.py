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
        left_distance_desired = 35  # 左壁からの望ましい距離
        left_front_distance_safe = 45  # 左前方の安全距離
        right_distance_desired = 75  # 右壁からの望ましい距離
        right_front_distance_safe = 80  # 右前方の安全距離
        front_distance_stop = 4  # 前方の停止距離
        right_distance_stop = 2   # 右側の停止距離
        front_center_distance_safe = 10  # 前方中央の安全最小距離
        front_center_distance_max = 70  # 前方中央の安全最大距離
        left_front_distance_max = 60  # 左前方の安全最大距離

        # 左センサーの値が取れない場合、最大左に舵を切る
        # if self.left_distance is None:
        #   return 0  # 最大左に舵を切る
    
        # 前方センサーによる停止判定
        # if self.front_center_distance < front_distance_stop or \
        if self.front_left_distance < front_distance_stop or \
        self.front_right_distance < front_distance_stop:
            rospy.loginfo("前方に障害物が検出されたため、停止します。")
            return None  # 前方障害物があれば停止

        # 右側センサーによる停止判定
        if self.right_distance < right_distance_stop:
            rospy.loginfo("右側に障害物が検出されたため、停止します。")
            return None  # 右側に障害物があれば停止

        # 特定の条件下で最大右に舵を切る
        if left_front_distance_safe <= self.front_left_distance <= left_front_distance_max and \
        front_center_distance_safe <= self.front_center_distance <= front_center_distance_max:
            rospy.loginfo("左前方および前方中央のセンサーが安全な距離を示しているため、最大限右に舵を切ります。")
            return 175  # 最大右に舵を切る

        # 左壁と左前方の距離を比較し、より近い距離を使用
        closer_left_distance = min(self.left_distance, self.front_left_distance)

        # 左壁との距離に基づく制御
        if closer_left_distance == self.left_distance and \
        self.left_distance > left_distance_desired:
            rospy.loginfo("左壁からの距離が安全範囲内にあるため、左に舵を切ります。")
            return 45  # 左に曲がる
        elif self.left_distance < left_distance_desired:
            # 右側のセンサーの値を考慮して右か左かを決定
            if self.right_distance > right_distance_desired:
                rospy.loginfo("右側が安全であるため、右に舵を切ります。")
                return 180  # 安全なら右に曲がる
            else:
                rospy.loginfo("右側が安全でないため、左に舵を切ります。")
                return 0  # 右側が安全でなければ左に曲がる

        # 左前方センサーとの距離に基づく制御
        if closer_left_distance == self.front_left_distance and \
        self.front_left_distance > left_front_distance_safe:
            rospy.loginfo("左前方からの距離が安全範囲内にあるため、左に舵を切ります。")
            return 45  # 左に曲がる
        elif self.front_left_distance < left_front_distance_safe:
            # 右側のセンサーの値を考慮して右か左かを決定
            if self.front_right_distance > right_front_distance_safe:
                rospy.loginfo("右前方が安全であるため、右に舵を切ります。")
                return 179  # 安全なら右に曲がる
            else:
                rospy.loginfo("右前方が安全でないため、左に舵を切ります。")
                return 50  # 右側が安全でなければ左に曲がる

        rospy.loginfo("特に障害物が検出されていないため、直進します。")
        return 90  # それ以外の場合は直進


    def update(self):
        if all([self.front_left_distance, self.front_center_distance, self.front_right_distance, self.left_distance, self.right_distance]):
            # 前方の3つのセンサーから最小の距離を取得
            min_front_distance = min(self.front_left_distance, self.front_center_distance, self.front_right_distance)

            # 距離に基づいてモータの制御
            # if min_front_distance > 100:
            #     self.current_esc_pulse = min(self.esc_neutral + 22, self.esc_max)
            # elif min_front_distance > 60:
            #     self.current_esc_pulse = min(self.esc_neutral + 22, self.esc_max)
            # elif min_front_distance > 50:
            #     self.current_esc_pulse = min(self.esc_neutral + 22, self.esc_max)
            # elif min_front_distance > 40:
            #     self.current_esc_pulse = min(self.esc_neutral + 22, self.esc_max)
            # elif min_front_distance > 30:
            #     self.current_esc_pulse = min(self.esc_neutral + 22, self.esc_max)
            # elif min_front_distance > 21:
            #     self.current_esc_pulse = min(self.esc_neutral + 22, self.esc_max)
            if min_front_distance > 5:
                self.current_esc_pulse = min(self.esc_neutral + 23, self.esc_max)
            if min_front_distance > 3:
                self.current_esc_pulse = min(self.esc_neutral + 22, self.esc_max)
            else:
                self.current_esc_pulse = self.esc_neutral
            self.pwm.set_pwm(self.esc_channel, 0, self.current_esc_pulse)

            # サーボモータの制御
            servo_angle = self.calculate_servo_angle()

            if servo_angle is None:
                # 停止条件に該当する場合
                self.current_esc_pulse = self.esc_neutral
                rospy.loginfo("停止条件に該当するため、車両を停止します。")
                # ESCパルスの設定
                self.pwm.set_pwm(self.esc_channel, 0, self.current_esc_pulse)
            else:
                # サーボモーターの角度に基づいてESCパルスを設定
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
