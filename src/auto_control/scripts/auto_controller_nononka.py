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
        front_left = self.front_left_distance
        front_right = self.front_right_distance
        front = self.front_center_distance
        right = self.right_distance
        left = self.left_distance

        f = r = l = e = 0
        rospy.sleep(0.5)

        while (e < 500):
            if front > 40 and (left > 25 or front_left > 30) and (right > 25 or front_right > 30):
                # rospy.loginfo("前方")
                f += 1
            elif left > 25 or front_left > 30:
                # rospy.loginfo("右折")
                l += 1
            elif right > 25 or front_right > 30:
                # rospy.loginfo("左折")
                r += 1
            elif front <= 40:
                if front_left <= front_right:
                    # rospy.loginfo("前方が近づいたので右折")
                    r += 1
                elif front_right <= front_left:
                    l += 1
            e += 1
            if f > 200:
                rospy.loginfo("前進")
                return 90
            if e > 400 and r > l:
                rospy.loginfo("右折")
                return 180
            if e > 400 and l > r:
                rospy.loginfo("左折")
                return 0
            # if e > 7:
                # break
        return None

    def update(self):
        if all([self.front_left_distance, self.front_center_distance, self.front_right_distance, self.left_distance, self.right_distance]):
            min_front_distance = min(self.front_left_distance, self.front_center_distance, self.front_right_distance)

            if min_front_distance > 5:
                self.current_esc_pulse = min(self.esc_neutral + 10, self.esc_max)
            elif min_front_distance > 3:
                self.current_esc_pulse = min(self.esc_neutral + 10, self.esc_max)
            else:
                self.current_esc_pulse = self.esc_neutral

            self.pwm.set_pwm(self.esc_channel, 0, self.current_esc_pulse)

            servo_angle = self.calculate_servo_angle()

            if servo_angle is None:
                self.current_esc_pulse = self.esc_neutral
                rospy.loginfo("停止条件に該当するため、車両を停止します。")
                self.pwm.set_pwm(self.esc_channel, 0, self.current_esc_pulse)
            else:
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
