#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import rospy
from sensor_msgs.msg import Range

# 設定値
trig_pin = 4
echo_pins = [5, 6, 7, 8, 9]  # 各エコーピンの番号
speed_of_sound = 34370  # 20℃での音速(cm/s)
topics = ['/ultrasonic/right', '/ultrasonic/front_right', '/ultrasonic/front_center', '/ultrasonic/front_left', '/ultrasonic/left']
timeout = 0.01  # タイムアウト値（秒）

def get_distance(trig_pin, echo_pin):
    try:
        # 距離を計測する関数
        GPIO.output(trig_pin, GPIO.HIGH)
        time.sleep(0.000010)
        GPIO.output(trig_pin, GPIO.LOW)

        start_time = time.time()

        # EchoピンがHIGHになるのを待機（タイムアウト付き）
        while not GPIO.input(echo_pin):
            if (time.time() - start_time) > timeout:
                return None  # タイムアウトした場合はNoneを返す
        t1 = time.time()

        # EchoピンがLOWになるのを待機（タイムアウト付き）
        while GPIO.input(echo_pin):
            if (time.time() - start_time) > timeout:
                return None  # タイムアウトした場合はNoneを返す
        t2 = time.time()

        return (t2 - t1) * speed_of_sound / 2
    except Exception as e:
        rospy.logerr(f"Error in get_distance: {e}")
        return None

def sensor_node():
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(trig_pin, GPIO.OUT)
        for pin in echo_pins:
            GPIO.setup(pin, GPIO.IN)

        rospy.init_node('sensor_node', anonymous=True)
        pubs = [rospy.Publisher(topic, Range, queue_size=10) for topic in topics]
        rate = rospy.Rate(30)  # 1秒間に30回の頻度で実行

        while not rospy.is_shutdown():
            for i, (pin, pub) in enumerate(zip(echo_pins, pubs)):
                distance = get_distance(trig_pin, pin)
                if distance is not None:
                    # rospy.loginfo(f"{topics[i]} Distance: {distance:.2f} cm")
                    range_msg = Range()
                    range_msg.range = distance / 100.0  # cmをmに変換
                    pub.publish(range_msg)
                # else:
                    # rospy.logerr(f"Failed to get distance for {topics[i]}")
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in sensor_node: {e}")
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    sensor_node()
