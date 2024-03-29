#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import rospy
from sensor_msgs.msg import Range
import numpy as np  # NumPyをインポート

# 設定値
trig_pin = 4
echo_pins = [5, 6, 7, 8, 9]  # 各エコーピンの番号
speed_of_sound = 34370  # 20℃での音速(cm/s)
topics = ['/ultrasonic/right', '/ultrasonic/front_right', '/ultrasonic/front_center', '/ultrasonic/front_left', '/ultrasonic/left']
timeout = 0.01  # タイムアウト値（秒）
num_samples = 8  # メディアンフィルタリングに使用するサンプル数

# 各エコーピンに対応する距離サンプルのリストを保持する辞書
distance_samples = {pin: [] for pin in echo_pins}

def get_distance(trig_pin, echo_pin):
    try:
        # 距離を計測する関数
        GPIO.output(trig_pin, GPIO.HIGH)
        time.sleep(0.00001)
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

def get_median_distance(trig_pin, echo_pin):
    global distance_samples
    distance = get_distance(trig_pin, echo_pin)
    if distance is not None:
        # サンプルリストに距離を追加し、サイズを制限する
        distance_samples[echo_pin].append(distance)
        if len(distance_samples[echo_pin]) > num_samples:
            distance_samples[echo_pin].pop(0)
        # メディアンを計算して返す
        return np.median(distance_samples[echo_pin])
    return None

def sensor_node():
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(trig_pin, GPIO.OUT)
        for pin in echo_pins:
            GPIO.setup(pin, GPIO.IN)

        rospy.init_node('sensor_node', anonymous=True)
        pubs = [rospy.Publisher(topic, Range, queue_size=10) for topic in topics]
        rate = rospy.Rate(22)  # 1秒間に20回の頻度で実行

        while not rospy.is_shutdown():
            for i, (pin, pub) in enumerate(zip(echo_pins, pubs)):
                median_distance = get_median_distance(trig_pin, pin)
                if median_distance is not None:
                    range_msg = Range()
                    range_msg.range = median_distance / 100.0  # cmをmに変換
                    pub.publish(range_msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in sensor_node: {e}")
    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    sensor_node()
