#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Float64

# 設定値
trig_pin = 4
echo_pins = [5, 6, 7, 8, 9]  # 各エコーピンの番号
speed_of_sound = 34370  # 20℃での音速(cm/s)

def get_distance(trig_pin, echo_pin):
    # 距離を計測する関数
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.000010)
    GPIO.output(trig_pin, GPIO.LOW)

    while not GPIO.input(echo_pin):
        pass
    t1 = time.time()

    while GPIO.input(echo_pin):
        pass
    t2 = time.time()

    return (t2 - t1) * speed_of_sound / 2

def sensor_node():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(trig_pin, GPIO.OUT)
    for pin in echo_pins:
        GPIO.setup(pin, GPIO.IN)

    rospy.init_node('sensor_node', anonymous=True)
    pubs = [rospy.Publisher(f'ultrasound_{i+1}', Float64, queue_size=10) for i in range(len(echo_pins))]
    rate = rospy.Rate(10)  # 10Hz

    while not rospy.is_shutdown():
        for i, pin in enumerate(echo_pins):
            distance = get_distance(trig_pin, pin)
            rospy.loginfo(f"Distance {i+1}: {distance:.2f} cm")
            pubs[i].publish(Float64(distance))
        rate.sleep()

    GPIO.cleanup()

if __name__ == '__main__':
    try:
        sensor_node()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
        pass
