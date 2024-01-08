#include <Servo.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Time.h>

// Direction (dir)
#define MAXLEFT              2000
#define MAXRIGHT             1000
#define MIDDLE               1500
#define SERVOPIN                8

// Speed (throttle)
#define THROTTLE_MAXREVERSE  1000
#define THROTTLE_MAXFORWARD  2000
#define THROTTLE_IDLE        1500
#define THROTTLEPIN          9


ros::NodeHandle nh;

Servo servo;
Servo throt;

void servo_cb(const std_msgs::Int16& cmd_msg) {
  servo.write(cmd_msg.data);
  digitalWrite(13, HIGH-digitalRead(13));
}

void throt_cb(const std_msgs::Int16& cmd_msg) {
  throt.write(cmd_msg.data);
  digitalWrite(13, HIGH-digitalRead(13));
}


ros::Subscriber<std_msgs::Int16> sub_servo("servo", servo_cb);
ros::Subscriber<std_msgs::Int16> sub_throt("esc", throt_cb);

std_msgs::Time time_msg;
ros::Publisher chatter("chatter", &time_msg);


void throt_init() {
  throt.attach(THROTTLEPIN);

  // ESC init sequence
  throt.writeMicroseconds(THROTTLE_MAXFORWARD);
  delay(25);
  throt.writeMicroseconds(THROTTLE_MAXREVERSE);
  delay(25);
  throt.writeMicroseconds(THROTTLE_IDLE);
}


void servo_init() {
  servo.attach(SERVOPIN);
}



void setup() {
  servo_init();
  throt_init();
  Serial.begin(9600);

  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub_servo);
  nh.subscribe(sub_throt);

  nh.advertise(chatter);
}

void loop() {
  nh.spinOnce();
  time_msg.data = nh.now();
  chatter.publish(&time_msg);
  delay(1);
}


