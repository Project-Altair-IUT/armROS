// #define __STM32F1__ //for stm32 bluepill usb serial comms

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include "Motor.h"
#include <Servo.h>

#define debug true
#define arm_armed ch[4] > 1200 and ch[4] < 1700

#define baseChannel 5
#define primaryChannel 3
#define secondaryChannel 2
#define wristPitchChannel 1
#define wristRollChannel 0
#define gripperChannel 6

#define gripperServoPin 13


// change as needed
Motor base(10, 11);
Motor primary(26, 27);
Motor secondary(28, 29);
Motor wristL(30, 31);
Motor wristR(32, 33);


bool isNear(int val1, int val2) {
  return (abs(val1 - val2) < 70);
}

inline int inverted(int val) {
  return map(val, 1000, 2000, 2000, 1000);
}

int ch[8];

void ros_callback(const std_msgs::Int16MultiArray& rc_vals) {
  // expecting value to be between 1000 to 2000
  for (int i = 0; i < 8; i++)
    ch[i] = rc_vals.data[i];
}


Servo gripperServo;
int pwmLen = 1500;
int increment_value = 10;

String data;

ros::Subscriber<std_msgs::Int16MultiArray> sub("rc_signal", ros_callback);

void halt() {
  primary.rotate(1500);

  secondary.rotate(1500);

  wristL.rotate(1500);

  wristR.rotate(1500);
}

ros::NodeHandle  nh;
#define LOOPTIME 10

void setup() {

  nh.initNode();
  nh.subscribe(sub);

  nh.getHardware()->setBaud(57600);

  gripperServo.attach(gripperServoPin);

  delay(2000);
}

char* str;

void loop() {
  nh.spinOnce();
  sprintf(str, "ch %d: %d", primaryChannel, ch[primaryChannel]);

  if (arm_armed) {
    primary.rotate(ch[primaryChannel]);
    nh.loginfo(str);

    secondary.rotate(ch[secondaryChannel]);

    if (!isNear(ch[wristPitchChannel], 1500)) {
      int x_w = ch[wristPitchChannel];
      x_w = constrain(x_w, 1300, 1700);
      wristL.rotate(x_w);
      wristR.rotate(inverted(x_w));
    }

    else {
      int x_w = ch[wristRollChannel];
      x_w = constrain(x_w, 1300, 1700);
      wristL.rotate(x_w);
      wristR.rotate(x_w);
    }

    if (ch[gripperChannel] > 1600) {
      pwmLen += increment_value;
    } else if (ch[gripperChannel] < 1400) {
      pwmLen -= increment_value;
    }
    gripperServo.writeMicroseconds(pwmLen);

  } else {
    halt();
  }
}
