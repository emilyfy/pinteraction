/* this is the same as ros_target
 *  with an additional str sent to ros to check for arduino's values
 *  check values by running on terminal $ rostopic echo /print
 */

#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <math.h>

// use an arduino mega, pwm 2-13
// for pin n (n from 0 to 9)
// potPin          An               connect to potentiometer Vref
// en              n+2              connect to EN on h-bridge. has to be pwm pin       
// inA             2n+14            connect to h-bridge in1
// inB             2n+15            connect to h-bridge in2

#define pins 12     // max of 12 pwm pins
int i;
String str;

const int potPin[pins] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11};
const int en[pins] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
const int inA[pins] = {14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36};
const int inB[pins] = {15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35, 37};
double target[pins];
int targetv[pins];
int val[pins];

ros::NodeHandle nh;

void messageCb(const std_msgs::Float64MultiArray& height) {
  for (i=0;i<pins;i++) {
    target[i] = height.data[i];
    targetv[i] = target[i]/10.0*1023;
  }

  str = String("1: " + String(target[0]) + " 2: " + String(target[1]) + " 3: " + String(target[2]) + " 4: " + String(target[3]) + " 5: " + String(target[4]) + " 6: " + String(target[5]) + " 7: " + String(target[6]) + " 8: " + String(target[7]) + " 9: " + String(target[8]) + " 10: " + String(target[9]) + " 11: " + String(target[10]) + " 12: " + String(target[11]) );
}

std_msgs::String str_msg;
ros::Publisher pub_data("/print", &str_msg);
ros::Subscriber<std_msgs::Float64MultiArray> sub("/fourier", &messageCb);

void setup() {  

  for (i=0;i<pins;i++) {
    pinMode(potPin[i], INPUT);
    pinMode(en[i], OUTPUT);
    pinMode(inA[i], OUTPUT);
    pinMode(inB[i], OUTPUT);
  }
  
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub_data);
}

void loop() {
  for (i=0;i<pins;i++) val[i] = analogRead(potPin[i]);
  
  char _str[str.length()+1];
  str.toCharArray(_str, sizeof(_str));
  str_msg.data = _str;
  pub_data.publish( &str_msg );
  nh.spinOnce();
  
  for (i=0;i<pins;i++) goTarget(i);

}

void goTarget(int pin) {
  if (abs(val[pin] - targetv[pin]) > 20) {
    
    if(val[pin] > targetv[pin]) {
      digitalWrite(inA[pin], LOW);
      digitalWrite(inB[pin], HIGH);
    }
    else {
      digitalWrite(inA[pin], HIGH);
      digitalWrite(inB[pin], LOW); 
    }
    analogWrite(en[pin], max(min(abs(val[pin] - targetv[pin]), 255), 200));
  }
  
  else{
    digitalWrite(inA[pin], LOW);
    digitalWrite(inB[pin], LOW);
    analogWrite(en[pin], 0);
  }
}

