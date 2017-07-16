#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <math.h>

// use an arduino mega, pwm 2-13
// for pin n (n from 0 to 9)
// potPin          An               connect to potentiometer Vref
// en              n+2              connect to EN on h-bridge. has to be pwm pin. first pin in molex
// inA             2n+14            connect to h-bridge in1. middle pin in molex
// inB             2n+15            connect to h-bridge in2. third pin in molex

#define pins 10     // max of 12 pins can be set
int i;

const int potPin[pins] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9};
const int en[pins]     = { 2,  3,  4,  5,  6,  7,  8,  9, 10, 11};
const int inA[pins]    = {22, 24, 26, 28, 30, 32, 34, 36, 38, 40};
const int inB[pins]    = {23, 25, 27, 29, 31, 33, 35, 37, 39, 41};
int target[pins];
int val[pins];

ros::NodeHandle nh;

void messageCb(const std_msgs::UInt16MultiArray& height) {
  
  for (i=0;i<pins;i++) target[i] = height.data[i];
  
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub("/height/6", &messageCb); //change accordingly to row no of arduino

void setup() {  

  for (i=0;i<pins;i++) {
    pinMode(potPin[i], INPUT);
    pinMode(en[i], OUTPUT);
    pinMode(inA[i], OUTPUT);
    pinMode(inB[i], OUTPUT);
  }
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  for (i=0;i<pins;i++) val[i] = analogRead(potPin[i]);
  
  nh.spinOnce();
  
  for (i=0;i<pins;i++) goTarget(i);

}

void goTarget(int pin) {
  if (abs(val[pin] - target[pin]) > 20) {
    
    if(val[pin] > target[pin]) {
      digitalWrite(inA[pin], LOW);
      digitalWrite(inB[pin], HIGH);
    }
    else {
      digitalWrite(inA[pin], HIGH);
      digitalWrite(inB[pin], LOW); 
    }
    analogWrite(en[pin], max(min(abs(val[pin] - target[pin]), 255), 200));
  }
  
  else{
    digitalWrite(inA[pin], LOW);
    digitalWrite(inB[pin], LOW);
    analogWrite(en[pin], 0);
  }
}

