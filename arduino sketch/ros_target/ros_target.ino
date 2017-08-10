/*  pinteraction
 *  sketch for arduino mega controlling 10 motorized linear potentiometers
 *  moves the 10 pins according to the target height set by the high level programming
 *  through rosserial subscribe to topic /height/<rownumber> for the target heights of the 10 pins in the row
 */

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

const int potPin[pins] = {A9, A8, A7, A6, A5, A4, A3, A2, A1, A0};
const int en[pins]     = { 2,  3,  4,  5,  6,  7,  8,  9, 10, 11};
const int inA[pins]    = {22, 24, 26, 28, 30, 32, 34, 36, 38, 40};
const int inB[pins]    = {23, 25, 27, 29, 31, 33, 35, 37, 39, 41};
int target[pins];
int val[pins];
unsigned int currTime, updateTime = 0;

ros::NodeHandle nh;

void messageCb(const std_msgs::UInt16MultiArray& height) {
  
  updateTime = currTime;
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
  currTime = millis();
  
  for (i=0;i<pins;i++) val[i] = analogRead(potPin[i]);
  
  nh.spinOnce();

  if (currTime - updateTime > 1000) {
    for (i=0;i<pins;i++) target[i] = 0;
  }
  
  if (currTime - updateTime > 2000) {
    for (i=0;i<pins;i++) {
      digitalWrite(inA[i], LOW);
      digitalWrite(inB[i], LOW);
      analogWrite(en[i], 0);
    }
  }
  else {
    for (i=0;i<pins;i++) goTarget(i);
  }

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

