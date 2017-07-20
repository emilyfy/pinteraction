#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
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
double target[pins];

// equalizer feedback
unsigned int currTime, commTime = 0;
int val[pins];
int stash[pins][10];
int off[pins] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int vartn(int[]);

// glove sensor
const int sensorPin = A10;

int mode = 0;

ros::NodeHandle nh;

void heightCb(const std_msgs::UInt16MultiArray& height) {
  
  for (i=0;i<pins;i++) target[i] = height.data[i];
  
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub_ht("/height/1", &heightCb); //change accordingly to row no of arduino
std_msgs::Bool fdb;
ros::Publisher pub_fdb("/feedback", &fdb);
std_msgs::UInt16 dst;
ros::Publisher pub_dst("/distance", &dst);

void setup() {  

  for (i=0;i<pins;i++) {
    pinMode(potPin[i], INPUT);
    pinMode(en[i], OUTPUT);
    pinMode(inA[i], OUTPUT);
    pinMode(inB[i], OUTPUT);
  }

  for (i=0;i<pins;i++) fdb.data[i] = true;
  
  nh.initNode();
  nh.subscribe(sub_ht);
  nh.advertise(pub_fdb);
  nh.advertise(pub_dst);

  pinMode(sensorPin, INPUT);
}

void loop() {
  currTime = millis();
  
  for (i=0;i<pins;i++) val[i] = analogRead(potPin[i]);
  
  nh.spinOnce();
  
  for (i=0;i<pins;i++) goTarget(i);

  if (mode==2 || mode==3)
  {
    if (currTime > commTime) {
      nh.getParam("mode", &mode);
      if (mode==2) for (i=0;i<pins;i++) checkFdb(i);
      else if (mode==3) sendGlove();
      commTime = commTime + 100;
    }
  }
  
}

void goTarget(int pin) {
  if (off[pin]) return;
  
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

void checkFdb(int pin) {
  for (i=0;i<9;i++) stash[pin][i] = stash[pin][i+1];
  stash[pin][9] = val[pin];

  if (off[pin]==0)
  {
    if (vartn(stash[pin]) < 5 && val[pin]<10) {
      fdb.data[pin] = false;
      pub_fdb.publish( &fdb );
      off[pin] = 1;
    }
  }

  else
  {
    if (val[pin]>1013) {
      fdb.data[pin] = true;
      pub_fdb.publish( &fdb );
      off[pin] = 0;
    }
  }
  
}

int vartn(int arr[]) {
  int amin = arr[0], amax = arr[0];
  for (i=1;i<pins;i++) {
    if (arr[i] < amin) amin = arr[i];
    if (arr[i] > amax) amax = arr[i];
  }
  return amax - amin;
}

void sendGlove(void) {
  dst.data = analogRead(sensorPin);
  pub_fdb.publish( &dst );
}

