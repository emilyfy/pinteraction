/*  pinteraction
 *  sketch for first row arduino mega controlling 10 motorized linear potentiometers
 *  
 *  moves the 10 pins according to the target height set by the high level programming
 *  through rosserial subscribe to topic /height/<rownumber> for the target heights of the 10 pins in the row
 *  
 *  checks for tactile feedback on pins for equalizer mode (mode 2)
 *  serial communication with another arduino mega to send info about feedback
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
unsigned int currTime, updateTime = 0, commTime = 0, chgTime = 0;

int stash[pins][10];
char off[pins] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
int vartn(int[]);

int mode = 0, prevmd = 0;

ros::NodeHandle nh;

void messageCb(const std_msgs::UInt16MultiArray& height) {
  
  updateTime = currTime;
  for (i=0;i<pins;i++) target[i] = height.data[i];
  
}
ros::Subscriber<std_msgs::UInt16MultiArray> sub("/height/1", &messageCb);

void setup() {  

  for (i=0;i<pins;i++) {
    pinMode(potPin[i], INPUT);
    pinMode(en[i], OUTPUT);
    pinMode(inA[i], OUTPUT);
    pinMode(inB[i], OUTPUT);
  }

  Serial3.begin(115200);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  currTime = millis();
  
  for (i=0;i<pins;i++) val[i] = analogRead(potPin[i]);
  
  nh.spinOnce();

  if (currTime - updateTime > 1000) {
    for (i=0;i<pins;i++) target[i] = 0;
    mode = 0;
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
  
  if (currTime > commTime) {
    prevmd = mode;
    nh.getParam("mode", &mode);
    if (prevmd!=2 && mode==2) chgTime = currTime;
    if (mode==2) sendFdb();
    commTime = commTime + 100;
  }
  
}

void goTarget(int pin) {
  if (abs(val[pin] - target[pin]) > 20 && off[pin]) {
    
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

void sendFdb() {
  for (int pin=0;pin<pins;pin++)
  {
    for (i=0;i<9;i++) stash[pin][i] = stash[pin][i+1];
    stash[pin][9] = val[pin];
  
    if (off[pin]==1)
    {
      if (vartn(stash[pin]) < 5 && val[pin]<10 && currTime-chgTime>5000) off[pin] = 0;
    }
  
    else
    {
      if (val[pin]>1013) off[pin] = 1;
    }
    
  }

  Serial3.write(0xFA);
  Serial3.write(0xBC);
  for (i=0;i<pins;i++){
    Serial3.write(off[i]);
  }
  Serial3.write(0xDE);
  
}

int vartn(int arr[]) {
  int amin = arr[0], amax = arr[0];
  for (i=1;i<pins;i++) {
    if (arr[i] < amin) amin = arr[i];
    if (arr[i] > amax) amax = arr[i];
  }
  return amax - amin;
}

