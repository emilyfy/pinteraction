#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>

// use an arduino mega
// pwm 2-13

#define potPin1    A0        // connect to potentiometer Vref
#define en1        2         // if they have one. if not ignore this
#define inA1       14        // connect to h-bridge input A
#define inB1       15        // connect to h-bridge input B

#define potPin2    A1
#define en2        3
#define inA2       16
#define inB2       17

double target1, target2;
int target_1, target_2;

ros::NodeHandle nh;

void messageCb(const std_msgs::Float64MultiArray& height) {
  target1 = height.data[0];
  target2 = height.data[1];

  target_1 = target1/10.0*1023;
  target_2 = target2/10.0*1023;
  
}

ros::Subscriber<std_msgs::Float64MultiArray> sub("/fourier", &messageCb);

void setup() {
  Serial.begin(9600);
  
  pinMode(potPin1, INPUT);
  pinMode(en1, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inB1, OUTPUT);
  
  pinMode(potPin2, INPUT);
  pinMode(en2, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB2, OUTPUT);

  digitalWrite(en1, LOW);
  digitalWrite(inA1, LOW);
  digitalWrite(inB1, LOW);
  digitalWrite(en2, LOW);
  digitalWrite(inA2, LOW);
  digitalWrite(inB2, LOW);
  
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  int val1 = analogRead(potPin1);
  int val2 = analogRead(potPin2);

  nh.spinOnce();
  
  Serial.print("target 1 = ");
  Serial.print(target1);
  Serial.print("\ttarget 2 = ");
  Serial.println(target2);

  // motor 1
  if (abs(val1-target_1) > 20) {
    if(val1 > target_1) {
      digitalWrite(inA1, LOW);
      digitalWrite(inB1, HIGH);
    }
    else {
      digitalWrite(inA1, HIGH);
      digitalWrite(inB1, LOW); 
    }
    analogWrite(en1, max(min(abs(val1 - target_1), 255), 200));
  }
  
  else{
    digitalWrite(inA1, LOW);
    digitalWrite(inB1, LOW);  
    analogWrite(en1, 0);
  }

  // motor 2
  if (abs(val2-target_2) > 20) {
    if(val2 > target_2) {
      digitalWrite(inA2, LOW);
      digitalWrite(inB2, HIGH);
    }
    else {
      digitalWrite(inA2, HIGH);
      digitalWrite(inB2, LOW); 
    }
    analogWrite(en2, max(min(abs(val2 - target_2), 255), 200));
  }
  
  else{
    digitalWrite(inA2, LOW);
    digitalWrite(inB2, LOW);  
    analogWrite(en2, 0);
  }

  // this is bad style. will change (or you guys can) to using array/class so that for loop/class method can be used.
}
