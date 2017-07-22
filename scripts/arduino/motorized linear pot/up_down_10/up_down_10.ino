#define pins 10     // max of 12 pins can be set
int i;

const int potPin[pins] = {A9, A8, A7, A6, A5, A4, A3, A2, A1, A0};
const int en[pins]     = { 2,  3,  4,  5,  6,  7,  8,  9, 10, 11};
const int inA[pins]    = {22, 24, 26, 28, 30, 32, 34, 36, 38, 40};
const int inB[pins]    = {23, 25, 27, 29, 31, 33, 35, 37, 39, 41};
double target[pins];
int targetv[pins];
int val[pins];

const int period = 250;  // milliseconds

void setup() {
  Serial.begin(9600);

  for (i=0;i<pins;i++) {
    pinMode(potPin[i], INPUT);
    pinMode(en[i], OUTPUT);
    pinMode(inA[i], OUTPUT);
    pinMode(inB[i], OUTPUT);
  }
}

void loop() {
  for (i=0;i<pins;i++) val[i] = analogRead(potPin[i]);
  
  for (i=0;i<pins;i++) goTarget(i);
  
  unsigned int curr = millis();
  curr = curr%period;
  double target;
  if (curr<period/2) target = curr/(period/2.0)*10;
  else  target = curr/(period/2.0)*-10 + 20;

  for (i=0;i<pins;i++) {
    targetv[i] = target/10.0*1023;;
  }

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
