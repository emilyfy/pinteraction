#define potPin    A0      // connect to potentiometer Vref
#define enA       5       // if they have one. if not ignore this
#define in1       9       // connect to h-bridge out1
#define in2       8       // connect to h-bridge out2

void setup() {
  Serial.begin(9600);
  pinMode(potPin, INPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void loop() {
  int val = analogRead(potPin);
  unsigned int curr = millis();
  curr = curr%1000;
  double target;
  if (curr<500) target = curr*10.0/500.0;
  else  target = curr*-10.0/500.0 + 20;
  Serial.println(target);
  int targetv = target/10.0*1023;

  if (abs(val-targetv) > 20) {
    if(val > targetv) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    else {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW); 
    }
    analogWrite(enA, max(min(abs(val - targetv), 255), 200));
  }
  
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);  
    analogWrite(enA, 0);
  }
}
