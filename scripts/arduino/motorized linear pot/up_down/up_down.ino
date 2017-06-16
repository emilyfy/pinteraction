#define potPin    A1      // connect to potentiometer Vref
#define en        3       // connect to EN on h-bridge. has to be pwm pin
#define in1       16      // connect to h-bridge in1
#define in2       17      // connect to h-bridge in2

const int period = 250;  // milliseconds

void setup() {
  Serial.begin(9600);
  pinMode(potPin, INPUT);
  pinMode(en, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void loop() {
  int val = analogRead(potPin);
  unsigned int curr = millis();
  curr = curr%period;
  double target;
  if (curr<period/2) target = curr/(period/2.0)*10;
  else  target = curr/(period/2.0)*-10 + 20;
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
    analogWrite(en, max(min(abs(val - targetv), 255), 200));
  }
  
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);  
    analogWrite(en, 0);
  }
}
