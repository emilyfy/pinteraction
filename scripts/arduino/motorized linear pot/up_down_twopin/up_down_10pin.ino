#define potPin0   A0
#define potPin    A1      // connect to potentiometer Vref

#define en0       2
#define en        3       // connect to EN on h-bridge. has to be pwm pin

#define in10      14 
#define in20      15 

#define in1       16      // connect to h-bridge in1
#define in2       17      // connect to h-bridge in2

const int period = 250;  // milliseconds

void setup() {
  Serial.begin(9600);
  pinMode(potPin, INPUT);
  pinMode(en, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  pinMode(potPin0, INPUT);
  pinMode(en0, OUTPUT);
  pinMode(in10, OUTPUT);
  pinMode(in20, OUTPUT);
}

void loop() {
  int val = analogRead(potPin);
  int val0 = analogRead(potPin0);
    
  unsigned int curr = millis();
  curr = curr%period;
  double target;
  if (curr<period/2) target = curr/(period/2.0)*10;
  else  target = curr/(period/2.0)*-10 + 20;
  Serial.println(target);
  if (abs(val-targetv) > 20) {
    if(val > targetv) {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);

    }
    else {      digitalWrite(in2, LOW);
  int targetv = target/10.0*1023;


      digitalWrite(in1, HIGH);

    }
    analogWrite(en, max(min(abs(val - targetv), 255), 200));
  }
  
  if (abs(val0-targetv) > 20) {
    if(val0 > targetv) {
      digitalWrite(in10, LOW);
      digitalWrite(in20, HIGH);

      digitalWrite(in10, HIGH);
      digitalWrite(in20, LOW);     else {    }

    }
    analogWrite(en0, max(min(abs(val0 - targetv), 255), 200));
    }
   
  
  
  else{
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);  
    analogWrite(en, 0);
    
    digitalWrite(in10, LOW);
    digitalWrite(in20, LOW);  
    analogWrite(en0, 0);
  }
  
}
