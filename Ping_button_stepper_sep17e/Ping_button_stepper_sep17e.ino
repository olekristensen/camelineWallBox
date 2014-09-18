
#include <AFMotor.h>

AF_Stepper motor(48, 2);

const int buttonPin = 1;
int buttonState = 0;
int pingPin = 2;
int pingValue = 0;
int distanceMm = 0;



void setup() {
  
  Serial.begin(9600); // set up Serial library at 9600 bps
 
  pinMode(buttonPin, INPUT);  // initialize the pushbutton pin as an input:  

  motor.setSpeed(100);  // 10 rpm   

  motor.step(100, FORWARD, SINGLE); 
  motor.release();
  delay(1000);  
 

}

void loop(){
  
  long duration, mm;

  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  
  mm = microsecondsToMillimeters(duration); 
  
//read state of pushbutton value
buttonState = digitalRead(buttonPin);

if (buttonState == HIGH) {     
    // make motor turn down:    
     motor.setSpeed(100);
     motor.step(100, FORWARD, SINGLE);
  }
  else if (buttonState == LOW) {
   motor.setSpeed(100);
   motor.step(100, BACKWARD, SINGLE);
   
   else if (distanceMm>1000)
   motor.setSpeed(100);
   motor.step(100, BACKWARD, SINGLE);
   
  }
}
