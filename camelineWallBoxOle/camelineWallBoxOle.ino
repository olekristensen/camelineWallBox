#include <AccelStepper.h>

const int buttonPin = 3;
int lastButtonState = LOW;
int wasButtonPushed = LOW;
int buttonState;
long lastButtonPushed = 0;
long buttonDebounceDelay = 50L;
boolean buttonRisingEdge = false;

long maxPositionSteps = 0L;
boolean foundMaxPositionSteps = false;
long minPositionSteps = 0L;
boolean foundMinPositionSteps = false;
int maxSpeed = 1000;
int maxAcceleration = round(maxSpeed * 0.5);
long margin = 200;

const int sensorPin = 2;

AccelStepper stepper( AccelStepper::FULL4WIRE, 8, 9, 12, 13);

void setup() {

  Serial.begin(9600); // set up Serial library at 9600 bps
  pinMode(buttonPin, INPUT);  // initialize the pushbutton pin as an input:  
  pinMode(sensorPin, INPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(maxAcceleration);
}

void loop(){

  /*********
   ** BUTTON
   ** debouncing button and changing state on logical rising edge
   *********/

  int isButtonPushed = digitalRead(buttonPin);
  if(isButtonPushed != lastButtonState){
    lastButtonPushed = millis();
  }
  if((millis() - lastButtonPushed) > buttonDebounceDelay) {
    buttonState = isButtonPushed;
  }
  buttonRisingEdge = (!wasButtonPushed && buttonState);

  /***********
   ** FIND maxDistanceSteps
   ***********/

  if(!foundMinPositionSteps){
    // find starting point
    if(buttonRisingEdge){
      foundMinPositionSteps = true;
      Serial.print(" MIN:");
      Serial.println(minPositionSteps);
      stepper.setAcceleration(maxSpeed*8);
      stepper.stop();
      stepper.runToPosition();
      stepper.setAcceleration(maxAcceleration);
      stepper.setSpeed(maxSpeed);
      goToMiddleIfFound();
    } 
    else {
      if(stepper.distanceToGo() > -500){
        stepper.moveTo(stepper.currentPosition()-1000);
      }
    }
    minPositionSteps = stepper.currentPosition()+margin;
  } 
  else if (!foundMaxPositionSteps) {
    if(buttonRisingEdge){
      foundMaxPositionSteps = true;
      Serial.print(" MAX:");
      Serial.println(maxPositionSteps);
      stepper.setAcceleration(maxSpeed*8);
      stepper.stop();
      stepper.runToPosition();
      stepper.setAcceleration(maxAcceleration);
      stepper.setSpeed(maxSpeed);
      goToMiddleIfFound();
    } 
    else {
      if(stepper.distanceToGo() < 500){
        stepper.moveTo(stepper.currentPosition()+1000);
      }
    }
    maxPositionSteps = stepper.currentPosition()-margin;
  } 
  else {

    /**********
     ** MOVE
     **********/

    if(buttonRisingEdge){
      long currentPosition = stepper.currentPosition();
      long targetPosition = stepper.targetPosition();
      if(targetPosition > currentPosition){
        // found new max
        maxPositionSteps = currentPosition-margin;
        Serial.print(" MAX:");
        Serial.println(maxPositionSteps);
        // start looking for a new min
        foundMinPositionSteps = false;
      } 
      else {
        // found new min
        minPositionSteps = currentPosition+margin;
        Serial.print(" MIN:");
        Serial.println(minPositionSteps);
        // start looking for a new max
        foundMaxPositionSteps = false;
      }
      // button was pushed, stop the spinning
      stepper.setAcceleration(maxSpeed*8);
      stepper.stop();
      stepper.runToPosition();
      // set max speed and acc for searching the other end
      stepper.setAcceleration(maxAcceleration);
      stepper.setSpeed(maxSpeed);
    } 
    else {
      if (stepper.distanceToGo() == 0)
      {

        /* PING SENSOR LOGIC
        // establish variables for duration of the ping, 
        // and the distance result in inches and centimeters:
        long duration, cm;

        // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
        // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
        pinMode(sensorPin, OUTPUT);
        digitalWrite(sensorPin, LOW);
        delayMicroseconds(2);
        digitalWrite(sensorPin, HIGH);
        delayMicroseconds(5);
        digitalWrite(sensorPin, LOW);

        // The same pin is used to read the signal from the PING))): a HIGH
        // pulse whose duration is the time (in microseconds) from the sending
        // of the ping to the reception of its echo off of an object.
        pinMode(sensorPin, INPUT);
        duration = pulseIn(sensorPin, HIGH);

        // convert the time into a distance
        cm = microsecondsToCentimeters(duration);

        Serial.print(cm);
        Serial.print("cm");
        Serial.println();

        if(cm < pingMaxCm){
          setNewRandomDestination();
        } else {
          delay(100);
        }
        */
        if(digitalRead(sensorPin) == HIGH){
          setNewRandomDestination();
        } else {
          delay(100);
        }
      }
    }
  }

  stepper.run();

  //Serial.print("CURRENT:");
  //Serial.println(stepper.currentPosition());

  lastButtonState = isButtonPushed;
  wasButtonPushed = buttonState;

}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

void setNewRandomDestination(){
  // Random change to speed, position and acceleration
  // Make sure we dont get 0 speed or accelerations
  delay(1000);
  long destinationSteps = random(minPositionSteps, maxPositionSteps);
  Serial.print("RUN TO: ");
  Serial.println(destinationSteps);
  stepper.moveTo(destinationSteps);
  stepper.setMaxSpeed((rand() % maxSpeed) + 1);
  stepper.setAcceleration((rand() % maxAcceleration) + 1);
}

void goToMiddleIfFound(){
  if(foundMinPositionSteps && foundMaxPositionSteps){
    Serial.print("RUN TO MIDDLE: ");
    long destinationSteps = (maxPositionSteps+minPositionSteps)/2;
    Serial.println(destinationSteps);
    stepper.setSpeed(maxSpeed);
    stepper.setAcceleration(maxAcceleration);
    stepper.moveTo(destinationSteps);
  }
}
