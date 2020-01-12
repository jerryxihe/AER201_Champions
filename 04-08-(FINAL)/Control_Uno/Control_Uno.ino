#include <Wire.h>
#include <elapsedMillis.h>
#include <Stepper.h>

#define encoderPin 2
#define runFinished 4
#define startDrive 5
#define returnToStartCommand 6
#define startRun 7
//#define enL 5
//#define enR 6
//#define in1 7
//#define in2 8
//#define in3 9
//#define in4 10
#define unoBusy 11
#define stopExtending 12
#define armStepperIn1 14
#define armStepperIn2 15
#define armStepperIn3 16
#define armStepperIn4 17

elapsedMillis timer;
unsigned int poleDuration = 0;
float poleDist = 0;
volatile unsigned int distTravelled = 0;
volatile boolean isRunning = false;

volatile unsigned char command = 0;     // Command from PIC
volatile bool newCommand = false;       // Used to check if command has changed

volatile unsigned long encoderPos = 0;  // a counter for the dial
volatile boolean rotating = false;     // debounce management
volatile boolean set = false;

const int stepsPerRevolution = 500;
volatile int armStepperCurrentPosition = 0;
volatile int armStepperNewPosition = 0;
Stepper armStepper(stepsPerRevolution, armStepperIn1, armStepperIn2, armStepperIn3, armStepperIn4);

unsigned long i = 0;

void setup() {
  pinMode(runFinished, OUTPUT);
  digitalWrite(runFinished, LOW);
  
  Wire.begin(8); // Join I2C bus with address 8
  // Register callback functions
  Wire.onReceive(receiveEvent); // Called when this slave device receives a data transmission from master
  Wire.onRequest(requestEvent); // Called when master requests data from this slave device
  
  pinMode(encoderPin, INPUT);
  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(digitalPinToInterrupt(encoderPin), doEncoder, CHANGE);

  pinMode(startDrive, OUTPUT);
  digitalWrite(startDrive, LOW);

  pinMode(returnToStartCommand, OUTPUT);
  digitalWrite(returnToStartCommand, LOW);

  pinMode(startRun, OUTPUT);
  digitalWrite(startRun, LOW);
  
//  pinMode(enL, OUTPUT);
//  pinMode(enR, OUTPUT);
//  pinMode(in1, OUTPUT);
//  pinMode(in2, OUTPUT);
//  pinMode(in3, OUTPUT);
//  pinMode(in4, OUTPUT);
//  digitalWrite(enL, LOW);
//  digitalWrite(enR, LOW);
//  digitalWrite(in1, LOW);
//  digitalWrite(in2, LOW);
//  digitalWrite(in3, LOW);
//  digitalWrite(in4, LOW);

  pinMode(unoBusy, OUTPUT); 
  digitalWrite(unoBusy, LOW);

  pinMode(stopExtending, INPUT); 

  armStepper.setSpeed(60);
  pinMode(armStepperIn1, OUTPUT);
  pinMode(armStepperIn2, OUTPUT);
  pinMode(armStepperIn3, OUTPUT);
  pinMode(armStepperIn4, OUTPUT);
  digitalWrite(armStepperIn1, LOW);
  digitalWrite(armStepperIn2, LOW);
  digitalWrite(armStepperIn3, LOW);
  digitalWrite(armStepperIn4, LOW);
  
  Serial.begin(9600);
}

void returnToStart(){
//  Serial.println("returning");
  digitalWrite(runFinished, HIGH);
  isRunning = false;
  distTravelled = encoderPos;
  encoderPos = 1;
  armStepper.step(-armStepperCurrentPosition-20);
  armStepperCurrentPosition = 0;
  digitalWrite(armStepperIn1, LOW);
  digitalWrite(armStepperIn2, LOW);
  digitalWrite(armStepperIn3, LOW);
  digitalWrite(armStepperIn4, LOW);
  digitalWrite(returnToStartCommand, HIGH);
//  Serial.println("start driven high");
//  Serial.println("encoder");
//  Serial.println(distTravelled);
//  Serial.println(encoderPos);
  while ((encoderPos < distTravelled + 140) && (timer < 175000)){
    rotating = true;
//    Serial.println(encoderPos);
  }
  digitalWrite(returnToStartCommand, LOW);
  digitalWrite(runFinished, LOW);
}

void loop() {
//  Serial.println(timer);
  rotating = true;
//  Serial.println(encoderPos);
  // Reached 4 metres or 2:30 has elapsed
  // 820
  // 806
  if (isRunning && (((encoderPos) > 865) || (timer > 153000))){
//    Serial.println("2:35");
    digitalWrite(runFinished, HIGH);
  }
  
  if (newCommand){
    // New run
    if (command == 1){
      encoderPos = 1;
      timer = 0;
      isRunning = true;
      armStepperCurrentPosition = 0;
      digitalWrite(startRun, HIGH);
      delay(50);
      digitalWrite(startRun, LOW);
    } else if (isRunning){
      switch(command){        
        // Normal speed forward
        case 2:
          digitalWrite(startDrive, HIGH);
          delay(50);
          digitalWrite(startDrive, LOW);
          break;
          
        // Extend arm
        case 4:
          while (!digitalRead(stopExtending)){
            armStepper.step(1);
            armStepperCurrentPosition++;
            if (armStepperCurrentPosition >= 1750){
              break;
            }
          }
          armStepper.step(180);
          armStepperCurrentPosition += 180;
          digitalWrite(armStepperIn1, LOW);
          digitalWrite(armStepperIn2, LOW);
          digitalWrite(armStepperIn3, LOW);
          digitalWrite(armStepperIn4, LOW);
//          Serial.print("stepped forward");
//          Serial.println(armStepperCurrentPosition);
          break;

        // Retract arm
        case 5:
//          Serial.print("Stepping backward ");
//          Serial.println(armStepperCurrentPosition);
          armStepper.step(-armStepperCurrentPosition-20);
          armStepperCurrentPosition = 0;
          digitalWrite(armStepperIn1, LOW);
          digitalWrite(armStepperIn2, LOW);
          digitalWrite(armStepperIn3, LOW);
          digitalWrite(armStepperIn4, LOW);
          break;
  
        // Reached max poles or out of tires
        case 6:
//          Serial.println("returning");
          returnToStart();
          break;

        // Position cell
        case 7:
          if (armStepperCurrentPosition >= 950){
            armStepper.step(-950);
            armStepperCurrentPosition -= 950;
          } else {
            armStepper.step(-armStepperCurrentPosition-20);
            armStepperCurrentPosition = 0;
          }
          digitalWrite(armStepperIn1, LOW);
          digitalWrite(armStepperIn2, LOW);
          digitalWrite(armStepperIn3, LOW);
          digitalWrite(armStepperIn4, LOW);
//          Serial.print("Position cell: ");
//          Serial.println(armStepperCurrentPosition);
          break;
      }
    }
  newCommand = false;
  digitalWrite(unoBusy, LOW);
  }
}

// Interrupt on encoder changing state
void doEncoder() {
  // debounce
  if ( rotating ) delay (1);  // wait a little until the bouncing is done
    
  // Test transition, did things really change?
  if ( digitalRead(encoderPin) != set ) { // debounce once more
  set = !set;
    
  encoderPos += 1;
//  Serial.println(encoderPos);
     
  rotating = false;  // no more debouncing until loop() hits again
  }
}

/** @brief Callback for when the master transmits data */
void receiveEvent(void){
  digitalWrite(unoBusy, HIGH);
  newCommand = true;
  command = Wire.read(); // Receive byte
//  Serial.print("command: ");
//  Serial.println(command);
}

/** @brief Callback for when the master requests data */
void requestEvent(void){
  if (isRunning){
    if (encoderPos == 1){
      Wire.write(0);
      Wire.write(0);
    } else {
      Wire.write(encoderPos>>8);
      Wire.write(encoderPos);
    }
  } else {
//    Serial.print("timer: ");
//    Serial.println(timer);
    Wire.write(timer/1000);
  }
}
