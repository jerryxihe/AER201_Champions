#include "Adafruit_VL53L0X.h"
#include <Servo.h>

#define startWhackerBottom 10
#define startWhackerTop 3
#define whackerPin 9
#define runFinished 11
#define cellPin 12
#define gripperPin 13
#define startServos 14
#define bluetoothStart 15
#define startPollingLaser 16
#define stopMotors 17

#define gripperNeutral 138
#define cellNeutral 146
#define whackerNeutral 127

Adafruit_VL53L0X poleSensor = Adafruit_VL53L0X();

VL53L0X_RangingMeasurementData_t poleMeasure;

volatile unsigned int poleDist = 10000;

Servo gripper;
Servo cell;
Servo whacker;

volatile unsigned int i = 0;
volatile char startByte = 0;

void setup() {
  pinMode(stopMotors, OUTPUT);
  digitalWrite(stopMotors, LOW);
  
  Serial.begin(9600);

  // wait until serial port opens for native USB devices
  while (! Serial) {
    delay(1);
  }
  
  Serial.println("Adafruit VL53L0X test");
  if (!poleSensor.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }

  pinMode(startWhackerBottom, INPUT);

  pinMode(startWhackerTop, INPUT);

  pinMode(startServos, INPUT);
  
  pinMode(startPollingLaser, INPUT);

  pinMode(bluetoothStart, OUTPUT);
  digitalWrite(bluetoothStart, LOW);

  pinMode(runFinished, INPUT);

  whacker.attach(whackerPin);
  whacker.write(whackerNeutral);

  gripper.attach(gripperPin);
  gripper.write(gripperNeutral);
  
  cell.attach(cellPin);
  cell.write(cellNeutral);
}

void loop() {
//  for (i=whackerNeutral;i>59;i--){
//    whacker.write(i);
//    delay(8);
//  }
//  whacker.write(whackerNeutral);
//  while (1){}
  if (digitalRead(runFinished) && Serial.available() > 0){
    startByte = Serial.read();
    if (startByte == '1'){
      digitalWrite(bluetoothStart, HIGH);
      startByte = 0;
      delay(50);
      digitalWrite(bluetoothStart, LOW);
    }
  }
  if (digitalRead(startPollingLaser)){
    digitalWrite(stopMotors, LOW);
    poleDist = 10000;
    while (poleDist > 350 && !digitalRead(runFinished)){
      poleSensor.rangingTest(&poleMeasure, false);
      poleDist = poleMeasure.RangeMilliMeter;
//      Serial.println(poleDist);
      if (poleDist <= 350){
        poleSensor.rangingTest(&poleMeasure, false);
        poleDist = poleMeasure.RangeMilliMeter;
//        delay(12);
//        digitalWrite(stopMotors, HIGH);
//        Serial.println(poleDist);
        if (poleDist <= 350){
          delay(13);
          digitalWrite(stopMotors, HIGH);
        }
      }
    }
  }
  // Operate cell
  if (digitalRead(startServos)){
    cell.write(cellNeutral+20);
    delay(350);
    gripper.write(gripperNeutral+30);
    delay(100);
    cell.write(cellNeutral-90);
    delay(600);
    gripper.write(gripperNeutral);
    delay(100);
    cell.write(cellNeutral);
  }
  if (digitalRead(startWhackerTop)){
    for (i=whackerNeutral;i>59;i--){
      whacker.write(i);
      delay(8);
    }
    whacker.write(whackerNeutral);
  }
  if (digitalRead(startWhackerBottom)){
    for (i=whackerNeutral;i>59;i--){
      whacker.write(i);
      delay(8);
    }
    whacker.write(whackerNeutral);
  }
}
