#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include "pitches.h"

#define returnToStart 2
#define nanoStop 3
#define startRun 4 
#define enL 5
#define enR 6
#define in1 7
#define in2 8
#define in3 9
#define in4 10
#define startDrive 11
#define poleLED 13
#define speakerOn 14

#define sixteenth 90

MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


volatile unsigned char leftPWM = 63;
volatile unsigned char rightPWM = 53;
volatile bool driving = false;
volatile bool runFinished = false;
volatile float factor = 0;
float yawRef = 0.0;
volatile bool sound = false;

void setup() {
  pinMode(poleLED, OUTPUT);
  digitalWrite(poleLED, LOW);

  pinMode(speakerOn, OUTPUT);
  digitalWrite(speakerOn, LOW);
  
  pinMode(returnToStart, INPUT);
  // runFinished pin on interrupt 1 (pin 2)
  attachInterrupt(digitalPinToInterrupt(returnToStart), doReturnToStart, RISING);
  
  pinMode(nanoStop, INPUT);
  // nano pin on interrupt 1 (pin 3)
  attachInterrupt(digitalPinToInterrupt(nanoStop), doNanoStop, RISING);

  pinMode(startRun, INPUT);
  pinMode(startDrive, INPUT);
  
  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  digitalWrite(enL, LOW);
  digitalWrite(enR, LOW);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // ready when buffer empty
  while (Serial.available() && Serial.read()); // empty buffer

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  Serial.println("hi");

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready!"));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop() {
//  digitalWrite(speakerOn, HIGH);
//  tone(15, NOTE_FS4, sixteenth*4);
//  delay(sixteenth*4*1.3);
//  noTone(15);
//  tone(15, NOTE_CS5, sixteenth*2);
//  delay(sixteenth*2*1.3);
//  noTone(15);
//  tone(15, NOTE_AS4, sixteenth*2);
//  delay(sixteenth*2*1.3);
//  noTone(15);
//  tone(15, NOTE_AS4, sixteenth*4);
//  delay(sixteenth*4*1.3);
//  noTone(15);
//  tone(15, NOTE_GS4, sixteenth*2);
//  delay(sixteenth*2*1.3);
//  noTone(15);
//  tone(15, NOTE_FS4, sixteenth*2);
//  delay(sixteenth*2*1.3);
//  noTone(15);
//  while (1){}
//  while (1){
//  tone(15, NOTE_D4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  tone(15, NOTE_D4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  tone(15, NOTE_D5, sixteenth*2);
//  delay(sixteenth*2*1.3);
//  noTone(15);
//  tone(15, NOTE_A4, sixteenth*3);
//  delay(sixteenth*3*1.3);
//  noTone(15);
//  tone(15, NOTE_GS4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  delay(104);
//  tone(15, NOTE_G4, sixteenth*2);
//  delay(sixteenth*2*1.3);
//  noTone(15);
//  tone(15, NOTE_F4, sixteenth*2);
//  delay(sixteenth*2*1.3);
//  noTone(15);
//  tone(15, NOTE_D4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  tone(15, NOTE_F4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  tone(15, NOTE_G4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//
//  tone(15, NOTE_C4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  tone(15, NOTE_C4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  tone(15, NOTE_D5, sixteenth*2);
//  delay(sixteenth*2*1.3);
//  noTone(15);
//  tone(15, NOTE_A4, sixteenth*3);
//  delay(sixteenth*3*1.3);
//  noTone(15);
//  tone(15, NOTE_GS4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  delay(104);
//  tone(15, NOTE_G4, sixteenth*2);
//  delay(sixteenth*2*1.3);
//  noTone(15);
//  tone(15, NOTE_F4, sixteenth*2);
//  delay(sixteenth*2*1.3);
//  noTone(15);
//  tone(15, NOTE_D4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  tone(15, NOTE_F4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  tone(15, NOTE_G4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//
//  tone(15, NOTE_B3, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  tone(15, NOTE_B3, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  tone(15, NOTE_D5, sixteenth*2);
//  delay(sixteenth*2*1.3);
//  noTone(15);
//  tone(15, NOTE_A4, sixteenth*3);
//  delay(sixteenth*3*1.3);
//  noTone(15);
//  tone(15, NOTE_GS4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  delay(104);
//  tone(15, NOTE_G4, sixteenth*2);
//  delay(sixteenth*2*1.3);
//  noTone(15);
//  tone(15, NOTE_F4, sixteenth*2);
//  delay(sixteenth*2*1.3);
//  noTone(15);
//  tone(15, NOTE_D4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  tone(15, NOTE_F4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  tone(15, NOTE_G4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//
//  tone(15, NOTE_AS3, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  tone(15, NOTE_AS3, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  tone(15, NOTE_D5, sixteenth*2);
//  delay(sixteenth*2*1.3);
//  noTone(15);
//  tone(15, NOTE_A4, sixteenth*3);
//  delay(sixteenth*3*1.3);
//  noTone(15);
//  tone(15, NOTE_GS4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  delay(104);
//  tone(15, NOTE_G4, sixteenth*2);
//  delay(sixteenth*2*1.3);
//  noTone(15);
//  tone(15, NOTE_F4, sixteenth*2);
//  delay(sixteenth*2*1.3);
//  noTone(15);
//  tone(15, NOTE_D4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  tone(15, NOTE_F4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//  tone(15, NOTE_G4, sixteenth);
//  delay(sixteenth*1.3);
//  noTone(15);
//}
//  digitalWrite(speakerOn, LOW);
  
  factor = 0.0;
  leftPWM = 63;
  rightPWM = 53;
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
//  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
//    // reset so we can continue cleanly
//    mpu.resetFIFO();
//    fifoCount = mpu.getFIFOCount();
////    Serial.println(F("FIFO overflow!"));
//
//  // otherwise, check for DMP data ready interrupt (this should happen frequently)
//  } else {
    mpu.resetFIFO();
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
        
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.println(ypr[0] * 180/M_PI);
//    Serial.print("\t");
//    Serial.print(ypr[1] * 180/M_PI);
//    Serial.print("\t");
//    Serial.println(ypr[2] * 180/M_PI);
//  }

  if (runFinished){
    digitalWrite(poleLED, LOW);
    digitalWrite(in1, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in4, LOW);
    analogWrite(enL, 70);
    analogWrite(enR, 85);
    delay(1000);
    analogWrite(enL, 160);
    analogWrite(enR, 170);
    delay(1000);
    analogWrite(enL, 250);
    analogWrite(enR, 255);
    while (digitalRead(returnToStart)){
//      Serial.println("stuck");  
    }
    digitalWrite(in2, HIGH);
    digitalWrite(in4, HIGH);
    runFinished = false;
    driving = false;
  }
  if (digitalRead(startRun)){
    yawRef = (ypr[0] * 180/M_PI);
  }
  if (digitalRead(startDrive)){
    digitalWrite(poleLED, LOW);
    driving = true;
    factor = (ypr[0] * 180/M_PI) - yawRef;
    factor = calc(factor);
//    Serial.print("factor:");Serial.println(factor);
    analogWrite(enL, leftPWM + factor);
    analogWrite(enR, rightPWM - factor);
    digitalWrite(in1, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    digitalWrite(in2, HIGH);
  }         
  if (driving){
    factor = (ypr[0] * 180/M_PI) - yawRef;
    factor = calc(factor);
//    if (factor == 0){
//      digitalWrite(poleLED, HIGH);
//    } else {
//      digitalWrite(poleLED, LOW);
//    }
//    Serial.print("factor:");Serial.println(factor);
    analogWrite(enL, leftPWM + factor);
    analogWrite(enR, rightPWM - factor);
//    Serial.print("left:");Serial.println(leftPWM + factor);
//    Serial.print("right:");Serial.println(rightPWM - factor);
  }
  if (sound){
    digitalWrite(speakerOn, HIGH);
    tone(15, NOTE_G5, 250*1.3);
    delay(250*1.3);
    noTone(15);
    tone(15, NOTE_E5, 250*1.3);
    delay(250*1.3);
    noTone(15);
    tone(15, NOTE_C5, 250*1.3);
    delay(250*1.3);
    noTone(15);
    digitalWrite(speakerOn, LOW);
    sound = false;
  }
}

void doReturnToStart(){
  runFinished = true;
}

void doNanoStop(){
//  Serial.print("Nanostop");
  digitalWrite(in1, HIGH);
  digitalWrite(in3, HIGH);
  analogWrite(enL, 255);
  analogWrite(enR, 255);
  digitalWrite(in2, HIGH);
  digitalWrite(in4, HIGH);
  digitalWrite(poleLED, HIGH);
  driving = false;
  sound = true;
}

float calc(float d){
  unsigned char amp = 8;
  unsigned char var = 2;
  unsigned char maxVar = 10;
  if (d < (-360.0+maxVar)){
    if (d < (-360.0+var)){
      //return 0;
      return amp*sqrt((d+360)/var);
    }
    return amp;
    //return map(d,-360.0,-360.0+maxVar,0.0,-amp);
  } else if (d > 360.0-maxVar){
    if (d > (360.0-var)){
      //return 0;
      return -amp*sqrt(-(d-360.0)/var);
    }
    return -amp;
    //return map(d,360.0-maxVar,360.0,amp,0.0);
  } else  if (d > -maxVar&& d < maxVar) {
    if (d > -var && d < var){
      if (d > 0){
        //return 0;
        return -amp*sqrt(d/var);
      } else if (d < 0){
        //return 0;
        return amp*sqrt(-d/var);
      }
    }
    if (d > var){
      return -amp;
    } else if (d < -var){
      return amp;
    }
  } else {
    return 0;
  }
  return 0;
}
