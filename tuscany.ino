#define PIN_LED 13

// MOTORs

#define STEP_MS 25

#define MOTOR_G 0
#define MOTOR_H 1
#define MOTOR_R 2
#define MOTOR_V 3
#define MOTOR_ALL 255

#define DIR_LOOSE 0
#define DIR_TIGHT 1

#define DIR_BACK 0
#define DIR_FRWD 1

#define DIR_UP 0
#define DIR_DN 1

#define DIR_CW 0
#define DIR_CCW 1

#define STOP_REASON_BLOCK 0
#define STOP_REASON_SENS_IR 1
#define STOP_REASON_TIME 2
#define STOP_REASON_GYRO 3
#define STOP_REASON_BUTTON 4

// ULTRASONIC SENSOR

#define PIN_TRIG 7
#define PIN_ECHO 8

// SHIFT REGISTER

#define PIN_SER 13
#define PIN_LATCH 12
#define PIN_CLK 11

// MOTORs PWM and CURRENT SENSORS

uint8_t pinPWM[4] = {5, 6, 9, 10};
uint8_t pinCS[4] = {0, 1, 2, 3}; 

// MOTOR CURRENT MATRIX
// CurrentSensorMatrix Format Example:
// motorCurrentMax[8] = {MOTOR_G__DIR_LOOSE,  MOTOR_G__DIR_TIGHT, 
//                       MOTOR_H__DIR_BACK,  MOTOR_H__DIR_FRWD,
//                       MOTOR_R__DIR_CW, MOTOR_R__DIR__CCW};

uint8_t motorCurrentMax[8] =    {48, 48,    45, 45,   70, 70,   50, 50};

// MOTOR SPEED MATRIX
// SpeedMatrix Format Example:
// motorSpeedStart[8] = {MOTOR_G__DIR_LOOSE, MOTOR_G__DIR_TIGHT,  
//                       MOTOR_H__DIR_BACK,  MOTOR_H__DIR_FRWD,
//                       MOTOR_R__DIR_CW, MOTOR_R__DIR__CCW,
//                       MOTOR_V__DIR_UP,  MOTOR_V__DIR_DN,};

uint8_t motorSpeedStart[8] = { 80,  80,    80,  80,       50,   50,    20, 20};
uint8_t motorSpeedMax[8] =   {120, 120,    110,  110,     80,   60,    40, 40};
uint8_t motorSpeedMin[8] =   {110,  70,    30,  30,       60,   40,    30, 30};
uint8_t motorSpeedInc[4] = {1, 1, 1, 1};
uint8_t motorSpeedDelay[4] = {250, 250, 250, 250};

uint8_t motorDir[4] = {DIR_LOOSE, DIR_BACK, DIR_UP, DIR_CW};
int motorCurrentSensorValue;

#define GYRO_MAX_UP 52
#define GYRO_MAX_DW 10

#define PRECISON 0.20
#define CORRELATION 3

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL


#define LED_PIN 13 
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// GYRO
float startMPUGyroZ = 0; 
float endMPUGyroZ = 0; 
float curMPUGyroZ = 0;
float absTreshold; 
float startTrMPUGyroZ;
float endTrMPUGyroZ;
float mpuGyroX = 0; 
float mpuGyroY = 0; 
float mpuGyroZ = 0; 

boolean startPosition = false;
boolean endPosition = false;

volatile bool mpuInterrupt = false; 

boolean getMPUGyroXYZ ()
{    
      boolean flagReturn = false;
      mpuInterrupt = false;
      mpuIntStatus = mpu.getIntStatus();
      
      // get current FIFO count
      fifoCount = mpu.getFIFOCount();
      
      // check for overflow (this should never happen unless our code is too inefficient)
      if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
       
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
      } else 
      if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) 
             fifoCount = mpu.getFIFOCount();
        Serial.print(F(" FIFO count: "));
        Serial.println(fifoCount);

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.resetFIFO();
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpuGyroX = ypr[0] * 180/M_PI;
        mpuGyroY = ypr[1] * 180/M_PI;
        mpuGyroZ = ypr[2] * 180/M_PI;
        /*
            Serial.print("gyro\t");
            Serial.print(mpuGyroX);
            Serial.print("\t");
            Serial.print(mpuGyroY);
            Serial.print("\t");
            Serial.println(mpuGyroZ);        
         */   
           flagReturn = true;
      }
     return flagReturn;
}


void dmpDataReady() {
      mpuInterrupt = true; 
}




volatile bool flagIntButton = true; 

void intButton() 
{
  Serial.println();
  Serial.println(F(" INT: BUTTON "));
  digitalWrite(PIN_LED, LOW);
  motorOff(MOTOR_ALL, STOP_REASON_BUTTON); 
}

void setup() 
{
  Serial.begin(115200);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_SER, OUTPUT);
  pinMode(PIN_LATCH, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);
 
  attachInterrupt(1, intButton, RISING);
  
   Wire.begin();
    TWBR = 24;

    Serial.begin(115200);

    // initialize device
   // Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
   // Serial.println(F("Testing device connections..."));
   // Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready

   // delay(500);    
    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
      //  Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
      //  Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
      //  Serial.println(F("DMP ready! Waiting for first interrupt..."));
      digitalWrite(PIN_LED, HIGH);
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

 
 // initMotorH();
  initMotorR();
 // testMotorAll();

// trackSurface(160);
}


void loop()
{
//trackSurface(10);
//delay(250);
}


void motorOff(uint8_t motor, uint8_t reason) 
{
 
 digitalWrite(PIN_LATCH, LOW);
 shiftOut(PIN_SER, PIN_CLK, LSBFIRST, B00000000);
 digitalWrite(PIN_LATCH, HIGH);
 analogWrite(pinPWM[MOTOR_G], 0);
 analogWrite(pinPWM[MOTOR_H], 0);
 analogWrite(pinPWM[MOTOR_R], 0);

 Serial.print(F("MOTOR: "));
 if (motor == MOTOR_ALL)
   Serial.print(F("ALL "));
 else
   Serial.print(motor);
   
 switch (reason) 
 {
  case STOP_REASON_BLOCK:
    Serial.println(F(" STOP: BLOCK"));
    break;
  case STOP_REASON_SENS_IR:
    Serial.println(F(" STOP: IR"));
    break;  
  case STOP_REASON_TIME:
    Serial.println(F(" STOP: TIME"));
    break;  
  case STOP_REASON_GYRO:
    Serial.println(F(" STOP: GYRO"));
    break;
  case STOP_REASON_BUTTON:
    Serial.println(F(" STOP: BUTTON"));
    break;
  default:
    Serial.println(F(" STOP: NA"));  
 }
 
}

void setMotorDirection(uint8_t motor, uint8_t motorDir)
{
   uint8_t shiftDir;
   digitalWrite(PIN_LATCH, LOW);
   shiftOut(PIN_SER, PIN_CLK, MSBFIRST, 0);
   digitalWrite(PIN_LATCH, HIGH); 
   
   shiftDir = (1 << ((2 * motor) + motorDir));
   digitalWrite(PIN_LATCH, LOW);
   shiftOut(PIN_SER, PIN_CLK, MSBFIRST, shiftDir);
   digitalWrite(PIN_LATCH, HIGH);  

//   Serial.print(F("  MOTOR: "));
//   Serial.print(motor);
//   Serial.print(F(" SET SHIFT DIR: "));
//   Serial.println(shiftDir);
}


uint8_t motorRun(uint8_t motor, uint8_t curMotorDir, uint8_t curMotorSpeed)
{        
    int motorCurrentSensorValue;   
    Serial.print(F("  MOTOR: "));
    Serial.print(motor);
    if (curMotorDir == DIR_LOOSE)
    {
      Serial.print(F("  DIR: L/B/CW "));
    }
    else
    {
      Serial.print(F("  DIR: T/F/CCW"));
    }
    Serial.print(F("  SPEED: "));
    Serial.print(curMotorSpeed);

    motorCurrentSensorValue = analogRead(pinCS[motor]);
    Serial.print("  CURRENT: ");
    Serial.println(motorCurrentSensorValue);
     
    setMotorDirection(motor, curMotorDir);
    analogWrite(pinPWM[motor], curMotorSpeed);
    delay(STEP_MS);
    return motorCurrentSensorValue;
}

void testMotorAll ()
{

  motorRun(MOTOR_H, DIR_FRWD,  70);
  delay(1000);
  motorOff(MOTOR_ALL, STOP_REASON_TIME);
  delay(100);

  motorRun(MOTOR_G, DIR_LOOSE,  50);
  delay(2000);
  motorOff(MOTOR_ALL, STOP_REASON_TIME);
  delay(100);
  motorRun(MOTOR_G, DIR_TIGHT,  50);
  delay(2000);
  motorOff(MOTOR_ALL, STOP_REASON_TIME);
  delay(500);

  motorRun(MOTOR_R, DIR_CW,  100);
  delay(750);
  motorOff(MOTOR_ALL, STOP_REASON_TIME);
  delay(500);
  motorRun(MOTOR_R, DIR_CCW,  70);
  delay(750);
  motorOff(MOTOR_ALL, STOP_REASON_TIME);
  delay(500);

  motorRun(MOTOR_H, DIR_BACK,  50);
  delay(900);
  motorOff(MOTOR_ALL, STOP_REASON_TIME);
  delay(500);

}


void initMotorR()
{
  boolean startPosition = false;
  Serial.println(F("INIT MOTOR_R"));
  
 // motorRunTimeUSonic(MOTOR_R, DIR_CW, 1500); 
  delay(500);   
   motorRunTimeUSonic(MOTOR_R, DIR_CCW, 3500);

  Serial.print("DONE INIT MOTOR_R");
  delay(1000);   
}

void initMotorH()
{
   
  Serial.println(F("INIT MOTOR_H"));
  
  motorRunTime(MOTOR_H, DIR_FRWD, 100); 
  delay(2000);   

  motorRunTime(MOTOR_H, DIR_BACK, 100); 
  
  Serial.print("DONE INIT MOTOR_H");
  delay(1000);   
}

uint8_t motorRunTime(uint8_t motor, uint8_t motorDir, int runTime)
{
  int i;
  int motorSpeed = motorSpeedStart[2*motor];
  int motorCurrentSensorValue;
  boolean flagStart = true;
  
  for(i=0; i<runTime; i+=STEP_MS)
  {
   
    motorCurrentSensorValue = motorRun(motor, motorDir, motorSpeed);
    if (motorCurrentSensorValue > motorCurrentMax[2*motor])
      {
        motorOff(motor, STOP_REASON_BLOCK);
        return STOP_REASON_BLOCK;
      }  
    if (flagStart && (motorSpeed < motorSpeedMax[2*motor]))
      motorSpeed++;
    else
    {
      motorSpeed = motorSpeedMin[2*motor];
      flagStart = false;
    }
  }
  motorOff(motor, STOP_REASON_TIME); 
  return STOP_REASON_TIME;
}

long usonicDistanceCm()
{
  long duration;
  long distanceCm;
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  pinMode(PIN_ECHO, INPUT);
  duration = pulseIn(PIN_ECHO, HIGH);
  distanceCm = (duration/2) / 29.1;
  return distanceCm;
}

uint8_t motorRunUSonic(uint8_t motor, uint8_t curMotorDir, uint8_t curMotorSpeed)
{        
    int motorCurrentSensorValue; 
    long usonicDistanceValue;  
    Serial.print(F("  MOTOR: "));
    Serial.print(motor);
    if (curMotorDir == DIR_LOOSE)
    {
      Serial.print(F("  DIR: L/B/CW "));
    }
    else
    {
      Serial.print(F("  DIR: T/F/CCW"));
    }
    Serial.print(F("  SPEED: "));
    Serial.print(curMotorSpeed);

    motorCurrentSensorValue = analogRead(pinCS[motor]);
    Serial.print("  CURRENT: ");
    Serial.print(motorCurrentSensorValue);
    
    usonicDistanceValue = usonicDistanceCm();
    
    Serial.print("  USONIC: ");
    Serial.println(usonicDistanceValue);
     
    setMotorDirection(motor, curMotorDir);
    analogWrite(pinPWM[motor], curMotorSpeed);
    delay(STEP_MS);
    return motorCurrentSensorValue;
}

uint8_t motorRunTimeUSonic(uint8_t motor, uint8_t motorDir, int runTime)
{
  int i;
  int motorSpeed = motorSpeedStart[2*motor];
  boolean flagStart = true;
  
  for(i=0; i<runTime; i+=STEP_MS)
  {
   
    motorCurrentSensorValue = motorRunUSonic(motor, motorDir, motorSpeed);
    if (motorCurrentSensorValue > motorCurrentMax[2*motor])
      {
        motorOff(motor, STOP_REASON_BLOCK);
        return STOP_REASON_BLOCK;
      }  
    if (flagStart && (motorSpeed < motorSpeedMax[2*motor]))
      motorSpeed++;
    else
    {
      motorSpeed = motorSpeedMin[2*motor];
      flagStart = false;
    }
  }
  motorOff(motor, STOP_REASON_TIME); 
  return STOP_REASON_TIME;
}

void trackSurface(int angle)
{

  // Track surface look most higher point on it
  // Rotate while Angle

  boolean flagStart = true;
  boolean flagStartPosition = true;
  long lowUSonicDistanceCm;
  long curUSonicDistanceCm;
  float startMPUGyroX;
  float endMPUGyroX;
  float curMPUGyroX;
  float lowMPUGyroX;
  int i;
  uint8_t motorSpeed;
  uint8_t motor = MOTOR_R;
  
  // Save init gyro position
  
  if (mpuInterrupt && flagStartPosition)
    {
      while(!getMPUGyroXYZ());
      startMPUGyroX = mpuGyroX;
      if (startMPUGyroX != 0)
        {
          flagStartPosition = false;
          Serial.print(F("  startMpuGyroX = "));
          Serial.println(startMPUGyroX);
        }
    }

  // End gyro Position
  endMPUGyroX = startMPUGyroX + angle;
  if (endMPUGyroX > 180) 
    endMPUGyroX = endMPUGyroX - 360;
  Serial.print(F("  endMpuGyroX = "));
  Serial.print(endMPUGyroX);
  
  // Save cur_usonicDistanceCm
   lowUSonicDistanceCm = usonicDistanceCm();
   Serial.print(F("  LowUSonic = "));
   Serial.println(lowUSonicDistanceCm);
   

  // RUN MOTOR_R CW till endMPUGyroX
  motorSpeed = motorSpeedStart[2*motor];

  curMPUGyroX = startMPUGyroX;
  lowMPUGyroX = startMPUGyroX;
  
  while(curMPUGyroX < endMPUGyroX)
  {
     if (mpuInterrupt)
      {
        while(!getMPUGyroXYZ());
        curMPUGyroX = mpuGyroX;
        if (startMPUGyroX != 0)
          {
            Serial.print(F("  curMpuGyroX = "));
            Serial.print(curMPUGyroX);
          }
      }  

  // Read gyro_x and usonicDistanceCm

   curUSonicDistanceCm = usonicDistanceCm();
   Serial.print(F("  curUSonic = "));
   Serial.println(curUSonicDistanceCm);
   
   // Find lowest_usonicDistanceCm
   if  (curUSonicDistanceCm < lowUSonicDistanceCm)
    {
      lowUSonicDistanceCm = curUSonicDistanceCm;
      lowMPUGyroX = curMPUGyroX;
    }
   
    motorCurrentSensorValue = motorRun(motor, DIR_CW, motorSpeed);
    if (motorCurrentSensorValue > motorCurrentMax[2*motor])
      {
        motorOff(motor, STOP_REASON_BLOCK);
        return STOP_REASON_BLOCK;
      }  
    if (flagStart && (motorSpeed < motorSpeedMax[2*motor]))
      motorSpeed++;
    else
    {
      motorSpeed = motorSpeedMin[2*motor];
      flagStart = false;
    }
  }
  motorOff(motor, STOP_REASON_GYRO); 
   Serial.print(F("  lowUSonicDistanceCm = "));
   Serial.print(lowUSonicDistanceCm);
   Serial.print(F("  lowMPUGyroX = "));
   Serial.println(lowMPUGyroX);
    
   delay(1500);
   
  // RUN MOTOR_R CCW till lowest_gyro_x

  while(curMPUGyroX > (lowMPUGyroX + 15))
  {
     if (mpuInterrupt)
      {
        while(!getMPUGyroXYZ());
        curMPUGyroX = mpuGyroX;
        if (startMPUGyroX != 0)
          {
            Serial.print(F("  curMpuGyroX = "));
            Serial.print(curMPUGyroX);
          }
      }  
   
    motorCurrentSensorValue = motorRun(motor, DIR_CCW, motorSpeed);
    if (motorCurrentSensorValue > motorCurrentMax[2*motor+1])
      {
        motorOff(motor, STOP_REASON_BLOCK);
        return STOP_REASON_BLOCK;
      }  
    if (flagStart && (motorSpeed < motorSpeedMax[2*motor+1]))
      motorSpeed++;
    else
    {
      motorSpeed = motorSpeedMin[2*motor+1];
      flagStart = false;
    }
  }
  motorOff(motor, STOP_REASON_GYRO);
  
}

