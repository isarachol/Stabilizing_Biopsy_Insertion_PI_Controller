/*
  MPU6050 DMP6

  Digital Motion Processor or DMP performs complex motion processing tasks.
  - Fuses the data from the accel, gyro, and external magnetometer if applied, 
  compensating individual sensor noise and errors.
  - Detect specific types of motion without the need to continuously monitor 
  raw sensor data with a microcontroller.
  - Reduce workload on the microprocessor.
  - Output processed data such as quaternions, Euler angles, and gravity vectors.

  The code includes an auto-calibration and offsets generator tasks. Different 
  output formats available.

  This code is compatible with the teapot project by using the teapot output format.

  Circuit: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
  depends on the MPU6050's INT pin being connected to the Arduino's
  external interrupt #0 pin.

  The teapot processing example may be broken due FIFO structure change if using DMP
  6.12 firmware version. 
    
  Find the full MPU6050 library documentation here:
  https://github.com/ElectronicCats/mpu6050/wiki

*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Stepper.h>
#include <LiquidCrystal_I2C.h>

// RED LED Warning
const int led_pin = 5; //change this

// LCD screen
LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x20 for a 16 chars and 2 line display

/* MPU6050 default I2C address is 0x68*/
MPU6050 mpu_need; //needle
MPU6050 mpu_hand(0x69); //handle
MPU6050 mpu[2] = {mpu_need, mpu_hand};
const int mpuNeedId = 0;
const int mpuHandId = 1;
/* OUTPUT FORMAT DEFINITION---------------------------------
- Use "OUTPUT_READABLE_QUATERNION" for quaternion commponents in [w, x, y, z] format. Quaternion does not 
suffer from gimbal lock problems but is harder to parse or process efficiently on a remote host or software 
environment like Processing.
------------------------------------------------------------*/ 
#define OUTPUT_READABLE_YAWPITCHROLL

int const INTERRUPT_PIN = 2;  // Define the interruption #0 pin
bool blinkState;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorFloat gravity;    // [x, y, z]            Gravity vector
float YPR[2][3];

/*---offsets from calibration---*/
const int16_t offset[2][6] = {{-2010, 1864, 1429, -9, -123, -83},{-263, -3120, 799, 64, -9, 12}};

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

//Stepper
const float stepperRatio = 2050.0 / 360.0; // 1° ≈ 5.7 steps
const float STEPS_PER_REV = 32;
const float GEAR_RED = 64;
// Number of steps per geared output rotation
const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_RED;
float stepSize = STEPS_PER_REV; //1/64 rev output
Stepper steppermotor(STEPS_PER_REV, 8, 10, 9, 11);

//FSR
const int FSR_PIN = A0;
const int VCC = 5;
float force = 0;

//----------------------------------------------------make changes here-------------------------------
//Control variables
const float motorLimit = 15; //Depends on design
const float speedMin = 32; // change if you want
float prevYaw = 0.0;
float goalYaw = -52.5;
const float threshold = 2.0;       // Deadband (degrees)
unsigned long currentMillis = millis();
unsigned long lastUpdate = currentMillis;
//const unsigned long updateInterval = 100; // milliseconds
float stepCount = 0;
float revCount = 0;
const float forceLimit = 5; // newtons, change if applicable

// PI controller variables
float Kp = 15.0;  // 1° = ~5.7 steps (matching motor)
float Ki = 0.01;
float integral = 0;
//-----------------------------------------------------------------------------------------------------

//User Feedback
unsigned long currentMillisUser = millis();
unsigned long lastUpdateUser = currentMillisUser;

void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  Serial.begin(115200); //115200 is required for Teapot Demo output
  while (!Serial);

  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu[0].initialize();
  mpu[1].initialize(); // uncomment this
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu[0].testConnection() == false){
    Serial.println("IMU0 Needle connection failed");
    printScreen("IMU connection", "Fail!");
    while(true);
  }
  else {
    Serial.println("IMU0 Needle connection successful"); 
  }
  // uncomment this whole thing
  if(mpu[1].testConnection() == false){
    Serial.println("IMU1 Handle connection failed");
    printScreen("IMU connection", "Fail!");
    while(true);
  }
  else {
    Serial.println("IMU1 Handle connection successful");
  }
  printScreen("IMU connection", "Successful");
  delay(2000);
  printScreen("Calibrating...", "Please wait");
    
  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));

  // calibrate imus
  for (int i=0; i<2; i++) { //change to i<2
    Serial.print("IMU ");
    Serial.println(i);
    devStatus = mpu[i].dmpInitialize();

    /* Supply your gyro offsets here, scaled for min sensitivity */
    mpu[i].setXAccelOffset(offset[i][0]);
    mpu[i].setYAccelOffset(offset[i][1]);
    mpu[i].setZAccelOffset(offset[i][2]);
    mpu[i].setXGyroOffset(offset[i][3]);
    mpu[i].setYGyroOffset(offset[i][4]);
    mpu[i].setZGyroOffset(offset[i][5]);

    /* Making sure it worked (returns 0 if so) */ 
    if (devStatus == 0) {
      mpu[i].CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
      mpu[i].CalibrateGyro(6);
      Serial.println("These are the Active offsets: ");
      mpu[i].PrintActiveOffsets();
      Serial.println(F("Enabling DMP..."));   //Turning ON DMP
      mpu[i].setDMPEnabled(true);

      /*Enable Arduino interrupt detection*/
      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
      Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
      Serial.println(F(")..."));
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
      MPUIntStatus = mpu[i].getIntStatus();

      /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
      Serial.println(F("DMP ready! Waiting for first interrupt..."));
      DMPReady = true;
      packetSize = mpu[i].dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
    } 
    else {
      Serial.print(F("DMP Initialization failed (code ")); //Print the error code
      Serial.print(devStatus);
      Serial.println(F(")"));
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
    }
  
  }
  steppermotor.setSpeed(1000); // Speed in RPM
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(led_pin, OUTPUT);

  getYprAll();
  // If can get ansolute Yaw, but can't
  // while (abs(YPR[mpuNeedId][0]-YPR[mpuNeedId][0]) > threshold) { //change one of them to mpuHandId
  //   controlMotor(YPR[mpuNeedId][0]); //change one of them to mpuHandId
  // }
  // For initial calibration, manual unfortunately
  // while (YPR[mpuNeedId][0]>-30) {
  //   getYprAll();
  //   controlMotor(-30);
  // }
  delay(500);
  lcd.clear();
  printScreen("Done", "");
  delay(1000);
  lcd.clear();
}

void loop() {
  //if (!DMPReady) return; // Stop the program if DMP programming fails.
  getYprAll();
  //Serial.println("Needle - Hand: " + String(YPR[mpuNeedId][0]-YPR[mpuHandId][0]));
  // if (abs(YPR[mpuNeedId][0]-YPR[mpuHandId][0]) < motorLimit) { //if rotation is within range
  //   controlMotor(goalYaw);
  // }
  // else {
  //   if (YPR[mpuNeedId][0]-YPR[mpuHandId][0] < 0){
  //     steppermotor.setSpeed(200);
  //     steppermotor.step(stepSize);
  //   }
  //   else {
  //     steppermotor.setSpeed(200);
  //     steppermotor.step(-stepSize);
  //   }
  // }
  controlMotor(goalYaw);
  Serial.println();
  showUserFeedback();
}

void getYprAll() {
  getYpr(mpuNeedId);
  getYpr(mpuHandId);
}

void getYpr(int mpu_id) {
  float ypr[3];
  ypr[0] = YPR[mpu_id][0]* M_PI/180;
  ypr[1] = YPR[mpu_id][1]* M_PI/180;
  ypr[2] = YPR[mpu_id][2]* M_PI/180;
  /* Read a packet from FIFO */
  if (mpu[mpu_id].dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      /* Display Euler angles in degrees */
      mpu[mpu_id].dmpGetQuaternion(&q, FIFOBuffer);
      mpu[mpu_id].dmpGetGravity(&gravity, &q);
      mpu[mpu_id].dmpGetYawPitchRoll(ypr, &q, &gravity);
    #endif
  YPR[mpu_id][0] = ypr[0]* 180/M_PI;
  YPR[mpu_id][1] = ypr[1]* 180/M_PI;
  YPR[mpu_id][2] = ypr[2]* 180/M_PI;
  }
}

//PI controller
void controlMotor(float goal) {
  lastUpdate = currentMillis;
  currentMillis = millis();
  float timeLapse = currentMillis - lastUpdate;
  //Serial.println("Needle Yaw: " + String(YPR[mpuNeedId][0]));
  float error = goal - YPR[mpuNeedId][0];
  Serial.println("Needle error: " + String(error));

  integral += error * (timeLapse / 1000.0); //error*s
  float speed = (Kp * error) + (Ki * integral);
  Serial.println("Integral part: " + String(Ki * integral));
  Serial.println("Speed: " + String(speed));

  if (abs(speed) < speedMin) {
    Serial.println("Speed too low");
  }
  else {
    float stepSizeAdj = stepSize; //* speedRatio; // same time step
    Serial.println("Control activated, step: " + String(stepSizeAdj));
    if (YPR[mpuNeedId][0]-YPR[mpuHandId][0] < -motorLimit && speed < 0) {continue;}
    else if (YPR[mpuNeedId][0]-YPR[mpuHandId][0] > motorLimit && speed > 0) {continue;}
    else {
      if (speed < 0) { //step backward
        speed = -speed;
        stepSizeAdj = -stepSize;
      }
      steppermotor.setSpeed(speed);
      steppermotor.step(stepSizeAdj);
    }
  }
}

void showUserFeedback() {
  getForceValues();
  //Serial.println("Needle Yaw: " + String(YPR[mpuHandId][0]));
  float errorUser = goalYaw - YPR[mpuHandId][0]; // change to hand
  Serial.println("Error: " + String(errorUser));
  showForceAndError(force, errorUser);
  //warning when force too high
  if (force > forceLimit) {
    digitalWrite(led_pin, HIGH);
  }
  else {
    digitalWrite(led_pin, LOW);
  }
}

void getForceValues() {
  float forceVal = analogRead(FSR_PIN);  //reading force pin
  forceVal = analogRead(FSR_PIN);        //reading force pin
  float forceV = forceVal/1023*VCC;
  force = forceV*10.0;
  delay(1);
}

void showForceAndError(float force, float error) {
  String dir = "";
  if (error > threshold) {
    dir = "^  "; //Tell user to rotate ^
  }
  else if (error < -threshold) {
    dir = "v  "; //Tell user to rotate v
  }
  String line1 = "F (N): " + String(force);
  String line2 = "Err (d): " + String(error) + dir;
  printScreen(line1, line2);
}

void printScreen(String line1, String line2) {
  lcd.setCursor(0,0);
  lcd.print(line1);
  lcd.setCursor(0,1);
  lcd.print(line2);
}
