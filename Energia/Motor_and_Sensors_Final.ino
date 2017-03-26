//This program is meant to test and control the motors and sensors on the robot, Fetch
/*
TRUTH TABLE OF THE INPUT AND OUTPUT COMBOS
------------------------------------------

INA | INB | DIAGA/ENA | DIAGB/ENB | OUTA | OUTB |         CS        | Operating Mode
---------------------------------------------------------------------------------------------
  1 |  1  |     1     |     1     |  H   |  H   |      High Imp     | Brake to Vcc
---------------------------------------------------------------------------------------------
  1 |  0  |     1     |     1     |  H   |  L   |  Isense = Iout/K  | Clockwise (CW)
---------------------------------------------------------------------------------------------
  0 |  1  |     1     |     1     |  L   |  H   |  Isense = Iout/K  | Counterclockwise (CWCW)
---------------------------------------------------------------------------------------------
  0 |  0  |     1     |     1     |  L   |  L   |      High Imp     | Braker to GND
---------------------------------------------------------------------------------------------
*/

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Messenger.h>
#include <limits.h>


//Messenger stuff>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// Messenger Object
Messenger MessengerH = Messenger();
//Messenger stuff<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//Time stuff>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
unsigned long lastUpdateMicroSec = 0;
unsigned long lastUpdateMSec = 0;
unsigned long currentMicroSec = 0;
unsigned long microSecSinceUpdate = 0;
float secSinceUpdate = 0;
//Time stuff<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//Motor stuff>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// Motor pin definitions
// Right Motor Pins
#define INA_2 5
#define INB_2 6
#define PWM_2 36

// Left Motor Pins
#define INA_1 12
#define INB_1 13
#define PWM_1 35

float left_motor_speed = 0;
float right_motor_speed = 0;
//Motor stuff<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//Encoder stuff>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// Right Encoder Pins
#define Right_Encoder_PinA 33
#define Right_Encoder_PinB 34

// Left Encoder Pins
#define Left_Encoder_PinA 31
#define Left_Encoder_PinB 32

//Encoder ticks
volatile long Right_Encoder_Ticks = 0;
volatile long Left_Encoder_Ticks = 0;

//Variables to read the current state of the encoder pins
//NOTE: they are volatile (stored in RAM) due to the encoder values changing quickly
volatile bool RightEncoderBSet;
volatile bool LeftEncoderBSet;
//Encoder stuff<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//IMU stuff>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// Creating MPU6050 Object
MPU6050 imu(0x68);

//Initialize and Declare Variables to handle DMP (digital motion processing)
// DMP optins, true if initialization success
bool dmpReady = false;

// Holds actual interrupt status byte from MPU
uint8_t mpuIntStatus;

// Return status after each operation of the device
uint8_t devStatus;

// Expected packet size of DMP
uint16_t packetSize;

// Count of all the bytes currently in FIFO
uint16_t fifoCount;

// FIFO storage buffer
uint8_t fifoBuffer[64];

// Output format in quaternion
#define OUTPUT_READABLE_QUATERNION

// quaternion variable
Quaternion q;

// Interrupt service routine, called when MPU6050 INT pin generates an interrupt
volatile bool mpuInterrupt = false; //Intterupt detection routine for DMP handling
//IMU stuff<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//Ultrasonic stuff>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// Ultrasonic sensor pins definition
const int ECHO = 30, TRIG = 29;
long duration, cm;
//Ultrasonic stuff<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//MOTOR FUNCTIONS>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void setup_Motors();
void move_forward();
void move_left();
void move_right();
void move_backwards();
void stop_movement();

void set_Speed();
void update_Motors();
void move_Right_Motor();
void move_Left_Motor();

void set_Speed(){
  left_motor_speed = MessengerH.readLong();
  right_motor_speed = MessengerH.readLong();  
}

void move_Right_Motor(float input){
  if (input > 0){
    //Right Motor
    digitalWrite(INA_1, HIGH); 
    digitalWrite(INB_1, LOW);  
    analogWrite(PWM_1, input);
  } 
  else if (input < 0){
    digitalWrite(INA_1, LOW);
    digitalWrite(INB_1, HIGH);
    analogWrite(PWM_1, abs(input));
  }
  else if (input == 0){
    digitalWrite(INA_1, HIGH);
    digitalWrite(INB_1, HIGH);
  }
}

void move_Left_Motor(float input){
  if (input > 0){
    digitalWrite(INA_2, LOW);
    digitalWrite(INB_2, HIGH);
    analogWrite(PWM_2, input);
  }
  else if (input < 0){
    digitalWrite(INA_2, LOW);
    digitalWrite(INB_2, HIGH);
    analogWrite(PWM_2, abs(input));
  }
  else if (input == 0){
    digitalWrite(INA_2, HIGH);
    digitalWrite(INB_2, HIGH);
  }
}

void update_Motors(){
  move_Right_Motor(right_motor_speed);
  move_Left_Motor(left_motor_speed);

  Serial.print("Motors: "); Serial.print("\t");
  Serial.print(left_motor_speed); Serial.print("\t");
  Serial.print(right_motor_speed); Serial.print("\t");
  Serial.print("\n");
}

// Setting the CW rotation to and the Left Motor and CCW to Right Motor
// NOTE: When you call any movement function you have to have delay(); after it for it to move ____ milliseconds

void setup_Motors(){
  Serial.println("Setting up Motor Pins to OUTPUT...");
  //Set the Right Motor Pins as OUTPUT
  pinMode(INA_2, OUTPUT);
  pinMode(INB_2, OUTPUT);
  pinMode(PWM_2, OUTPUT);

  //Set the Left Motor Pins as OUTPUT
  pinMode(INA_1, OUTPUT);
  pinMode(INB_1, OUTPUT);
  pinMode(PWM_1, OUTPUT);
  Serial.println("Set up Motor Pins to OUTPUT");
}

void move_forward(){
  // Right Motor
  digitalWrite(INA_2, HIGH);
  digitalWrite(INB_2, LOW);
  analogWrite(PWM_2, 255);

  // Left Motor
  digitalWrite(INA_1, HIGH);
  digitalWrite(INB_1, HIGH);
  analogWrite(PWM_1, 0);
}

void move_left(){
  // Right Motor
  digitalWrite(INA_2, HIGH);
  digitalWrite(INB_2, LOW);
  analogWrite(PWM_2, 255);

  // Left Motor
  digitalWrite(INA_1, HIGH);
  digitalWrite(INB_1, HIGH);
  analogWrite(PWM_1, 255);
}

void move_right(){
  // Right Motor
  digitalWrite(INA_2, HIGH);
  digitalWrite(INB_2, HIGH);
  analogWrite(PWM_2, 255); //255 for full speed/power, TODO: figure out if it controls power or speed

  // Left Motor
  digitalWrite(INA_1, HIGH);
  digitalWrite(INB_1, HIGH);
  analogWrite(PWM_1, 0);
}

void move_backwards(){
  // Right Motor
  digitalWrite(INA_2, LOW);
  digitalWrite(INB_2, HIGH);
  analogWrite(PWM_2, 0);

  // Left Motor
  digitalWrite(INA_1, HIGH);
  digitalWrite(INB_1, LOW);
  analogWrite(PWM_1, 255);
}

void stop_movement(){
  // Right Motor
  digitalWrite(INA_2, HIGH);
  digitalWrite(INB_2, HIGH);
  analogWrite(PWM_2, 0);

  // Left Motor
  digitalWrite(INA_1, HIGH);
  digitalWrite(INB_1, HIGH);
  analogWrite(PWM_1, 0);
}
//MOTOR FUNCTIONS<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//TIME FUNCTIONS>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void update_Time();

void update_Time(){
  currentMicroSec = micros();
  lastUpdateMSec = millis();
  microSecSinceUpdate = currentMicroSec - lastUpdateMicroSec;
  if (microSecSinceUpdate < 0){
    microSecSinceUpdate = INT_MIN - lastUpdateMicroSec + currentMicroSec;
  }
  lastUpdateMicroSec = currentMicroSec;
  secSinceUpdate = microSecSinceUpdate / 1000000.0;

  Serial.print("Time: "); Serial.print("\t");
  Serial.print(lastUpdateMicroSec); Serial.print("\t");
  Serial.print(secSinceUpdate); Serial.print("\n");
}
//TIME FUNCTIONS<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//MESSENGER FUNCTIONS>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void msgCompleted();
void setup_Reset();
void Reset();
void read_From_Serial();

#define RESET_PIN PB_2 //if this pin is set to high then the Tiva C resets

void setup_Reset(){
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
}

void Reset(){
  digitalWrite(GREEN_LED, HIGH); //apparently GREEN_LED doesn't have to be declared lol
  delay(1000);
  digitalWrite(RESET_PIN, LOW);
  digitalWrite(GREEN_LED, LOW);
}

void msgCompleted(){
  
  char reset[] = "r";
  char set_speed[] = "s";
  
  if(MessengerH.checkString(reset)){
     Serial.println("Reset Done"); 
     Reset();
  }
  if(MessengerH.checkString(set_speed)){
     //This will set the speed
     set_Speed();
     return;   
  }  
}

void read_From_Serial(){
  while (Serial.available() > 0){
    int data = Serial.read();
    MessengerH.process(data);  
  }
}
//MESSENGER FUNCTIONS<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//ENCODER FUNCTIONS>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void setup_Encoders();
void update_Encoders();
void read_Right_Encoder();
void read_Left_Encoder();

// NOTE: These encoders are quadrature encoders

void setup_Encoders(){
  // Right Encoder
  Serial.println("Setting up Right Encoder Pins...");
  pinMode(Right_Encoder_PinA, INPUT_PULLUP); //pin A input
  pinMode(Right_Encoder_PinB, INPUT_PULLUP); //pin B input
  Serial.println("Set up Right Encoder Pins");
  
  // Left Encoder
  Serial.println("Setting up Left Encoder Pins...");
  pinMode(Left_Encoder_PinA, INPUT_PULLUP); //pin A input
  pinMode(Left_Encoder_PinB, INPUT_PULLUP); //pin B input
  Serial.println("Set up Left Encoder Pins");

  // Configure one of the encoder pins as an interrupt when a rise in pulse is detected
  attachInterrupt(Right_Encoder_PinA, read_Right_Encoder, RISING);

  // Configure one of the encoder pins as an interrupt when a rise in pulse is detected
  attachInterrupt(Left_Encoder_PinA, read_Left_Encoder, RISING);

  Serial.println("Setup Encoders Complete");
}

void update_Encoders(){
  Serial.print("Encoder: ");
  Serial.print("\t");
  Serial.print(Left_Encoder_Ticks);
  Serial.print("\t");
  Serial.print(Right_Encoder_Ticks);
  Serial.print("\n");
}

void read_Right_Encoder(){
  // Read the input pin
  RightEncoderBSet = digitalRead(Right_Encoder_PinB);
  Right_Encoder_Ticks += RightEncoderBSet ? -1 : +1;
}

void read_Left_Encoder(){
  // Read the input pin
  LeftEncoderBSet = digitalRead(Left_Encoder_PinB);
  Left_Encoder_Ticks += LeftEncoderBSet ? -1 : +1;
}
//ENCODER FUNCTIONS<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//ULTRASONIC FUNCTIONS>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void setup_Ultrasonic();
void update_Ultrasonic();
long MicrosecondsToCentimeters(long microseconds);

void setup_Ultrasonic(){
  Serial.println("Setting up Ultrasonic sensor...");
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  Serial.println("Setup Ultrasonic Complete");
}

void update_Ultrasonic(){
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  //echo pin for reading signal from PING, reception of the echo off of an object
  duration = pulseIn(ECHO, HIGH);
  cm = MicrosecondsToCentimeters(duration); //convert time into distance

  //print through the serial port
  Serial.print("Ultrasonic: "); Serial.print("\t");
  Serial.print(cm); Serial.print("\n");
}

long MicrosecondsToCentimeters(long microseconds){
  /*NOTE:
   *speed of sound = 340 m/s => 29 microseconds per cm
   *ping travels out and back of object, so take half of
   *distance that it travelled
   */
  return microseconds / 29 / 2;
}
//ULTRASONIC FUNCTIONS<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


//MPU6050 DMP FUNCTIONS>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
void dmpDataRead();
void setup_MPU6050_DMP();
void update_MPU6050_DMP();
void setup_MPU6050();

// indicates whether MPU interrupt pin has gone high
void dmpDataReady(){
  mpuInterrupt = true;
}

void setup_MPU6050(){
  Serial.println("Start I2C communication & initialize MPU6050 chip");
  Wire.begin(); //start I2C communication & initialize the MPU6050 chip

  //initialize device
  Serial.println("Initializing I2C devices...");
  imu.initialize();

  //verify connection
  Serial.println("Testing device connections...");
  
  if (imu.testConnection()){
    Serial.println("MPU6050 connection successful");
  }
  else{
    Serial.println("MPU6050 connection failed");
  }
  
}

void setup_MPU6050_DMP(){
  Serial.println("Setting up MPU6050 DMP...");
  //DMP initialization
  devStatus = imu.dmpInitialize();
  //set offset in three axis
  imu.setXGyroOffset(220);
  imu.setXGyroOffset(76);
  imu.setXGyroOffset(-85);
  imu.setXGyroOffset(1788);
  // When data is ready on MPU6050 buffer then an interrupt will be generated, which read values from the bus
  if(devStatus == 0){
    imu.setDMPEnabled(true);
    pinMode(PUSH2, INPUT_PULLUP);
    attachInterrupt(PUSH2, dmpDataReady, RISING);
    mpuIntStatus = imu.getIntStatus();
    dmpReady = true;
    packetSize = imu.dmpGetFIFOPacketSize();  
  }
  else{
    ; // Do nothing
  }
  Serial.println("Setup MPU6050 DMP");
}

// Reads from the FIFO (first in, first out) data buffer register of MPU6050 and the quaternion value gets printed on serial
void update_MPU6050_DMP(){
    //DMP (Digital Motion Processing) Processing
    if (!dmpReady) return;
    while (!mpuInterrupt && fifoCount < packetSize){
      ; // Do Nothing
    }

    mpuInterrupt = false;
    mpuIntStatus = imu.getIntStatus();

    fifoCount = imu.getFIFOCount(); // Get current FIFO count

    if ((mpuIntStatus & 0x10) || fifoCount > 512){
      imu.resetFIFO(); // Reset to clean the buffer
    }
    else if (mpuIntStatus & 0x02){
      // Short wait for correct available data length
      while (fifoCount < packetSize) fifoCount = imu.getFIFOCount();
      imu.getFIFOBytes(fifoBuffer, packetSize); // Read a packet from FIFO

      // Track FIFO count in case > 1 packet available, this lets immediate read more without waiting for an interrupt
      fifoCount -= packetSize;

      #ifdef OUTPUT_READABLE_QUATERNION

      // Display quaternion values in matrix form: w x y z
      imu.dmpGetQuaternion(&q, fifoBuffer);

      Serial.print("i"); Serial.print("\t");
      Serial.print(q.x); Serial.print("\t");
      Serial.print(q.y); Serial.print("\t");
      Serial.print(q.z); Serial.print("\t");
      Serial.print(q.w);
      Serial.print("\n");
      #endif
    }
}
//MPU6050 DMP FUNCTIONS<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<


void setup(){
  Serial.begin(9600);
  Serial.println("Setting up motors and sensors...");
  setup_Reset();
  setup_Motors();
  setup_Encoders();
  setup_Ultrasonic();
  setup_MPU6050();
  setup_MPU6050_DMP();
  // Setup Messenger Object Handler
  MessengerH.attach(msgCompleted);
  Serial.println("Setup complete");
}

void loop(){
  
  read_From_Serial();
  update_Time();
  update_Encoders();
  update_Ultrasonic();
  //update_Motors();
  update_MPU6050_DMP();
  

  //TODO: FIX THE GODDAMN MOTORS
  //move_Right_Motor(255);
  //delay(1000);
  //move_Left_Motor(255);
  //delay(5000);
}
