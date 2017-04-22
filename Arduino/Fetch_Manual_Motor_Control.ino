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
#include <Messenger.h>
#include <limits.h>

int InA1 = 7;
int InB1 = 8;

int InA2 = 12;
int InB2 = 13;

int PWM2 = 5;
int PWM1 = 3;  //PWM1 connects to pin 3
//(25% = 64; 50% = 127; 75% = 191; 100% = 255)

float left_motor_speed = 0;
float right_motor_speed = 0;

Messenger MessengerH = Messenger();

//Time stuff>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
unsigned long lastUpdateMicroSec = 0;
unsigned long lastUpdateMSec = 0;
unsigned long currentMicroSec = 0;
unsigned long microSecSinceUpdate = 0;
float secSinceUpdate = 0;
//Time stuff<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

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

  Serial.print("t"); Serial.print("\t");
  Serial.print(lastUpdateMicroSec); Serial.print("\t");
  Serial.print(secSinceUpdate); Serial.print("\n");
}
//TIME FUNCTIONS<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

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

void move_Left_Motor(float input){
  if(input > 0){
    digitalWrite(InA1, HIGH);
    digitalWrite(InB1, LOW);
    analogWrite(PWM1, input);
  }
  else if(input < 0){
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, HIGH);
    analogWrite(PWM1, abs(input));
  }
  else if(input == 0){
    digitalWrite(InA1, HIGH);
    digitalWrite(InB1, HIGH);
  }
}

void move_Right_Motor(float input){
  if(input > 0){
    digitalWrite(InA2, HIGH);
    digitalWrite(InB2, LOW);
    analogWrite(PWM2, input);
  }
  else if(input < 0){
    digitalWrite(InA2, LOW);
    digitalWrite(InB2, HIGH);
    analogWrite(PWM2, abs(input));
  }
  else if(input == 0){
    digitalWrite(InA2, HIGH);
    digitalWrite(InB2, HIGH);
  }
}

void update_Motors(){
  move_Right_Motor(right_motor_speed);
  move_Left_Motor(left_motor_speed);

  Serial.print("m"); Serial.print("\t");
  Serial.print(left_motor_speed); Serial.print("\t");
  Serial.print(right_motor_speed); Serial.print("\t");
  Serial.print("\n");
}

//MOTOR FUNCTIONS<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

void set_Speed(){
  left_motor_speed = MessengerH.readLong();
  right_motor_speed = MessengerH.readLong();  
}

void(*Reset)(void) = 0;

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

void setup(){
  Serial.begin(9600);
  pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(InA2, OUTPUT);
  pinMode(InB2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  MessengerH.attach(msgCompleted);
  Serial.println("Setup Complete for Fetch_Manual_Motor_Control");

  Serial.println("Controls:");
  Serial.println("-----------------------------------");
  Serial.println("Moving around:");
  Serial.println("   q    w    e"); //q angle turn left, w forward, e angle turn right
  Serial.println("   a    s    d"); //a point turn left, s stop, d point turn right
  Serial.println("   z    x    c"); //z angle turn back left, x backward, c angle turn back right
  Serial.println("o/p : increase/decrease max speeds by 10% (25.5)");
  Serial.println("k : force stop");
  Serial.println("Motor Speed ranges from 0 - 255");
}

void loop(){
  char ch = "";
  int power = 50; //default speed (safe speed)
  while(Serial.available()){
    ch = Serial.read();
    if(ch == 'q'){
      //angle turn left
      move_Right_Motor(power);
      move_Left_Motor(0);
      delay(50);
      break; //this "should" prevent the robot to continue moving after each serial input
    }
    if(ch == 'w'){
      //forward
      move_Right_Motor(power);
      move_Left_Motor(power);
      delay(50);
      break;
    }
    if(ch == 'e'){
      //angle turn right
      move_Right_Motor(0);
      move_Left_Motor(power);
      delay(50);
      break;
    }
    if(ch == 'a'){
      //point turn left
      move_Right_Motor(power);
      move_Left_Motor(-power);
      delay(50);
      break;
    }
    if(ch == 's'){
      //stop
      move_Right_Motor(0);
      move_Left_Motor(0);
      delay(50);
      break;
    }
    if(ch == 'd'){
      //point turn right
      move_Right_Motor(-power);
      move_Left_Motor(power);
      delay(50);
      break;
    }
    if(ch == 'z'){
      //angle turn back left
      move_Right_Motor(-power);
      move_Left_Motor(0);
      delay(50);
      break;
    }
    if(ch == 'x'){
      //backwards
      move_Right_Motor(-power);
      move_Left_Motor(-power);
      delay(50);
      break;
    }
    if(ch == 'c'){
      //angle turn back right
      move_Right_Motor(0);
      move_Left_Motor(-power);
      delay(50);
      break;
    }
    if(ch == 'o'){
      //increase speed by 10%
      power += 25.5;
      break;
    }
    if(ch == 'p'){
      //decrease speed by 10%
      power -= 25.5;
      break;
    }
    if(ch == 'k'){
      //force stop
      move_Right_Motor(0);
      move_Left_Motor(0);
      delay(50);
      break;
    }
  }
}

