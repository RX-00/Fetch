#include <Servo.h>

//Intiate the servos on the arm
Servo base, shoulder, elbow, claw;

//variables for the ultrasonic sensor underneath the arm
#define echoPin 22
#define trigPin 24

int maximumRange = 200;
int mininumRange = 0;
long duration, distance;

//variables for the positions of the arm
int pos1=90; //base
int pos2=90; //shoulder
int pos3=90; //elbow
const int clawOpen = 90;
const int clawClose = 115;

//variable for when the arm obtained its object
bool objectObtained = false;

//variable to identify the servos
const int servoBase = 1;
const int servoShoulder = 2;
const int servoElbow = 3;
const int servoClaw = 4;

//variable for serial input from the arm_control_node
int buff[5];
int j = -1;

//initiate the functions before hand
int ultra_sonic_sensor();
void control_arm();
void slow_servo();
void default_positions();

int ultra_sonic_sensor(){
  //The following trigPin/echoPin cycle is used to determine the distance of the nearest object by bouncing soundwaves off of it
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 

  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);

  //Calculate the distance (in cm) based on the speed of sound
  distance = duration/58.2;

  if (distance >= maximumRange || distance <= mininumRange){
    //Send a negative number to computer to indicate there is nothing in range
    Serial.println("-1");
    Serial.println(distance);
  }
  else{
    //Send the distance to the computer using the serial port
    Serial.println(distance);
  }

  return distance;

  //delay for 50 milliseconds
  delay(50);
}

void default_positions(){
  claw.write(90);
  base.write(90);
  elbow.write(90);
  shoulder.write(90);
  delay(50);
}

void slow_servo(int servoPosition, int servo_number_ID){
  /*
   * base = 1
   * shoulder = 2
   * elbow = 3
   * claw = 4
   */
  for(int movement = 0; movement < servoPosition; movement++){
      switch(servo_number_ID){
        case 1:
          base.write(movement);
          delay(10);
          break;
        case 2:
          shoulder.write(movement);
          delay(10);
          break;
        case 3:
          elbow.write(movement);
          delay(10);
          break;
        case 4:
          if(movement < 90){
            claw.write(90);
            delay(5);
          }
          else if(movement > 90){
            claw.write(movement);
            delay(10);
          }
          break;
      }
  }
}

void control_arm(){
  if(ultra_sonic_sensor() == 8){ //grab the object and retract
    claw.write(clawOpen);
    slow_servo(55, servoElbow);
    delay(1000);
    slow_servo(170, servoShoulder);
    delay(1000);
    slow_servo(clawClose, servoClaw);
    delay(1000);
    objectObtained = true;
    delay(1000);
    slow_servo(90, servoShoulder);
    delay(1000);
    slow_servo(90, servoElbow);
    delay(1000);
  }
  else if((objectObtained == false) && (ultra_sonic_sensor() != 8)){
    default_positions();
    objectObtained = false;
  }
  while((objectObtained == true) && (ultra_sonic_sensor() != 8)){
    //just do nothing
  }
}

//since arduino receives the info as characters this is to change the characters back into numbers
int calc()
{
    int num=0,x=0;
 
    for(x;x<=j;x++)
          num=num+(buff[x]-48)*pow(10,j-x);
           
    return num;
}


void setup(){
  //start up serial connection
  Serial.begin(9600);
  Serial.println("Let's Begin\n");

  //attach the servo signals to pins
  base.attach(9);
  shoulder.attach(8);
  elbow.attach(6);
  claw.attach(7);

  //pin modes of the ultrasonic pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop(){
  int num, input;
  if(Serial.available()){
    //If the input is a ',' then go onto the next to calculate the next input from serial
    input = Serial.read();
    if(input==','){
      num = calc();
      j = -1;
      Serial.println(num);
      if(num >= 0 && num <= 14){ //if num is more to the left, meaning the target object is to the left of Fetch's Arm's Camera
        base.write(num * 5);
        delay(500);
      }
      else if(num > 14 && num < 22){ //if num is more to the middle, meaning the target object is to the middle of Fetch's Arm's Camera
        base.write(num * 5);
        delay(500);
        control_arm();
      }
      else if(num >= 22 && num <= 36){ //if num is more to the right, meaning the target object is to the right of Fetch's Arm's Camera
        base.write(num * 5);
        delay(500);
      }
    }
    else{ //move onto the next input from serial
     //j++;
     //buff[j] = input;   
    }
  }
}


