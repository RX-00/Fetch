//This program is meant to test and control the ultrasonic sensor on the robot, Fetch
//The ultrasonic sensor is used to measure the distance in front of Fetch

//Ultrasonic sensor pins definition
#define ECHO 9
#define TRIG 10
long duration, cm;

void SetupUltrasonic();
void Update_Ultrasonic();
long MicrosecondsToCentimeters(long microseconds);

void SetupUltrasonic(){
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

long MicrosecondsToCentimeters(long microseconds){
  /*NOTE:
   *speed of sound = 340 m/s => 29 microseconds per cm
   *ping travels out and back of object, so take half of
   *distance that it travelled
   */
  return microseconds / 29 / 2;
}

void Update_Ultrasonic(){
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  //echo pin for reading signal from PING, reception of the echo off of an object
  duration = pulseIn(ECHO, HIGH);
  cm = MicrosecondsToCentimeters(duration); //convert time into distance

  //print through the serial port
  Serial.print("Ultrasonic"); Serial.print("\t");
  Serial.print(cm); Serial.print("\n");
}

void setup(){
  Serial.begin(115200);
  SetupUltrasonic();
}

void loop(){
  Update_Ultrasonic();
}
