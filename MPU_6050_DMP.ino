//This program is meant to test and control the MPU6050 chip on the robot, Fetch with DMP
//The MPU6050 is a chip used to measure the acceleration and the orientation of the robot

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h" //contains a class called MPU6050 that sends and receives data to and from the sensor

//Creating MPU6050 Object
MPU6050 accelgyro(0x68);

//functions
void Setup_MPU6050();
void Update_MPU6050();

void Setup_MPU6050(){
  Wire.begin(); //start I2C communication & initialize the MPU6050 chip

  //initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  //verify connection
  Serial.println("Testing device connections...");
  if (accelgyro.testConnection()){
    Serial.println("MPU6050 connection successful");
  }
  else{
    Serial.println("MPU6050 connection failed");
  }
}

void Update_MPU6050(){
  int16_t ax, ay, az; //accelerometer values 3-axises
  int16_t gx, gy, gz; //gyroscope values 3-axises

  //read raw accel & gyro values/measurements from MPU6050
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  //display the values, tab seperated
  Serial.print("ax: "); Serial.print(ax); Serial.print("\t");
  Serial.print("ay: "); Serial.print(ay); Serial.print("\t");
  Serial.print("az: "); Serial.print(az); Serial.print("\t");
  Serial.print("gx: "); Serial.print(gx); Serial.print("\t");
  Serial.print("gy: "); Serial.print(gy); Serial.print("\t");
  Serial.println(gz);
  Serial.print("\n");
}


void setup(){
  //Initiate Serial port with 115200 buad rate
  Serial.begin(115200);
  Setup_MPU6050();
}

void loop(){
  Update_MPU6050(); //Update MPU6050
}
