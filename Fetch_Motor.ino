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


/*--------------------------------------
  TODO: Check your pins in the program
  to the real life circuit and make
  sure everything matches

  August 5, 2016
  -Roy Xing
----------------------------------------
  Everything works now with the battery
  working full force
  the launchpad still is supplying the
  third power strip with its own power,
  but the ground is universal (shared)
  and things appear to be working fine

  However, the right motor is not
  responding, probably a wiring issue

  November 11, 2016
  -Roy Xing
----------------------------------------
  Fixed, the motor driver's soldering
  job was just iffy (the pins not always
  contacted to the PCB)

  November 13, 2016
  -Roy Xing
 --------------------------------------*/


//Motor Pins Definitions
// Right Motor Pins
#define INA_2 5
#define INB_2 6
#define PWM_2 /*PC_5*/ 36

// Left Motor Pins
#define INA_1 12
#define INB_1 13
#define PWM_1 /*PC_6*/ 35

//Enconder Pins Definitions
//Right encoder
#define Right_Encoder_PinA 33
#define Right_Encoder_PinB 34

//Left encoder
#define Left_Encoder_PinA 31
#define Left_Encoder_PinB 32

//Global Variables
//Encoder ticks
volatile long Right_Encoder_Ticks = 0;
volatile long Left_Encoder_Ticks = 0;

//Variables to read the current state of the encoder pins
volatile bool RightEncoderBSet;
volatile bool LeftEncoderBSet;
//NOTE: they are volatile (stored in RAM) due to the encoder values changing quickly

//initiate the functions before hand
void move_forward();
void move_left();
void move_right();
void move_backwards();
void stop();
void set_up_encoders();
void update_encoders();
void do_Right_Encoder();
void do_Left_Encoder();

// Function to move forward
void move_forward(){
  //Setting the CW rotation to and the Left Motor and CCW to Right Motor
  
  //Right Motor
  digitalWrite(INA_2, LOW);
  digitalWrite(INB_2, HIGH);
  analogWrite(PWM_2, 255);
  
  //Left Motor
  digitalWrite(INA_1, HIGH);
  digitalWrite(INB_1, LOW);
  analogWrite(PWM_1, 255); //255 for full speed
}

// Function to move left
void move_left(){
  //Right Motor
  digitalWrite(INA_2, HIGH);
  digitalWrite(INB_2, LOW);
  analogWrite(PWM_2, 255);
  
  //Left Motor
  digitalWrite(INA_1, HIGH);
  digitalWrite(INB_1, HIGH);
  analogWrite(PWM_1, 0);
}

// Function to move right
void move_right(){
  //Right Motor
  digitalWrite(INA_2, HIGH); 
  digitalWrite(INB_2, HIGH); 
  analogWrite(PWM_2, 0);
  
  //Left Motor
  digitalWrite(INA_1, HIGH); 
  digitalWrite(INB_1, LOW);
  analogWrite(PWM_1, 255);
}

// Function to move backwards
void move_backwards(){
  //Right Motor
  digitalWrite(INA_2, HIGH); 
  digitalWrite(INB_2, LOW); 
  analogWrite(PWM_2, 255);
  
  //Left Motor
  digitalWrite(INA_1, LOW); 
  digitalWrite(INB_1, HIGH); 
  analogWrite(PWM_1, 255);
}

// Function to stop
void stop(){
    //Right Motor
  digitalWrite(INA_2, HIGH); 
  digitalWrite(INB_2, HIGH); 
  analogWrite(PWM_2, 0);
  
  //Left Motor
  digitalWrite(INA_1, HIGH); 
  digitalWrite(INB_1, HIGH); 
  analogWrite(PWM_1, 0);  
}

// Function to set up the encoders
void set_up_encoders(){
  //Note: These encoders are quadrature encoders

  //Right Encoder
  pinMode(Right_Encoder_PinA, INPUT_PULLUP); //pin A input
  pinMode(Right_Encoder_PinB, INPUT_PULLUP); //pin B input

  //Left Encoder
  pinMode(Left_Encoder_PinA, INPUT_PULLUP); //pin A input
  pinMode(Left_Encoder_PinB, INPUT_PULLUP); //pin B input

  //configure one of the encoder pins as an interrupt when a rise in pulse is detected
  attachInterrupt(Right_Encoder_PinA, do_Right_Encoder, RISING);

  //configure one of the encoder pins as an interrupt when a rise in pulse is detected
  attachInterrupt(Left_Encoder_PinA, do_Left_Encoder, RISING);
}

// Function to print the updated ticks of the encoders to the serial monitor
void update_encoders(){
  Serial.print(Left_Encoder_Ticks);
  Serial.print("\t");
  Serial.print(Right_Encoder_Ticks);
  Serial.print("\n");
}

void do_Right_Encoder(){
  //Read the input pin
  RightEncoderBSet = digitalRead(Right_Encoder_PinB);
  Right_Encoder_Ticks += RightEncoderBSet ? -1 : +1;

  /*Note: condition ? result_if_true : result_if_false, it's just syntactic sugar*/
}

void do_Left_Encoder(){
  //Read the input pin
  LeftEncoderBSet = digitalRead(Left_Encoder_PinB);
  Left_Encoder_Ticks += LeftEncoderBSet ? -1 : +1;
}

void setup(){
  //Initiate a Serial Connection through the Serial Port at a buad rate of 115200
  Serial.begin(115200);

  //Set up the Encoders
  set_up_encoders();
  
  //Set the Right Motor Pins as OUTPUT
  pinMode(INA_2, OUTPUT);
  pinMode(INB_2, OUTPUT);
  pinMode(PWM_2, OUTPUT);

  //Set the Left Motor Pins as OUTPUT
  pinMode(INA_1, OUTPUT);
  pinMode(INB_1, OUTPUT);
  pinMode(PWM_1, OUTPUT);
}

void loop(){
  //update the encoders
  update_encoders();
  
  //Move forward for 5 seconds
  //move_forward();
  //delay(5000);

  //Stop for 1 second
  //stop();
  //delay(1000);

  //Move backward for 5 seconds
  //move_backwards();
  //delay(5000);

  //Move right for 5 seconds
  move_right();
  delay(5000);

  //Move left for 5 seconds
  move_left();
  delay(5000);
}
