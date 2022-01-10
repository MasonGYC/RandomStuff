//Include required libraries
#include <PPMReader.h>
#include <Servo.h>

//Instantiate servo motor objects, names can be changed
//Servo servo_Name;
Servo servo_base1;
Servo servo_base2;
Servo servo_gripper;

int pos_base1 = 0;
int pos_base2 = 0;
int pos_gripper = 0;

//Global Pins Assignment for motor pins, and motor enable pins
//IN1-IN4 pins can be connected to both digital and analog pins
//EN1-EN2 pins to be connected to D3, D5, D6, D9, D10, or D11
//const int pin_Name = pin_Number;
const int FL_FwdPin = A4;
const int FL_BwdPin = A5;
const int BL_FwdPin = 13;
const int BL_BwdPin = 12;
const int L_EN = 5;
const int FR_FwdPin = A3;
const int FR_BwdPin = A2;
const int BR_FwdPin = A1;
const int BR_BwdPin = A0;
const int R_EN = 6;

//Controller settings, do not change
byte interruptPin = 3;
byte channelAmount = 8;
PPMReader ppm(interruptPin, channelAmount);

//Initialize global variables
//These are used in the calculation for the robot's wheel speed
//when it is turning left/right
int turn_Speed = 0;
int L_Speed = 0;
int R_Speed = 0;
bool Dir = true; //Forward = true, Backward = false

//init utltrasonic sensor params
int echoPin = ;
int trigPin = ;
long duration;
float distance;
float diag = 184.3; //the distance betwn the tip of the gripper and the arm
float s = ;//dist btw the fron edge of servo mount and the ultra sensor

//init IR sensor pins
int irPinl = 4;//IR sensor digital pin left
int irPinm = 7;//IR sensor digital pin middle
int irPinr = 8;//IR sensor digital pin right

/*mayn not use but may use to terminate
int interruptPin = ;//any digital pin that is unused for terminating auto periord
volatile byte state = LOW;//LOW means auto is starting
*/

void setup() {
  //Assign servo motors to pins, can be any digital pins
  //ServoName.attach(pin_Number);
  servo_base1.attach(9);
  servo_base2.attach(10);
  servo_gripper.attach(11);

  //Setting the L298N pins as output pins
  //pinMode(pin_Name, OUTPUT);
  pinMode(FL_FwdPin, OUTPUT);
  pinMode(FL_BwdPin, OUTPUT);
  pinMode(BL_FwdPin, OUTPUT);
  pinMode(BL_BwdPin, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(FR_FwdPin, OUTPUT);
  pinMode(FR_BwdPin, OUTPUT);
  pinMode(BR_FwdPin, OUTPUT);
  pinMode(BR_BwdPin, OUTPUT);
  pinMode(R_EN, OUTPUT);

  //Setting the servo motor pins as output pins
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  

  //Start serial comm for troubleshooting through serial monitor
  //Serial.begin(baud rate);
  Serial.begin(115200);
  
  //setup utrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  //set up irsensor pins
  pinMode(irPinl, INPUT);
  pinMode(irPinm, INPUT);
  pinMode(irPinr, INPUT);
  
  /*for interrupt
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin),terminate , HIGH);*/
}



void loop() {
  //Read controller channel values
  //In this case, channel 1 and 2 are assigned to the right
  //stick and left stick respectively
  //channel 3 is right stick(up and down)
  //channel 4 is left stick(left and right)
  //int channel_Name = readChannel(channel_Number);
  int c1 = readChannel(1);
  int c2 = readChannel(2);
  int c3 = readChannel(3);
  int c4 = readChannel(4);

  //aux. channels value:
  //key: when press is 2000, release is 1000
  //SWx: A&D:up is 1000, down is 2000; B&C:up is 1000, middle is 1500, down is 2000

  int swa = readChannel(5);
  int swd = readChannel(6);
  int swb = readChannel(7);
  int swc = readChannel(8);
  
  //start auto function
  //insert channel value for starting auto period
  //one iteration is calc to be 30 sec sharp, will be terminate after finishing this round
  if() {autonomous()}
  
  
  //If the left stick is pushed down, wheels on both sides will
  //rotate forward. Speed of both sides will be calculated
  //with the "speed_Check" function.
  if (c2 >= 1000 and c2 < 1450) {
    leftBwd();
    rightBwd();
    Dir = false;
    speed_Check(Dir, c2, c4, 1450, 1000);
  }
  //If the left stick is pushed up, wheels on both sides
  //will rotate backwards.
  else if (c2 <= 2000 and c2 > 1550) {
    leftFwd();
    rightFwd();
    Dir = true;
    speed_Check(Dir, c2, c4, 1550, 2000);
  }
  //If the left stick is stationary (in the middle)
  else {
    //If the left stick is pushed left, left wheel will rotate
    //backwards, while the right wheel will rotate forward.
    //Rotation speed will be calculated using "rotate" function.
    if (c4 >= 1000 and c4 < 1450) {
      leftBwd();
      rightFwd();
      rotate(c4, 1450, 1000);
    }
    //If the left stick is pushed right,left wheel will rotate
    //forward, while the right wheel will rotate backwards.
    else if (c4 <= 2000 and c4 > 1550) {
      leftFwd();
      rightBwd();
      rotate(c4, 1550, 2000);
    }
    //If the left stick is stationary (in the middle),
    //stop the robot from moving
    else {
      leftStop();
      rightStop();
    }
  }

  //INSERT GRIPPER CODE HERE

  //servo_base
  //if right stick is pushed up, servo motor will rotate from 0 to 180 degree;
  //if right stick is pushed down, servo motor will rotate from 180 to 0 degree;
  if (c3 >= 1000 and c3 <= 1500){
    pos_base1 = map(c3, 1000, 1500, 0, 90);
    pos_base2 = map(c3, 1000, 1500, 180, 90);
    servo_base1.write(pos_base1);
    servo_base2.write(pos_base2);
    }
  else if (c3 <= 2000 and c3 > 1500){
    pos_base1 = map(c3, 1500, 2000, 90, 180);
    pos_base2 = map(c3, 1500, 2000, 90, 0);
    servo_base1.write(pos_base1);
    servo_base2.write(pos_base2);
    }

  
  //servo_gripper
  
  //using two SWx(SWA&SWD) pins to control the position of servo_griper
  //if swa is up and swd is up:
  //the gripper prepares to grab the ball
  if (swa == 1000 and swd == 1000){
    pos_gripper = 20;
    servo_gripper.write(pos_gripper);
    }
  //if swa is up and swd is down:
  //the gripper grabs the ball
  else if (swa == 1000 and swd == 2000){
    pos_gripper = 0;
    servo_gripper.write(pos_gripper);
    }
  //if swa is down and swd is up:
  //the gripper prepares to grab the cube
  else if (swa == 2000 and swd == 1000){
    pos_gripper = 90;
    servo_gripper.write(pos_gripper);
    }
  //if swa is down and swd is down:
  //the gripper
  else if (swa == 2000 and swd == 2000){
    pos_gripper = 70;
    servo_gripper.write(pos_gripper);
    }
  
  //ultra sensor to calc dist and then pickup ball/cube

  
  


    

  
    

  delay(10);
}

void speed_Check(bool dir, int throttle_Val,int steering_Val, int l_Limit, int r_Limit) {
  //Set minimum value for the motors to be 127
  //Map the input values from the controller to the
  //motors' speed value
  L_Speed = map(throttle_Val, l_Limit, r_Limit, 127, 255);
  R_Speed = map(throttle_Val, l_Limit, r_Limit, 127, 255);

  //If the right stick is pushed left,
  //1. robot will steer left if moving forward
  //2. robot will steer right if moving backwards
  if (steering_Val >= 1000 and steering_Val < 1400){
    turn_Speed = map(steering_Val, 1400, 1000, 0, 127);
    L_Speed = abs(L_Speed + (-2 * dir + 1)*turn_Speed);
    R_Speed = abs(R_Speed + (2 * dir - 1)*turn_Speed);
  }
  //If the right stick is pushed right,
  //1. robot will steer right if moving forward
  //2. robot will steer left if moving backwards
  else if (steering_Val <= 2000 and steering_Val > 1600){
    turn_Speed = map(steering_Val, 1600, 2000, 0, 127);
    L_Speed = abs(L_Speed + (2 * dir - 1)*turn_Speed);
    R_Speed = abs(R_Speed + (-2 * dir + 1)*turn_Speed);
  }
  //if the right stick is stationary, continue as per normal
  else{
    L_Speed = map(throttle_Val, l_Limit, r_Limit, 127, 255);
    R_Speed = map(throttle_Val, l_Limit, r_Limit, 127, 255);
  }

  //Limit the wheel's speed value to 255
  if(L_Speed > 255){L_Speed = 255;}
  if(R_Speed > 255){R_Speed = 255;}
  //Control the wheel's speed via PWM
  analogWrite(L_EN, L_Speed);
  analogWrite(R_EN, R_Speed);
}

void rotate(int rotate_Val, int l_Limit, int r_Limit){
    L_Speed = map(rotate_Val, l_Limit, r_Limit, 127, 255);
    R_Speed = map(rotate_Val, l_Limit, r_Limit, 127, 255);
    analogWrite(L_EN, L_Speed);
    analogWrite(R_EN, R_Speed);
}



void leftFwd() {
  digitalWrite(FL_FwdPin, HIGH);
  digitalWrite(FL_BwdPin, LOW);
  digitalWrite(BL_FwdPin, HIGH);
  digitalWrite(BL_BwdPin, LOW);
}

void leftBwd() {
  digitalWrite(FL_FwdPin, LOW);
  digitalWrite(FL_BwdPin, HIGH);
  digitalWrite(BL_FwdPin, LOW);
  digitalWrite(BL_BwdPin, HIGH);
}

void leftStop() {
  digitalWrite(FL_FwdPin, LOW);
  digitalWrite(FL_BwdPin, LOW);
  digitalWrite(BL_FwdPin, LOW);
  digitalWrite(BL_BwdPin, LOW);
}



void rightFwd() {
  digitalWrite(FR_FwdPin, HIGH);
  digitalWrite(FR_BwdPin, LOW);
  digitalWrite(BR_FwdPin, HIGH);
  digitalWrite(BR_BwdPin, LOW);
}

void rightBwd() {
  digitalWrite(FR_FwdPin, LOW);
  digitalWrite(FR_BwdPin, HIGH);
  digitalWrite(BR_FwdPin, LOW);
  digitalWrite(BR_BwdPin, HIGH);
}

void rightStop() {
  digitalWrite(FR_FwdPin, LOW);
  digitalWrite(FR_BwdPin, LOW);
  digitalWrite(BR_FwdPin, LOW);
  digitalWrite(BR_BwdPin, LOW);
}




int readChannel(int channelNumber) {
  unsigned value = ppm.latestValidChannelValue(channelNumber, 0);
  //Serial.print(value);
  //Serial.println(", ");
  return value;
}

float distance(){
  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  return distance;
}

void pick(int channel, double theta, float diag){
  //cube
  if channel == 
  {int lower = 30;
   int upper = 100;
   double beta = ;//fix an angle to pick cube
   float pos_gripper = ;//angle for picking up
   pick_helper(double theta, float diag, int lower, int upper, double beta,float pos_gripper)
    }
  //ball
  elif channel == 
  {int lower = 15;
   int upper = 45;
   double beta = ;//fix an angle to pick ball
   float pos_gripper = ;//angle for picking up
   pick_helper(double theta, float diag, int lower, int upper, double beta,float pos_gripper)
    }
}

void pick_helper(double theta, float diag, int lower, int upper, double beta,float pos_gripper){
  double c = 0;//init c: dist btw tip of gripper and inner edge of cube/ball
  
  //lift arm to nearly vertical
  pos_base1 = pos_base2 = ;
  servo_base1.write(pos_base1);
  servo_base2.write(pos_base2);
  
}

//release the ball to the stovetop
void release(){
  pos_gripper = ;//widest angle for releasing object
  servo_gripper.write(pos_gripper);
}

//read irsensor val,return analog and digit values
float ir_read(){
  irl_val = digitalRead(irPinl);
  irm_val = digitalRead(irPinm);
  irr_val = digitalRead(irPinr);
  return irl_val,irm_val,irr_val;
}

void follow_line(){
  irl_val,irm_val,irr_val = ir_read();
  //0 is white, 1 is black
  while (irm_val == 1 && irl_val == 0 && irr_val == 0){
    leftFwd();
    rightFwd();
    Dir = true;
    speed_Check(Dir, 1600, 1500, 1550, 2000);
    delaymicroseconds(10)
  }
}

void autonomous(int channel){
  //read sensor value and do sth
    irl_val,irm_val,irr_val = ir_read();
    d = distance()
  
  //switch among 2 diff strategy
    switch (channel)
    {
      case : ; break;
      case : ; break;
    }
  
  //terminate
  if (channel == ) break
  
    
    
}
