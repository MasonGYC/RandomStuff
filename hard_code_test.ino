//Include required libraries
#include <PPMReader.h>
#include <Servo.h>

//ps:
//6cm: deviation to run 110cm in 255 speed from the center line, 110 is for the mid of car to reach destination line, car head alinn with starting zone edge

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

int base_1_low = ;
int base_2_low = ;
int base_1_high = ;
int base_2_high = ;
int ball_prepare = 45;
int ball_pick = 37;

void setup() {

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

  //Start serial comm for troubleshooting through serial monitor
  //Serial.begin(baud rate);
  Serial.begin(115200);
}

void loop() {
  int swa = readChannel(5);
  if (swa>1900){
  start_to_pick_blue_ball_blueside();
}
else{
  leftStop();
  rightStop();
}
}

//auto
void start_to_pick_blue_ball_blueside(){
  //1m per 6cm deviation

  /*step1*/
  //110cm, from start to reaching blue ball center line
  int duration = 6;//time in sec in integer, got delay somehow
  for (int times = 0; times < (duration/1.5); times++){
    //1 second
    for (int count = 0; count < 300000; count++){go_forward();}
  }
  delay(200);

  /*step2*/
  //turn right
  int duration = 1.5;//time in sec in product of 1.5, got delay somehow
  for (int times = 0; times < (duration/1.5); times++){
    //1 second
    for (int count = 0; count < 300000; count++){turn_left();}
  }
  delay(200);

  /*step3*/
  //calc dist and pick a blue ball
  float dist = ;//dist btw wall and ultra
  float d = 0; 
  while (d<=dist and d>=0.05){
    go_forward();
    d = distance();}
    
  pick();

  /*step4*/
  //turn right
  int duration = 1.5;//time in sec in product of 1.5, got delay somehow
  for (int times = 0; times < (duration/1.5); times++){
    //1 second
    for (int count = 0; count < 300000; count++){turn_left();}
  }
  delay(200);
  
  /*step5*/
  //go straight back to starting zone, deviate 
  int duration = 6;//time in sec in integer, got delay somehow
  for (int times = 0; times < (duration/1.5); times++){
    //1 second
    for (int count = 0; count < 300000; count++){go_forward();}
  }
  delay(200);

  /*step6*/
  //turn right
  int duration = 1.5;//time in sec in product of 1.5, got delay somehow
  for (int times = 0; times < (duration/1.5); times++){
    //1 second
    for (int count = 0; count < 300000; count++){turn_left();}
  }
  delay(200);

  /*step7*/
  //go straight back to starting zone, deviate 
  int duration = 4;//time in sec in integer, got delay somehow
  for (int times = 0; times < (duration/1.5); times++){
    //1 second
    for (int count = 0; count < 300000; count++){go_forward();}
  }
  delay(200);

  /*step7*/
  //turn left
  int duration = 1.5;//time in sec in product of 1.5, got delay somehow
  for (int times = 0; times < (duration/1.5); times++){
    //1 second
    for (int count = 0; count < 300000; count++){turn_left();}
  }
  delay(200);

  /*step8*/
  //calc dist and pick a blue ball
  float dist = ;//dist btw stovetop and ultra
  float d = 0; 
  while (d<=dist){
    go_forward();
    d = distance();}
    
  /*step9*/
  release_ball();

  /*step10*/
  //go back to green ball center line
  
  }
void pick(){  
//lower down the arm
  pos_base1 = base_1_low;
  pos_base2 = base_2_low;
  servo_base1.write(pos_base1);
  servo_base2.write(pos_base2);
  
  //prepare to pick
  servo_gripper.write(ball_prepare);
  
  //pick
  servo_gripper.write(ball_pick);
  
  //lift arm to shoulder_picked
  pos_base1 = base_1_high;
  pos_base2 = base_2_high;
  servo_base1.write(pos_base1);
  servo_base2.write(pos_base2);
}

void release_ball(){
  servo_gripper.write(ball_prepare);
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





void turn_right(){
    //turn right when going forward
    leftFwd();
    rightBwd();
    rotate(1600, 1550, 2000);
}

void turn_left(){
    //turn left when going forward
    rightFwd();
    leftBwd();
    rotate(1400, 1550, 2000);
}

void go_forward(){
  //forward
    leftFwd();
    rightFwd();
    Dir = true;
    speed_Check(Dir, 1600, 1500, 1550, 2000);

}

void go_backward(){
  //backward
    leftBwd();
    rightBwd();
    Dir = false;
    speed_Check(Dir, 1600, 1500, 1550, 2000);
}


int readChannel(int channelNumber) {
  unsigned value = ppm.latestValidChannelValue(channelNumber, 0);
  //Serial.print(value);
  //Serial.println(", ");
  return value;
}
