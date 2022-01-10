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

//fixed angle for picking up balls or cubes
float shoulder[] = [,];//fix an angle to pick cube:0/ball:1
float gripper_init[] = [85,45];//fix gripper angle for picking up cube:0/ball:1
float gripper_pick[] = [63,33];

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

//auto params
float ball_pick_dist = ;//dist calc by ultra btw ultra and ball for pick it up
float put_stove_dist = 10;//dist calc by ultra btw stove and ultra to put the ball

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
  int swc = readChannel(8);//start auto for red or blue or 
  
  //start auto period,grab 1 green and 1 blue
  //B&C:up is 1000, middle is 1500, down is 2000
  //down is default
  if(swc >= 1400 and swc<=1600) {autonomous_blue();}
  elif(swc >= 1900 and swc<=2100) {autonomous_red();}

  
  
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

  
  //servo gripper
  //ball: prepare: 45 grabbed: 15
  //cube: prepare: 75 grabbed: 55
  
  //using two SWx(SWA&SWD) pins to control the position of servo_griper
  //if swa is up and swd is up:
  //the gripper prepares to grab the ball
  if (swa == 1000 and swd == 1000){
    pos_gripper = 45;
    servo_gripper.write(pos_gripper);
    }
  //if swa is up and swd is down:
  //the gripper grabbed the ball
  else if (swa == 1000 and swd == 2000){
    pos_gripper = 33;
    servo_gripper.write(pos_gripper);
    }
  //if swa is down and swd is up:
  //the gripper prepares to grab the cube
  else if (swa == 2000 and swd == 1000){
    pos_gripper = 85;
    servo_gripper.write(pos_gripper);
    }
  //if swa is down and swd is down:
  //the gripper grabbed the cube
  else if (swa == 2000 and swd == 2000){
    pos_gripper = 63;
    servo_gripper.write(pos_gripper);
    }

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


void turn_right(){
    //turn right when going forward
    leftFwd();
    rightBwd();
    rotate(1600, 1550, 2000);
    delay(10);
}

void turn_left(){
    //turn left when going forward
    rightFwd();
    leftBwd();
    rotate(1400, 1550, 2000);
    delay(10);
}

void go_forward(){
  //forward
    leftFwd();
    rightFwd();
    Dir = true;
    speed_Check(Dir, 1600, 1500, 1550, 2000);
    delay(10);
}

void go_backward(){
  //backward
    leftBwd();
    rightBwd();
    Dir = false;
    speed_Check(Dir, 1600, 1500, 1550, 2000);
    delay(10);
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

void pick(int channel){
  //cube
  if channel == {pick_helper(float shoulder[0], float gripper_init[0],float gripper_pick[0]);}
  //ball
  elif channel == {pick_helper(float shoulder[1],  float gripper_init[0], float gripper_pick[1]);}
}

void pick_helper(float shoulder,float gripper_init,float gripper_pick){
  
  float shoulder_picked = ;//the angle to get back after picking up
  
  //lower down the arm
  pos_base1 = pos_base2 = shoulder;
  servo_base1.write(pos_base1);
  servo_base2.write(pos_base2);
  
  //prepare to pick
  servo_gripper.write(gripper_init);
  
  //pick
  servo_gripper.write(gripper_pick);
  
  //lift arm to shoulder_picked
  pos_base1 = pos_base2 = shoulder_picked;
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
  delay(10);
  return irl_val,irm_val,irr_val;
}


//follow a staright line autoly
void straight_fwd(){
  irl_val,irm_val,irr_val = ir_read();
  //0 is white, 1 is black
  if ((irm_val == 1 && irl_val == 0 && irr_val == 0)||(irm_val == 0 && irl_val == 0 && irr_val == 0)){
    go_forward();
  }
  if (irm_val == 0 && irl_val == 0 && irr_val == 1){
    turn_right();
  }
  if (irm_val == 0 && irl_val == 1 && irr_val == 0){
    turn_left();
  }
   if (irm_val == 1 && irl_val == 1 && irr_val == 1)
    break;
   delaymicroseconds(50);
}

//follow a staright line autoly
void straight_bwd(){
  irl_val,irm_val,irr_val = ir_read();
  //0 is white, 1 is black
  if (irm_val == 1 && irl_val == 0 && irr_val == 0){
    go_backward();
  }
  if (irm_val == 0 && irl_val == 1 && irr_val == 0){
    turn_right();
  }
  if (irm_val == 0 && irl_val == 0 && irr_val == 1){
    turn_left();
  }
   if (irm_val == 1 && irl_val == 1 && irr_val == 1)
     break;
   delaymicroseconds(50);
}



//if we are blue team, and stand at the outer side
void autonomous_blue()
{

  //go straight and suppose follow black line
  //grab green ball and put on stove

  //go out and pick the ball
  while (1){straight_fwd();}
  turn_right();
  while (1){straight_fwd();}
  float d = 1000;
  while (d> ball_pick_dist ){
    go_forward();
    delaymicroseconds(50);
    d = distance();}
  pick();//insert channel value for picking ball
  //going back from ball zone
  go_backward();
  delaymicroseconds(50);
  while (1) {straight_bwd();}//back to black line
  turn_right();
  //heading to starting zone
  while (1){straight_fwd();}
  //untill starting zone
  //dist btw starting wall & car, heading to wall
  while (d>20){
    go_forward();
    delaymicroseconds(50);
    d = distance();}
  
  turn_right();
  //dist btw middle wall and car
  while (d>30){
      go_forward();
      delaymicroseconds(50);
      d = distance();}
  //heading to stove
  turn_left();
  while (d>put_stove_dist){
    go_forward();
    delaymicroseconds(50);
    d = distance();}
  release();

  //grab blue ball and put on stove
  go_backward();
  delay(5);
  //out of starting zone
  while (1) {straight_bwd();}
  turn_left();
  //follow line heading to green
  while (1) {straight_fwd();}
  turn_left();
  while (1) {straight_fwd();}
  turn_right();
  while (1) {straight_fwd();}
  //reach ball zone
  while (d> ball_pick_dist ){
    go_forward();
    delaymicroseconds(50);
    d = distance();}
  pick();//insert channel value for picking ball
   //going back from ball zone
  go_backward();
  delaymicroseconds(50);
  while (1) {straight_bwd();}//back to black line
  turn_right();
  //heading to starting zone
  while (1){straight_fwd();}
  //untill starting zone
  //dist btw starting wall & car, heading to wall
  while (d>20){
    go_forward();
    delaymicroseconds(50);
    d = distance();}
  //head to blocking glass
  turn_right();
  //dist btw middle wall and car
  while (d>30){
      go_forward();
      delaymicroseconds(50);
      d = distance();}
  //heading to stove
  turn_left();
  while (d>put_stove_dist){
    go_forward();
    delaymicroseconds(50);
    d = distance();}
  release();;
}

//if we are red team, and stand at the outer side
void autonomous_red(){
  //go straight and suppose follow black line
  //grab green ball and put on stove

  //go out and pick the ball
  while (1){straight_fwd();}
  turn_left();
  while (1){straight_fwd();}
  float d = 1000;
  while (d> ball_pick_dist ){
    go_forward();
    delaymicroseconds(50);
    d = distance();}
  pick();//insert channel value for picking ball
  //going back from ball zone
  go_backward();
  delaymicroseconds(50);
  while (1) {straight_bwd();}//back to black line
  turn_left();
  //heading to starting zone
  while (1){straight_fwd();}
  //untill starting zone
  //dist btw starting wall & car, heading to wall
  while (d>20){
    go_forward();
    delaymicroseconds(50);
    d = distance();}
  
  turn_left();
  //dist btw middle wall and car
  while (d>30){
      go_forward();
      delaymicroseconds(50);
      d = distance();}
  //heading to stove
  turn_right();
  while (d>put_stove_dist){
    go_forward();
    delaymicroseconds(50);
    d = distance();}
  release();

  //grab blue ball and put on stove
  go_backward();
  delay(5);
  //out of starting zone
  while (1) {straight_bwd();}
  turn_right();
  //follow line heading to green
  while (1) {straight_fwd();}
  turn_right();
  while (1) {straight_fwd();}
  turn_left();
  while (1) {straight_fwd();}
  //reach ball zone
  while (d> ball_pick_dist ){
    go_forward();
    delaymicroseconds(50);
    d = distance();}
  pick();//insert channel value for picking ball
   //going back from ball zone
  go_backward();
  delaymicroseconds(50);
  while (1) {straight_bwd();}//back to black line
  turn_left();
  //heading to starting zone
  while (1){straight_fwd();}
  //untill starting zone
  //dist btw starting wall & car, heading to wall
  while (d>20){
    go_forward();
    delaymicroseconds(50);
    d = distance();}
  //head to blocking glass
  turn_left();
  //dist btw middle wall and car
  while (d>30){
      go_forward();
      delaymicroseconds(50);
      d = distance();}
  //heading to stove
  turn_right();
  while (d>put_stove_dist){
    go_forward();
    delaymicroseconds(50);
    d = distance();}
  release();;
}