
#include <Servo.h>

//double check that the color sensors match the sensors in the vehicle 

//Right Color Sensor, CS1, right sensor
#define RS0 28
#define RS1 29
#define RS2 30
#define RS3 31
#define RSOut 32

//Left Color Sensor, CS2, left sensor 
#define LS0 33
#define LS1 34
#define LS2 35
#define LS3 36
#define LSOut 37

//Arm Color Sensor, CS3, the sensor on the top of the gripper 
#define AS0 13
#define AS1 14
#define AS2 15
#define AS3 16
#define ASOut 17

// Analog values for colors
//this may need recalibration depending on tests but hasnt been changed yet 
#define redMin 38
#define redMax 236
#define greenMin 40
#define greenMax 364
#define blueMin 36
#define blueMax 319

//Left Motor
//I changed the values here to align with the chart Perrin made
#define ENA 0 //changed from 1
#define IN1 1 //changed from 2
#define IN2 2 //changed from 3

// RIGHT Motor
#define ENB 3 //changed from 5
#define IN3 4 //changed from 6
#define IN4 5 //changed from 7

// Ultrasonic Sensor 1, front of the vehicle, !I added the numbers to the end of the names to differentiate from each ultrasonic sensor, ie echopin -> echopin1, I also added the other ultra sonic sensor!
#define echoPin1 24 //this sensor is the one that is in the front 
#define trigPin1 25//this sensor is the one that is in the front

// _Ultrasonic Sensor 2, side of the vehicle 
#define echoPin2 26 //side ultrasonis sensor 
#define trigPin2 27 //side ultrasonis sensor

//Force sensor
// this has not been wired yet should be doubled chech when it is
#define forceSensor A6

//Timing
#define speedR 30
#define speedL 30
#define waitTime 1450
#define turnTime 1380
#define time1 1000
#define time2 1000

//motors
int PWMvalR = 255*speedR/100;
int PWMvalL = 255*speedL/100;

//color sensor
int RredFreq = 0;     int LredFreq = 0;
int RgreenFreq = 0;   int LgreenFreq = 0;
int RblueFreq = 0;    int LblueFreq = 0;
int RredColor = 0;    int LredColor = 0;
int RgreenColor = 0;  int LgreenColor = 0;
int RblueColor = 0;   int LblueColor = 0;

char isColorR = 0;    char isColorL = 0;
char greenR = 0;      char greenL = 0;
//char is used to store a character value 

//ultrasonic sensor
long duration;
//long forces the variables to be long
int distance;
int targetDistance;

//gripper
Servo Gripper; // create servo object to control a servo
Servo Arm; // create servo object to control a servo
int actualPosition1 = 0;
int actualPosition2 = 300;
int PWMSignal;
int change1 = 0;
int change2 = 0;

//force sensor
int forceValue;

//Box information
char isLarge = 0; //0 = small, 1 = big
char isBlue = 0; //0 = red, 1 = blue

//States
enum {FindBox, FollowLine, PlaceBox, Return, End};
//enum tells the code to start a new tab
unsigned char state;
//unsigned char takes up 1byte of memory, creates a place to store a value

void setup() {
  // Set all motor controller pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initial state - Turn off all the motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  // Port setup for LEFT and RIGHT color sensor pins
  pinMode(RS0, OUTPUT); pinMode(RS1, OUTPUT); pinMode(RS2, OUTPUT); pinMode(RS3, OUTPUT); pinMode(RSOut, INPUT);
  pinMode(LS0, OUTPUT); pinMode(LS1, OUTPUT); pinMode(LS2, OUTPUT); pinMode(LS3, OUTPUT); pinMode(LSOut, INPUT);
  pinMode(AS0, OUTPUT); pinMode(AS1, OUTPUT); pinMode(AS2, OUTPUT); pinMode(AS3, OUTPUT); pinMode(ASOut, INPUT); //I added this line to account for the 3rd color sensor

  // Setting sensor frequency scaling to 20%
  digitalWrite(RS0, HIGH); digitalWrite(RS1, LOW);
  digitalWrite(LS0, HIGH); digitalWrite(LS1, LOW);
  digitalWrite(AS0, HIGH); digitalWrite(AS1, LOW); //I added this line to account for the 3rd color sensor, dont need for this code but useful for the other code 

  //ultrasonic sensors
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);// I added this line to account for the other color sensor, it needs to be determined if the color sensor is the fron or the side one 
  pinMode(echoPin2, INPUT);// I added this line to account for the other color sensor, it needs to be determined if the color sensor is the fron or the side one

  //Gripper
  Gripper.attach(15);//setting gripper pin, Analog 
  Arm.attach(14);//setting the arm pin, Analog

  //force sensor
  pinMode(forceSensor, INPUT);
}

void loop() {
  switch (state){
  
    case FindBox:
      findBox();
      if(LeftSensor() == 2 && RightSensor() == 2){ //if both color sensors see green(2) then the conditions are met to begin, this will be at the start line 
        Forward(); //drive up to the pick up box
        delay(waitTime); // determine the duration the vehicle will be moving forward, this may need to be changed depending how far the start path is from the start line  
        if(!isBlue) //is blue is 0
          RotateLft();
        else
          RotateRt();
        delay(turnTime);
        state = FollowLine;
      }
    break;
    //"state" specifies what code to use depending on the conditions 
    //"break" moves on to the next segment 

    case FollowLine:
      followLine();
      if(LeftSensor() == 2 && RightSensor() == 2){ //when these conditions are met it will be at the finsh line 
        Stop();
        state = PlaceBox;
      }
    break;

    case PlaceBox:
      placeBox();
      if(LeftSensor() == 2 && RightSensor() == 2){
        state = Return;
      }
    break;

    case Return:
      returnMethod();
    break;

    case End:
    break;

    default:
      state = FindBox;
    break;
  }
}

//state functions
//"void" ensure that the function is only used when declared
void findBox()
{
  if(Ultrasonic() <= targetDistance){
    Stop();
    MoveArm(180); //(value between 0 and 180) moves arm horizontal in the front
    MoveGripper(90); //opens gripper 
    if(analogRead(forceSensor) < forceValue){
      MoveGripper(180); //opens gripper more?, shouldn't it start large then close more if it is smaller so that it doesnt know the box over  
      isLarge = 1;
    }
    MoveArm(0);// moves arm horizonatal behind the ultrasonic sensor
    RotateRt();
    delay(2*turnTime); //turning 180 degrees
  }
  else
    Forward();
}

void followLine() 
{
  if(LeftSensor() == 1 && !(RightSensor() == 1)){ //if the left sensor detects the line color but the right does not 
    Forward();
    delay(waitTime);
    RotateLft();
    delay(turnTime);
  } 
  else if(!(LeftSensor() == 1) && RightSensor() == 1){ //if the right sensor detects the line color but the left does not
    Forward();
    delay(waitTime);
    RotateRt();
    delay(turnTime);
  }
  else if(LeftSensor() == 0 && RightSensor() == 0){ //if both detect the color, !ask! it should be if both detect black, was changed from 1 to 0 
    Forward();
    delay(waitTime);
    if(!isLarge) //not large 
      RotateRt();
    else
      RotateLft();
  }
  else{
    Forward();
  }
}

void placeBox()
{
  if(Ultrasonic() <= targetDistance){
    Stop();
    MoveArm(180); //moves the are horizonatl in the front 
    MoveGripper(0); //opens gripper 
    MoveArm(0); // moves arm back 
    RotateRt();
    delay(2*turnTime); //turn around
    state = Return;
  }
  else{
    followLine();
  }
}

void returnMethod()
{
  int num = 0;
  followLine();
  if(LeftSensor() == 2 && RightSensor() == 2)
    num++;
  if(num >= 2){
    Forward();
    delay(waitTime);
    if(!isBlue)
      RotateLft();
    else
      RotateRt();
    delay(turnTime);
    Forward();
    delay(time1);
    if(!isBlue)
      RotateRt();
    else
      RotateLft();
    delay(turnTime);
    Forward();
    delay(time2);
    state = End;
  }
}

//sensors and motors

char RightSensor() { // Uses color sensor to estimate the color of an object
  // Setting RED (R) filter photodiodes to be read
  digitalWrite(RS2,LOW);
  digitalWrite(RS3,LOW);
  RredFreq = pulseIn(RSOut, LOW);
  RredColor = map(RredFreq, redMin, redMax, 255, 0);
  
  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(RS2,HIGH);
  digitalWrite(RS3,HIGH);
  RgreenFreq = pulseIn(RSOut, LOW);
  RgreenColor = map(RgreenFreq, greenMin, greenMax, 255, 0);
  
  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(RS2,LOW);
  digitalWrite(RS3,HIGH);
  RblueFreq = pulseIn(RSOut, LOW);
  RblueColor = map(RblueFreq, blueMin, blueMax, 255, 0);

  if((isBlue && RblueColor>RgreenColor && RblueColor>RredColor) || (!isBlue && RredColor>RgreenColor && RredColor>RblueColor)){ //blue or red
    isColorR = 1; 
  } 
  else if(RgreenColor>RredColor && RgreenColor>RblueColor){ //green
    isColorR = 2;
  }
  else {
    isColorR = 0;
  }
  
  return isColorR; // Returns the colorEst value for use in the void loop
}

char LeftSensor() { // Uses color sensor to estimate the color of an object
  // Setting RED (R) filter photodiodes to be read
  digitalWrite(LS2,LOW);
  digitalWrite(LS3,LOW);
  LredFreq = pulseIn(LSOut, LOW);
  LredColor = map(LredFreq, redMin, redMax, 255, 0);
  
  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(LS2,HIGH);
  digitalWrite(LS3,HIGH);
  LgreenFreq = pulseIn(LSOut, LOW);
  LgreenColor = map(LgreenFreq, greenMin, greenMax, 255, 0);
  
  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(LS2,LOW);
  digitalWrite(LS3,HIGH);
  LblueFreq = pulseIn(LSOut, LOW);
  LblueColor = map(LblueFreq, blueMin, blueMax, 255, 0);

  if((isBlue && LblueColor>LgreenColor && LblueColor>LredColor) || (!isBlue && LredColor>LgreenColor && LredColor>LblueColor)){ //blue or red
    isColorL = 1; 
  } 
  else if(RgreenColor>RredColor && RgreenColor>RblueColor){ //green
    isColorL = 2;
  }
  else{
    isColorL = 0;
  }
  
  return isColorL; // Returns the colorEst value for use in the void loop
}

int Ultrasonic()
{
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  return distance;
}

void MoveArm(int theta) //ask about this 
{
  actualPosition1 = actualPosition1 + change1;
  if(actualPosition1<30){
    change1 = 30;
  }
  if(actualPosition1>270){
    change1 = -30;
  }
  int val1 = map(actualPosition1, 0, 300, 0, theta);
  Arm.write(val1);
  delay(100);
}

void MoveGripper(int theta){
  actualPosition2 = actualPosition2 + change2;
  if(actualPosition2<30){
    change2 = 30;
  }
  if(actualPosition2>270){
    change2 = -30;
  }
  int val2 = map(actualPosition2, 0, 300, 0, theta);
  Gripper.write(val2);
  delay(100);
}

void Stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void Forward() {
  analogWrite(ENA, PWMvalR);
  analogWrite(ENB, PWMvalL);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void RotateRt() {
  analogWrite(ENA, PWMvalR);
  analogWrite(ENB, PWMvalL);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void RotateLft() {
  analogWrite(ENA, PWMvalR);
  analogWrite(ENB, PWMvalL);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void Backward()
{
  analogWrite(ENA, PWMvalR);
  analogWrite(ENB, PWMvalL);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
} 
