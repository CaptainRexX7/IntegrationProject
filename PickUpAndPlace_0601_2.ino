#include <Servo.h>

//Right Color Sensor, CS1, right sensor
#define RS0 33
#define RS1 34
#define RS2 35
#define RS3 36
#define RSOut 37

//Left Color Sensor, CS2, left sensor 
#define LS0 28
#define LS1 29
#define LS2 30
#define LS3 7
#define LSOut 12

//Arm Color Sensor, CS3, the sensor on the top of the gripper 
#define AS0 17
#define AS1 14
#define AS2 15
#define AS3 16
#define ASOut 13

// Analog values for colors
//this may need recalibration depending on tests but hasnt been changed yet 
//CS1
#define RredMin 90
#define RredMax 560
#define RgreenMin 100
#define RgreenMax 720
#define RblueMin 90
#define RblueMax 640
//CS2
#define LredMin 130
#define LredMax 470
#define LgreenMin 130
#define LgreenMax 560
#define LblueMin 110
#define LblueMax 480
//CS3
#define AredMin 90
#define AredMax 160
#define AgreenMin 130
#define AgreenMax 200
#define AblueMin 150
#define AblueMax 185

//Left Motor
//I changed the values here to align with the chart Perrin made
#define ENA 0 
#define IN1 1 
#define IN2 2 

// RIGHT Motor
#define ENB 3 
#define IN3 4 
#define IN4 5 

// Ultrasonic Sensor 1, front of the vehicle
#define echoPin1 27 //this sensor is the one that is in the front 
#define trigPin1 26//this sensor is the one that is in the front

// _Ultrasonic Sensor 2, side of the vehicle, Commented because it's not used in the code
//#define echoPin2 26 //side ultrasonis sensor 
//#define trigPin2 27 //side ultrasonis sensor

//Gripper Moved from int variables to defines to save memory 
#define RestingArm 180 //changed
#define WorkingArm 15 //changed
#define SmallBox 15
#define LargeBox 29
#define Open 70
#define targetDistance 11 // the vehicle needs to be within 10cm

//Force sensor
#define forceSensor A9 //changed
#define forceValue 25

//Timing
#define speedR 22 //slowed down to give the light sensors more time to read
#define speedL 22 //slowed down to give the light sensors more time to read
#define waitTime 1450
#define turnTime1 9750//7000
#define turnTime2 2000

//motors
int PWMvalR = 255*speedR/100;
int PWMvalL = 255*speedL/100;
//int Movefwd10CM = 550;

//color sensor                              Added arm values
int RredFreq = 0;     int LredFreq = 0;     int AredFreq = 0;
int RgreenFreq = 0;   int LgreenFreq = 0;   int AgreenFreq = 0;
int RblueFreq = 0;    int LblueFreq = 0;    int AblueFreq = 0;
int RredColor = 0;    int LredColor = 0;    int AredColor = 0;
int RgreenColor = 0;  int LgreenColor = 0;  int AgreenColor = 0;
int RblueColor = 0;   int LblueColor = 0;   int AblueColor = 0;

char isColorR = 0;    char isColorL = 0;
char greenR = 0;      char greenL = 0;
//char is used to store a character value 

//ultrasonic sensor
long duration;
//long forces the variables to be long
int distance;

//gripper
Servo Gripper;
Servo Arm;
int PWMSignal;
int armAngle;
int handAngle;


//Box information
char isLarge = 0; //0 = small, 1 = big
char isBlue = 0; //0 = red, 1 = blue

//FollowLine setting   checks if FollowLine state has ran twice
char firstTime = 1;
char firstTurn = 1;

//States  removed return to simplfy code
enum {FindBox, FollowLine, PlaceBox, End}; //I removed the return state
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
  //pinMode(trigPin2, OUTPUT);// I added this line to account for the other color sensor, it needs to be determined if the color sensor is the fron or the side one 
  //pinMode(echoPin2, INPUT);// I added this line to account for the other color sensor, it needs to be determined if the color sensor is the fron or the side one

  //Gripper
  Gripper.attach(39);//setting gripper pin, !!changed to digital based on the servo calibration code!! 
  Arm.attach(38);//setting the arm pin, !!changes to digital!!
  MoveArm(RestingArm); //Sets gripper arm postion
  MoveGripper(Open); //sets gripper hand postion

  //force sensor
  pinMode(forceSensor, INPUT);

  firstTime = 1;
  firstTurn = 1;

  state = FollowLine;//PlaceBox;

  //Serial.begin(9600); //Temporary. Used for testing
}

void loop() {
  //Arm.write(armAngle);  
  //Gripper.write(handAngle);
  //Ensures the gripper stays in the last set position

  switch (state){
  
    case FindBox:
      findBox();
      if(LeftSensor() == 2 && RightSensor() == 2){ //if both color sensors see green(2) then the conditions are met, this will be at the start line 
        Forward(); //moves point of rotaion onto the green line.
        delay(waitTime); // determine the duration the vehicle will be moving forward, this may need to be changed  
        if(!isBlue) //if blue is 0, a line added to force this to not be blue 
          RotateLft();
        else
          RotateRt();
        delay(turnTime1);
        state = FollowLine;
      }
    break;
    //"state" specifies what code to use depending on the conditions 
    //"break" moves on to the next segment 

    case FollowLine:
      if(firstTurn){
        followLine(turnTime1);
      }
      else{
        followLine(turnTime2);
      }
      if(isColorR == 2 && isColorL == 2){ //when these conditions are met it will be at the green line 
        Stop();
        if(firstTurn){
          Forward();
          delay(1000);
          firstTurn = 0;
        }
        else if(firstTime){
          firstTime = 0;
          Forward();
          delay(1000);
          state = PlaceBox;
        }
        else{           //The second time this code runs will end the program arriving back at the first green line.
          state = End;
        }
      }
    break;

    case PlaceBox: //Fixed some errors and set to return to follow line
      placeBox();
      if(LeftSensor() == 2 && RightSensor() == 2){
        state = FollowLine;
      }
    break;

    case End:
      Stop();
    break;

    default:
      state = End;
    break;
  }
}

//state functions
//"void" ensure that the function is only used when declared
void findBox()
{
  MoveGripper(Open);
  MoveArm(RestingArm);
  if(Ultrasonic() <= targetDistance){
    Stop();
    delay(250);
    MoveArm(WorkingArm); //moves the arm into the position to pick up the box
    delay(500);
    MoveGripper(LargeBox); //Moves gripper to the designated size of the large box
    delay(900);
    if(analogRead(forceSensor) < forceValue){ //if there is no force onto the side then it is not a large box and it will close more to be able to pick up the smal box
      MoveGripper(SmallBox); 
      delay(500);
      isLarge = 0; //documenting that the box is small
    }
    else{
      isLarge = 1;
    }
    MoveArm(RestingArm);// moves arm so that it is out of the way of the ultrasonic sensor
    ArmSensor(); //checks the color of the box //the light sensor requires at least 250ms to read
    delay(500);
    //RotateRt();
    //delay(2*turnTime); //turning 180 degrees
    state = End; //Temporary. Ends the program early for testing
  }
  else{
    Forward();
  }
}

void followLine(int time) 
{
  LeftSensor();
  RightSensor();
  
  if(isColorL == 1){ //if the left sensor detects the line color but the right does not
    if(firstTurn){
      Forward();
      delay(200); //500
    }
    RotateLft();
    delay(time);
    Forward();
    delay(1000);
    Stop();
    
  } 
  else if(isColorR == 1){ //if the right sensor detects the line color but the left does not
    if(firstTurn){
      Forward();
      delay(200); //500
    }
    RotateRt();
    delay(time);
    Forward();
    delay(1000);
    Stop();
  }
  else{
    Forward();
  }
}

void placeBox()
{
  RightSensor();
  LeftSensor();
  if(isColorL == 1 && isColorR == 1){ //if both detect the color, choose which way to go to place the box 
    Forward();
    delay(waitTime);
    if(isLarge)
      RotateLft();
    else
      RotateRt();
    delay(turnTime1);
  }
  else if(isLarge && isColorR == 1){
    RotateRt();
    delay(turnTime1);
    Forward();
    delay(waitTime);
  }
  else if(!isLarge && isColorL == 1){
    RotateLft();
    delay(turnTime1);
    Forward();
    delay(waitTime);
  }
  else if(Ultrasonic() <= targetDistance){
    Stop();
    state = End;
    /*MoveArm(WorkingArm); //moves the arm horizonatl in the front 
    delay(500);
    MoveGripper(Open); //opens gripper 
    delay(500);
    MoveArm(RestingArm); // moves arm back 
    delay(500);
    RotateRt();
    delay(turnTime1*2); //turn around*/
  }
  else{
    Forward();
  }
  
}

//sensors and motors

char RightSensor() { // Uses color sensor to estimate the color of an object
  // Setting RED (R) filter photodiodes to be read
  digitalWrite(RS2,LOW);
  digitalWrite(RS3,LOW);
  RredFreq = pulseIn(RSOut, LOW);
  RredColor = map(RredFreq, RredMin, RredMax, 255, 0);
  
  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(RS2,HIGH);
  digitalWrite(RS3,HIGH);
  RgreenFreq = pulseIn(RSOut, LOW);
  RgreenColor = map(RgreenFreq, RgreenMin, RgreenMax, 255, 0);
  
  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(RS2,LOW);
  digitalWrite(RS3,HIGH);
  RblueFreq = pulseIn(RSOut, LOW);
  RblueColor = map(RblueFreq, RblueMin, RblueMax, 255, 0);

  if(RredColor<50 && RgreenColor<50 && RblueColor<50){
    isColorR = 0;
  }
  else if(RredColor>150 && RgreenColor>150){ //if yellow
    isColorR = 0;
  }
  else if((isBlue && RblueColor>RgreenColor && RblueColor>RredColor) || (!isBlue && RredColor>RgreenColor && RredColor>RblueColor)){ //blue or red
    isColorR = 1; 
  } 
  else if(RgreenColor>RredColor && RgreenColor>RblueColor){ //green
    isColorR = 2;
  }
  else {
    isColorR = 0;
  }
  //Serial.println((int)isColorR);
  return isColorR; // Returns the colorEst value for use in the void loop
}

char LeftSensor() { // Uses color sensor to estimate the color of an object   Fixed an error where varabiles were set to their right counterpart
  // Setting RED (R) filter photodiodes to be read
  digitalWrite(LS2,LOW);
  digitalWrite(LS3,LOW);
  LredFreq = pulseIn(LSOut, LOW);
  LredColor = map(LredFreq, LredMin, LredMax, 255, 0);
  
  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(LS2,HIGH);
  digitalWrite(LS3,HIGH);
  LgreenFreq = pulseIn(LSOut, LOW);
  LgreenColor = map(LgreenFreq, LgreenMin, LgreenMax, 255, 0);
  
  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(LS2,LOW);
  digitalWrite(LS3,HIGH);
  LblueFreq = pulseIn(LSOut, LOW);
  LblueColor = map(LblueFreq, LblueMin, LblueMax, 255, 0);

  if(LredColor<50 && LgreenColor<50 && LblueColor<50){
    isColorL = 0;
  }
  else if(LredColor > 150 && LgreenColor > 150){ //if yellow
    isColorL = 0;
  }
  else if((isBlue && LblueColor>LgreenColor && LblueColor>LredColor) || (!isBlue && LredColor>LgreenColor && LredColor>LblueColor)){ //blue or red
    isColorL = 1; 
  } 
  else if(LgreenColor>LredColor && LgreenColor>LblueColor){ //green
    isColorL = 2;
  }
  else{
    isColorL = 0;
  }
  //Serial.println((int)isColorL);
  return isColorL; // Returns the colorEst value for use in the void loop
}

void ArmSensor() {  //Checks the color of the box
  // Setting RED (R) filter photodiodes to be read
  digitalWrite(AS2,LOW);
  digitalWrite(AS3,LOW);
  AredFreq = pulseIn(ASOut, LOW);
  AredColor = map(AredFreq, AredMin, AredMax, 255, 0);
  
  // Setting GREEN (G) filtered photodiodes to be read
  digitalWrite(AS2,HIGH);
  digitalWrite(AS3,HIGH);
  AgreenFreq = pulseIn(ASOut, LOW);
  AgreenColor = map(AgreenFreq, AgreenMin, AgreenMax, 255, 0);
  
  // Setting BLUE (B) filtered photodiodes to be read
  digitalWrite(AS2,LOW);
  digitalWrite(AS3,HIGH);
  AblueFreq = pulseIn(ASOut, LOW);
  AblueColor = map(AblueFreq, AblueMin, AblueMax, 255, 0);

  if(AblueColor>AgreenColor && AblueColor>AredColor){
    isBlue = 1; 
    //Serial.print("blue");
  } 
  else{
    isBlue = 0;
    //Serial.print("red");
  }
}

int Ultrasonic()
{
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration = pulseIn(echoPin1, HIGH);
  distance = duration * 0.034 / 2;
  //Serial.println(distance); //Temporary. Used for testing
  return distance;
}

void MoveArm(int ArmPosition)
{
  armAngle = map(ArmPosition, 0, 300, 0, 180);
  Arm.write(armAngle);
  delay(100);//other delays were added to this one was shorted
}

void MoveGripper(int GripperPosition) 
{
  handAngle = map(GripperPosition, 0, 300, 0, 180);
  Gripper.write(handAngle);
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

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void RotateRt() {
  analogWrite(ENA, PWMvalR*2);
  analogWrite(ENB, PWMvalL*2);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void RotateLft() {
  analogWrite(ENA, PWMvalR*2);
  analogWrite(ENB, PWMvalL*2);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void Backward()
{
  analogWrite(ENA, PWMvalR);
  analogWrite(ENB, PWMvalL);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
} 