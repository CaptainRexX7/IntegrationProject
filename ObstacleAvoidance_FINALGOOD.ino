//Right Color Sensor, CS1
#define RS0 33
#define RS1 34
#define RS2 35
#define RS3 36
#define RSOut 37

//Left Color Sensor, CS2 
#define LS0 28
#define LS1 29
#define LS2 30
#define LS3 7
#define LSOut 12

// Analog values for colors
//CS1
#define RredMin 90
#define RredMax 540
#define RgreenMin 100
#define RgreenMax 630
#define RblueMin 90
#define RblueMax 530
//CS2
#define LredMin 150
#define LredMax 540
#define LgreenMin 130
#define LgreenMax 610
#define LblueMin 110
#define LblueMax 520

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
#define echoPin1 27 
#define trigPin1 26

// _Ultrasonic Sensor 2, side of the vehicle 
#define echoPin2 25
#define trigPin2 24

//Timing
#define speedR 24
#define speedL 24
#define waitTime 2800
#define turnTime1 6300
#define turnTime2 6950
#define turnTime3 6600

//Ultrasonic
#define targetDistance1 12
#define targetDistance2 30

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

//front ultrasonic sensor
long duration1;
int distance1;

//side ultrasonic sensor
long duration2;
int distance2;

//States
enum {FindBoundary,FindGap,Choice,End};
unsigned char state;

void setup() {
  // put your setup code here, to run once:
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
  
  // Setting sensor frequency scaling to 20%
  digitalWrite(RS0, HIGH); digitalWrite(RS1, LOW);
  digitalWrite(LS0, HIGH); digitalWrite(LS1, LOW);

  //ultrasonic sensors
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);// I added this line to account for the other color sensor, it needs to be determined if the color sensor is the fron or the side one 
  pinMode(echoPin2, INPUT);

  state = Choice;

  //Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  switch (state){

    case FindBoundary:
      if(SideUltrasonic() > targetDistance2){
        RotateRt();
        delay(turnTime2);
        Stop();
        state = Choice;
      }
      else if(RightSensor() == 1 && LeftSensor() == 1){
        Stop();
        state = FindGap;
      }
      else{
        Forward();
      }
    break;

    case FindGap:
      if(SideUltrasonic() > targetDistance2){
        Backward();
        delay(waitTime);
        RotateRt();
        delay(turnTime3);
        state = Choice;
      }
      else{
        Backward();
      }
    break;

    case Choice:
      if(FrontUltrasonic() < targetDistance1){
        RotateLft();
        delay(turnTime1);
        Stop();
        state = FindBoundary;
      }
      else if(RightSensor() == 2 && LeftSensor() == 2){
        Stop();
        state = End;
      }
      else{
        Forward();
      }
    break;

    case End:
      Stop();
    break;

    default:
      state = FindBoundary;//Choice;
    break;

  }

}

char RightSensor() {
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

  if(RredColor>200 && RgreenColor>200 && RblueColor>200){ //checks if color is white
    isColorR = 1;
  }
  else if(RredColor<50 && RgreenColor<50 && RblueColor<50){
    isColorR = 0;
  }
  else if(RgreenColor>RredColor && RgreenColor>RblueColor){ //green
    isColorR = 2;
  }
  else{
    isColorR = 0;
  }
  return isColorR;
}

char LeftSensor() { // Uses color sensor to estimate the color of an object
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

  if(LredColor>200 && LgreenColor>200 && LblueColor>200){
    isColorL = 1;
  }
  else if(LgreenColor>LredColor && LgreenColor>LblueColor){ //green
    isColorL = 2;
  }
   else if(LredColor<50 && LgreenColor<50 && LblueColor<50){
    isColorL = 0;
  }
  else{
    isColorL = 0;
  }
  return isColorL;
}

int FrontUltrasonic()
{
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = duration1 * 0.034 / 2;
  if(distance1 <= 1){ //if sensor breaks
    distance1 = 20;
  }
  return distance1;
}

int SideUltrasonic()
{
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = duration2 * 0.034 / 2;
  if(distance2 < 1){ //if sensor breaks
    distance2 = 20;
  }
  return distance2;
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
