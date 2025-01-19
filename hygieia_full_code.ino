// Libraries
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

//define connection for three (front, left, right) sonar sensors pins
#define trigPinf 3
#define echoPinf 2
#define trigPinl 22
#define echoPinl 23
#define trigPinr 24
#define echoPinr 25

const int relayPin = 41;  //Relay setup
Servo myservo;   // servo
Adafruit_MPU6050 mpu;  //gyro sensor

float distancef;
float distancel;
float distancer;
float z_angle = 0;
int count = 0;
int corner = 0;

// motors
int ENA = 5; //regulates duty cycle
int ENB = 6; //regulates duty cycle
int spd = 100; // range: 0 to 255
float rot = 86;
int mop_cw = 28; //left mopping motor
int mop_ccw = 29; //right mopping motor
int mop_speed = 11;
int motorRightA = 7; // Right motor forward
int motorRightB = 8; // Right motor backward
int motorLeftA = 9; // Left motor forward
int motorLeftB = 10; // Left motor backward

void forward() // move forward
{
  digitalWrite(motorRightA, HIGH);
  digitalWrite(motorLeftA, HIGH);
  digitalWrite(motorRightB, LOW);
  digitalWrite(motorLeftB, LOW);
}

void stp() // stop
{
  digitalWrite(motorRightA, LOW);
  digitalWrite(motorLeftA, LOW);
  digitalWrite(motorRightB, LOW);
  digitalWrite(motorLeftB, LOW);
}

void backward() //move backward
{
  digitalWrite(motorRightA, LOW);
  digitalWrite(motorLeftA, LOW);
  digitalWrite(motorRightB, HIGH);
  digitalWrite(motorLeftB, HIGH);
}

void right() // turn right
{
  analogWrite(ENA, 140);
  analogWrite(ENB, 140);
  digitalWrite(motorRightA, LOW);
  digitalWrite(motorLeftA, HIGH);
  digitalWrite(motorRightB, HIGH);
  digitalWrite(motorLeftB, LOW);
}

void left() // turn left
{
  analogWrite(ENA, 140);
  analogWrite(ENB, 140);
  digitalWrite(motorRightA, HIGH);
  digitalWrite(motorLeftA, LOW);
  digitalWrite(motorRightB, LOW);
  digitalWrite(motorLeftB, HIGH);
}

int calcdisf() // measures the distance ahead from any obstacle
{
  float duration, cm;
  delay(70);
  digitalWrite(trigPinf, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinf, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinf, LOW);

  duration = pulseIn(echoPinf, HIGH);

  cm = (duration / 2) * 0.0343;
  return cm;
}

int calcdisl() // measures the distance in left from any obstacle
{
  float duration, cm;
  delay(70);
  digitalWrite(trigPinl, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinl, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinl, LOW);

  duration = pulseIn(echoPinl, HIGH);

  cm = (duration / 2) * 0.0343;
  return cm;
}

int calcdisr() // measures the distance in right from any obstacle
{
  float duration, cm;
  delay(70);
  digitalWrite(trigPinr, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinr, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinr, LOW);

  duration = pulseIn(echoPinr, HIGH);

  cm = (duration / 2) * 0.0343;
  return cm;
}

float angle() // measures the turning angle in degrees
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float Z_rotation;
  static unsigned long prevTime = 0;
  unsigned long currentTime = millis();
  float dt = (float)(currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  Z_rotation = (g.gyro.z + 0.012) * 180 / 3.1416;

  z_angle = (z_angle + Z_rotation * dt);
  return z_angle;
}

void zigzag() // function for taking U-turn
{
    //first turns left or right
    z_angle = 0;
    if (count % 2 == 0) {
      left();
    }
    else {
      right();
    }
    while (abs(angle()) <= rot)
    {
      if (count % 2 == 0) {
        left();
      }
      else {
        right();
      }
      if (abs(angle()) > rot)
        break;
    }
    z_angle = 0;
    stp();
    
    //Then goes forward for a little while
    forward();
    delay(350);
    stp();
    
    //detects if there is a corner
    distancef = calcdisf();
    distancel = calcdisl();
    distancer = calcdisr();
    if((distancef<30)&&((distancel<30)||(distancer<30)))
    {
      corner=corner+1;
    }

    //Then again turns left or right
    if (count % 2 == 0) {
      left();
    }
    else {
      right();
    }
    while (abs(angle()) <= rot)
    {
      if (count % 2 == 0) {
        left();
      }
      else {
        right();
      }
      if (abs(angle()) > rot)
        break;
    }
    z_angle = 0;
    stp();
    forward();
    count = count + 1;
    digitalWrite(relayPin, LOW); //throws water and disinfectant
    delay(80);
    digitalWrite(relayPin,HIGH);
}

int lookRight() // front sonar turns around 45 degrees right and measures distance
{
    myservo.write(72); 
    delay(200);
    int distance = calcdisf();
    delay(100);
    myservo.write(122);
    return distance;
}
int lookLeft()// front sonar turns around 45 degrees left  and measures distance
{
    myservo.write(172); 
    delay(200);
    int distance = calcdisf();
    delay(100);
    myservo.write(122); 
    return distance;
}

void mop() //rotation of two mops
{
  analogWrite(mop_speed, 80);
  digitalWrite(mop_cw, HIGH);
  digitalWrite(mop_ccw, LOW);
}

void stop_mop()  //stops rotation of mops
{
  digitalWrite(mop_cw, LOW);
  digitalWrite(mop_ccw, LOW);
}
void setup()
{
  //sonar
  pinMode(trigPinf, OUTPUT);
  pinMode(echoPinf, INPUT);
  pinMode(trigPinl, OUTPUT);
  pinMode(echoPinl, INPUT);
  pinMode(trigPinr, OUTPUT);
  pinMode(echoPinr, INPUT);
  //motors
  pinMode(motorRightA, OUTPUT);
  pinMode(motorRightB, OUTPUT);
  pinMode(motorLeftA, OUTPUT);
  pinMode(motorLeftB, OUTPUT);
  pinMode(relayPin, OUTPUT);
  
  Serial.begin(115200);
  myservo.attach(4);  
  myservo.write(122);

  digitalWrite(relayPin, HIGH);
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(100);
    }
  }

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
}

void loop()
{
  float distanceR = 0;
  float distanceL =  0;
  distancef = calcdisf(); //calculates forward distance

  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
  
  if(corner<3) //normally roams if endpoint not reached (at endpoint, corner=3)
  {
    forward();
    mop();
  }
  else  //if endpoint reached, all operations stop
  {
    stp();
    stop_mop();
  }
  
  if (distancef < 30) //front sensor detects something
  {
    stp();
    delay(100);
    distancel = calcdisl();
    distancer = calcdisr();
    
    if((distancel<30)||(distancer<30)) //if this is a corner of the room
    {
      corner=corner+1;
    }
    distanceR=lookRight();
    delay(200);
    distanceL = lookLeft();
    delay(200);
    
    if((distanceR<100)&&(distanceL<100)) //if the object is a wall
    {
      if(corner<3)
      {
        zigzag(); //takes a U-turn
      }
      else
      {
        stp();
      }
    }
    else //the object is an obstacle
    {
      //turns left at first
      z_angle=0;
      left(); 
      while (abs(angle()) <= rot)
      {
        left();
        if (abs(angle()) > rot)
          break;
      }//now right side of the bot is facing the obstacle
      
      //goes forward and tracks how much time needed to cross the length of the obstacle
      unsigned long start_time = millis(); 
      forward();
      delay(100);
      while(calcdisr()<20)
      {
        forward();
        if(calcdisr()>20)
          break;
      }
      unsigned long real_time = millis();
      
      //turns right
      z_angle=0;
      right(); 
      while (abs(angle()) <= rot)
      {
        right();
        if (abs(angle()) > rot)
          break;
      }

      //again goes forward untill obstacle is crossed
      forward(); 
      delay(500);
     while(calcdisr()<20)
      {
        forward();
        if(calcdisr()>20)
          break;
      }

      //takes turn at right
      z_angle=0;
      right(); 
      while (abs(angle()) <= rot)
      {
        right();
        if (abs(angle()) > rot)
          break;
      }
      
      //goes forward for the time recorded
      forward(); 
      delay(real_time - start_time);
      
      //takes final turn at left
      z_angle=0;
      left(); 
      while (abs(angle()) <= rot)
      {
        left();
        if (abs(angle()) > rot)
          break;
      }
      //successfully avoids the obstacle
      
      forward();  //resumes normal navigation
    }
  }
}
