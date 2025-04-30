#include <SoftwareSerial.h>
#include <Servo.h>

#define Lpwm_pin  5     //pin of controlling speed---- ENA of motor driver board
#define Rpwm_pin  6    //pin of controlling speed---- ENB of motor driver board
int pinLB=2;             //pin of controlling turning---- IN1 of motor driver board
int pinLF=4;             //pin of controlling turning---- IN2 of motor driver board
int pinRB=7;            //pin of controlling turning---- IN3 of motor driver board
int pinRF=8;            //pin of controlling turning---- IN4 of motor driver board
#define TRIG_PIN A0
#define ECHO_PIN A1


Servo myservo;
volatile int DL;
volatile int DM;
volatile int DR;

SoftwareSerial espSerial(0, 12); // RX = D2, TX = D12 (TX unused)
const double targetRSSI = 40; // ideal proximity
int rssi = 0;
int leftRSSI, rightRSSI;
const double kp = 2.0;
const double ki = 0.01;
const double kd = 0.5;

double kiTotal = 0.0;
double prevError = 0.0;
double pidSum = 0.0;

// --- Movement Parameters ---
const unsigned long moveDuration = 500; // milliseconds
const unsigned long turnDuration = 300;

void setup() {
  Serial.begin(9600);
  espSerial.begin(9600);

  pinMode(pinLB,OUTPUT); // /pin 2
  pinMode(pinLF,OUTPUT); // pin 4
  pinMode(pinRB,OUTPUT); // pin 7
  pinMode(pinRF,OUTPUT);  // pin 8
  pinMode(Lpwm_pin,OUTPUT);  // pin 5 (PWM) 
  pinMode(Rpwm_pin,OUTPUT);  // pin6(PWM)
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);



  //obsacle avoidance setup
  myservo.attach(A2);
  pinMode(A1, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(pinLB,OUTPUT); // /pin 2
  pinMode(pinLF,OUTPUT); // pin 4
  pinMode(pinRB,OUTPUT); // pin 7
  pinMode(pinRF,OUTPUT);  // pin 8
  pinMode(Lpwm_pin,OUTPUT);  // pin 5 (PWM) 
  pinMode(Rpwm_pin,OUTPUT);  // pin6(PWM) 
  DL = 0;
  DM = 0;
  DR = 0;
  myservo.write(90);

  //Serial.println("PID BLE Beacon Follower Ready...");
}

void loop() {
  DM = checkdistance();

  // PHASE 1: Immediate Obstacle in Front
  if (DM < 30) {
    handleObstacleAvoidance();
    return; // Start loop again after obstacle avoidance
  }

  // PHASE 2: Check Beacon RSSI
  stopp(); // stop before scanning
  int determineStopRSSI = median_rssi(90);
  calc_PID();

  int baseSpeed = 130;
  int speed = constrain(baseSpeed - abs(pidSum), 100, baseSpeed); 

  // PHASE 3: Decision Based on RSSI
  if (rssi == 999) {
    Serial.println("Beacon not found.");
    advance();
    kiTotal = 0;
    delay(1000);
    return;
  }

  if (rssi <= targetRSSI) {
    Serial.println("Beacon very close, stopping.");
    stopp();
    kiTotal = 0;
    delay(1000);
    return;
  }

  // PHASE 4: Scan and Turn Toward Beacon
  leftRSSI = median_rssi(150);
  myservo.write(90);
  rightRSSI = median_rssi(30);
  myservo.write(90);

  Serial.print("Left RSSI: "); Serial.println(leftRSSI);
  Serial.print("Right RSSI: "); Serial.println(rightRSSI);

  if (leftRSSI < rightRSSI) {
    Serial.println("Beacon is to the LEFT");
    turnL();
    delay(300);
  } 
  else if (rightRSSI < leftRSSI) {
    Serial.println("Beacon is to the RIGHT");
    turnR();
    delay(300);
  }

  // PHASE 5: Move Toward Beacon, but stop if obstacle appears
  advance();
  Set_Speed(speed);

  unsigned long moveStart = millis();
  while (millis() - moveStart < 3000) {
    DM = checkdistance();
    if (DM < 30) {
      Serial.println("Obstacle detected during forward movement");
      stopp();
      Set_Speed(0);
      handleObstacleAvoidance(); // respond to obstacle mid-way
      return;
    }
    delay(50);
  }
}

void collect_rssi(){
  if (espSerial.available()) {
    rssi = espSerial.readStringUntil('\n').toInt();
    rssi = rssi*-1; //turn the negative numbers positive
    return;
  }
}

int average_rssi(int angle) {
  myservo.write(angle);
  delay(1000); // settle

  int sum = 0;
  for (int i = 0; i < 15; i++) {
    collect_rssi();
    sum += rssi;
    delay(300);
  }
  return sum / 15;
}

int median_rssi(int angle) {
  myservo.write(angle);
  delay(1000); // allow the servo to settle

  const int numSamples = 15;
  int readings[numSamples];

  // Collect RSSI values
  for (int i = 0; i < numSamples; i++) {
    collect_rssi();
    readings[i] = rssi;
    delay(300);
  }

  // Sort the array
  for (int i = 0; i < numSamples - 1; i++) {
    for (int j = 0; j < numSamples - i - 1; j++) {
      if (readings[j] > readings[j + 1]) {
        int temp = readings[j];
        readings[j] = readings[j + 1];
        readings[j + 1] = temp;
      }
    }
  }

  // Return the median value
  return readings[numSamples / 2];
}

void calc_PID(){
  double error = targetRSSI - rssi;
  double proportional = kp * error;
  kiTotal += error;
  kiTotal = constrain(kiTotal, -100, 100); // prevent integral windup
  double integral = ki * kiTotal;
  double derivative = kd * (error - prevError);
  prevError = error;

  pidSum = proportional + integral + derivative;

  //Serial.print("Error: "); Serial.print(error);
  //Serial.print(" | P: "); Serial.print(proportional);
  //Serial.print(" I: "); Serial.print(integral);
  //Serial.print(" D: "); Serial.print(derivative);
  //Serial.print(" | PID Sum: "); Serial.println(pidSum);
}

// ------------------ Motor Functions ------------------
void Set_Speed(unsigned char pwm) //function of setting speed
{
  analogWrite(Lpwm_pin,pwm);
  analogWrite(Rpwm_pin,pwm);
}

void advance()    //  going forward
{
  digitalWrite(pinRB,LOW);  // making motor move towards right rear
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,LOW);  // making motor move towards left rear
  digitalWrite(pinLF,HIGH); 

}
void turnR()        //turning right(dual wheel)
{
  digitalWrite(pinRB,LOW);  //making motor move towards right rear
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,LOW);  //making motor move towards left front

}
void turnL()         //turning left(dual wheel)
{
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,LOW );   //making motor move towards right front
  digitalWrite(pinLB,LOW);   //making motor move towards left rear
  digitalWrite(pinLF,HIGH);

}    
void stopp()        //stop
{
  digitalWrite(pinRB,HIGH);
  digitalWrite(pinRF,HIGH);
  digitalWrite(pinLB,HIGH);
  digitalWrite(pinLF,HIGH);

}
void back()         //back up
{
  digitalWrite(pinRB,HIGH);  //making motor move towards right rear     
  digitalWrite(pinRF,LOW);
  digitalWrite(pinLB,HIGH);  //making motor move towards left rear
  digitalWrite(pinLF,LOW);
  
}

float checkdistance() {
  digitalWrite(A1, LOW);
  delayMicroseconds(2);
  digitalWrite(A1, HIGH);
  delayMicroseconds(10);
  digitalWrite(A1, LOW);
  float distance = pulseIn(A0, HIGH) / 58.00;
  delay(10);
  return distance;
}

void Detect_obstacle_distance() {
  myservo.write(160);
  for (int i = 0; i < 3; i = i + 1) {
    DL = checkdistance();
    delay(100);
  }
  myservo.write(20);
  for (int i = 0; i<3; i = i + 1) {
    DR = checkdistance();
    delay(100);
  }
}

void handleObstacleAvoidance() {
  stopp();
  Set_Speed(0);
  delay(500);
  Detect_obstacle_distance();

  if (DL < 50 || DR < 50) {
    if (DL > DR) {
      Serial.println("Turning left to avoid obstacle");
      turnL();
    } else {
      Serial.println("Turning right to avoid obstacle");
      turnR();
    }
  } else {
    if (random(1, 10) > 5) {
      Serial.println("Random: turning left");
      turnL();
    } else {
      Serial.println("Random: turning right");
      turnR();
    }
  }

  Set_Speed(200);
  delay(400);
  stopp();
}