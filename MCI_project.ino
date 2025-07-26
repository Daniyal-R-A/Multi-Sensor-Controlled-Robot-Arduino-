#define IR_SENSOR_RIGHT 13
#define IR_SENSOR_LEFT 12
#define IR_SENSOR_MID 11

#include <Servo.h> //Servo Motor
int servoPin = 4;
Servo servo;
int angle = 0;

const int trig = 3; //UltraSonic
const int echo = 2;

long duration;
int distance;

int enableRightMotor=5; //Right motor
int IN1=7;              //IN1
int IN2=8;              //IN2
int MOTOR_SPEED1;
int enableLeftMotor=6;  //Left motor
int IN3=10;             //IN3
int IN4=9;              //IN4
int MOTOR_SPEED2;

char state;
char command_detect= 's';
int check=0;

int rightIRSensorValue;
int midIRSensorValue;
int leftIRSensorValue;

void setup()
 {//This sets frequecny as 7812.5 hz.

  servo.attach(servoPin); // ServoMotor
  servo.write(90);

  pinMode(trig,OUTPUT); // UltraSonic
  pinMode(echo,INPUT);
  
  pinMode(enableRightMotor, OUTPUT); // Right Motor
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT); // Left Motor
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(IR_SENSOR_RIGHT, INPUT); // IR sensors
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(IR_SENSOR_MID, INPUT);

  Serial.begin(9600);}



 void loop()
{
  rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
  midIRSensorValue = digitalRead(IR_SENSOR_MID);
  leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);

  ultraSonic();
  Serial.print("Distance : ");
  Serial.println(distance);

  if (distance < 8)
  {
    Serial.print("Avoidance initiated\n");

    obstacle_avoiding();
  }

  else{
  if(midIRSensorValue == HIGH){
    if(leftIRSensorValue == LOW && rightIRSensorValue == LOW){
      straight(108,109);
      Serial.print("mid on , left --> off , right --> off");
      servo.write(0);
    }
    else if(leftIRSensorValue == HIGH && rightIRSensorValue == LOW){
      right(110);
      Serial.print("mid on | left --> on | right --> off");
      servo.write(0);
    }
    else if(leftIRSensorValue == LOW && rightIRSensorValue == HIGH){
      left(110);
      Serial.print("mid on | left --> off | right --> on");
      servo.write(0);
    }
    else if (leftIRSensorValue == HIGH && rightIRSensorValue == HIGH){
      straight(108,109);
      Serial.print("mid on , left --> on , right --> on");  // white -> low, both light;  black -> high, one light
      servo.write(0);
    }
  }
  else{
    if(leftIRSensorValue == HIGH && rightIRSensorValue == LOW){
      right(110);
      servo.write(0);
  }
    else if(leftIRSensorValue == LOW && rightIRSensorValue == HIGH){
      left(110);
      servo.write(0);
    }
    else if(leftIRSensorValue == LOW && rightIRSensorValue == LOW){
      stop();
    }
  }
  }
  
}



void ultraSonic(){
  digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    duration = pulseIn(echo, HIGH);
    distance = duration * 0.034 / 2;
}
void obstacle_avoiding(){

    stop();
    delay(500);
    back(90,90);
    delay(150);
    rotate1(95,95);
    delay(241);
    straight(108,109);
    delay(725);
    rotate2(95,95);
    delay(364);
    
    rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
    midIRSensorValue = digitalRead(IR_SENSOR_MID);
    leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
    while (leftIRSensorValue != HIGH && rightIRSensorValue != HIGH && midIRSensorValue != HIGH)
    {
      rightIRSensorValue = digitalRead(IR_SENSOR_RIGHT);
      midIRSensorValue = digitalRead(IR_SENSOR_MID);
      leftIRSensorValue = digitalRead(IR_SENSOR_LEFT);
      straight(108,109);
    }
}
void straight(int Rspeed,int Lspeed){
  analogWrite(enableRightMotor, abs(Rspeed));
  analogWrite(enableLeftMotor, abs(Lspeed));  
  digitalWrite (IN1,LOW);
  digitalWrite (IN2,HIGH);
  digitalWrite (IN3,LOW);
  digitalWrite (IN4,HIGH);

  
}
void back(int Rspeed,int Lspeed){
  analogWrite(enableRightMotor, abs(Rspeed));
  analogWrite(enableLeftMotor, abs(Lspeed));  
  digitalWrite (IN1,HIGH);
  digitalWrite (IN2,LOW);
  digitalWrite (IN3,HIGH);
  digitalWrite (IN4,LOW);

}
void stop(){ 
  digitalWrite (IN1,LOW);
  digitalWrite (IN2,LOW);
  digitalWrite (IN3,LOW);
  digitalWrite (IN4,LOW);
}
void rotate1(int Rspeed,int Lspeed){
  analogWrite(enableRightMotor, abs(Rspeed));
  analogWrite(enableLeftMotor, abs(Lspeed));  

  // Forward
  digitalWrite (IN1,LOW);
  digitalWrite (IN2,HIGH);
  // Backward
  digitalWrite (IN3,HIGH);
  digitalWrite (IN4,LOW);
}
void rotate2(int Rspeed,int Lspeed){
  analogWrite(enableRightMotor, abs(Rspeed));
  analogWrite(enableLeftMotor, abs(Lspeed));  

  // Forward
  digitalWrite (IN3,LOW);
  digitalWrite (IN4,HIGH);
  // Backward
  digitalWrite (IN1,HIGH);
  digitalWrite (IN2,LOW);
}
void left(int Lspeed){
  analogWrite(enableRightMotor, 0);
  analogWrite(enableLeftMotor, abs(Lspeed));  
    // Forward
  digitalWrite (IN1,LOW);
  digitalWrite (IN2,HIGH);
  // Backward
  digitalWrite (IN3,LOW);
  digitalWrite (IN4,LOW);
}
void right(int Rspeed){
  analogWrite(enableRightMotor, abs(Rspeed));
  analogWrite(enableLeftMotor, 0);  
    // Forward
  digitalWrite (IN1,LOW);
  digitalWrite (IN2,LOW);
  // Backward
  digitalWrite (IN3,LOW);
  digitalWrite (IN4,HIGH);
}
