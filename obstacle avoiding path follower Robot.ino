#include <NewPing.h>
#include <Servo.h>
#include <AFMotor.h>

//Ultrasonic Sensor
#define TRIG_PIN 9  
#define ECHO_PIN 10
#define MAX_DISTANCE 200  // in centimeter
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);

//ir sensor
#define irLeft A0
#define irRight A5


//motor
#define MAX_SPEED 200
#define MAX_SPEED_OFFSET 20

int distance ;
int leftDistance;
int rightDistance;
boolean object;
Servo servo;


AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(2, MOTOR12_1KHZ);
AF_DCMotor motor3(3, MOTOR34_1KHZ);
AF_DCMotor motor4(4, MOTOR34_1KHZ);

 
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
  

  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);
  
  
 servo.attach(10);
  servo.write(90); 

  motor1.setSpeed(80);
  motor2.setSpeed(80);
  motor3.setSpeed(80);
  motor4.setSpeed(80);
}

//....................................LOOPING.........................................

void loop() {
pinMode(9, OUTPUT);
  pinMode(10, INPUT);

  
 // put your main code here, to run repeatedly:
if (digitalRead(irLeft) == 0 && digitalRead(irRight) == 0 ) {     // for moving forward
     Serial.println("forward data by ir");
     objectAvoid();

  }
  else if (digitalRead(irLeft) == 0 && digitalRead(irRight) == 1 ) {    //for left turn
    
     Serial.println("Left turn data ir");
    moveLeft();
     objectAvoid();
  }
  else if (digitalRead(irLeft) == 1 && digitalRead(irRight) == 0 ) {    //for right turn
    Serial.println("right turn data ir");
    moveRight();
    objectAvoid();
  }
  else if (digitalRead(irLeft) == 1 && digitalRead(irRight) == 1 ) {    // stop case
  Serial.println("stopped data ir");
  Stop();
  }
// getDistance();

}


//.....................MOVEMENT ............................................................
void moveRight() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
   
}
void moveLeft() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}
 
void Stop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
 
}
void moveForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}
void moveBackward() {
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
  
}


//................................GET DISTANCE ....................................................

int getDistance() {
//Serial.println(sonar.ping_cm());
  delay(50);
  int cm = sonar.ping_cm();
  if (cm >= 100){
    cm = 200;
  }

 // Serial.println(cm);
  return cm;
}

//.................................... OBJECT AVOID .........................................

void objectAvoid() {
  //distance = getDistance();
  Serial.println("obstacle avoid ko mathi");
  Serial.println(getDistance());
  Serial.println("obstacle avoid ko tala");
  if (getDistance() <= 15)  {
    //stop
    Stop(); 
    Serial.println("stoped");

    lookLeft();
    delay(100);
    lookRight();
    delay(100);
  
    if (rightDistance <= leftDistance) {
      //left
      object = true;
      turn();
      } else {
      //right
      object = false;
      turn();
    }
    delay(100);
  }
  else {
    //forword
    moveForward();
  }
}

//..............................look LEFT AND RIGHT ....................................................

int lookLeft () {
  //lock left
  servo.write(150);
  delay(1000);
  leftDistance =  getDistance();
  delay(500);
  servo.write(90);

  Serial.print("Left:");
  Serial.print(leftDistance);
  Serial.println();
  return leftDistance;
  delay(500);
}

int lookRight() {
  servo.write(30);
  delay(1000);
  rightDistance = getDistance();
  delay(500);
  servo.write(90);
   Serial.print("Right:");
  Serial.println(rightDistance);
  return rightDistance;
  delay(500);
}

//..................................TURN............................................................
void turn() {
  if (object == false) {
    Serial.println("turn Right");
    moveLeft();
    delay(1000);
    moveForward();
    delay(1200);
    moveRight();
    delay(1400);
    //servo.write(30); // additional
    if (digitalRead(irRight) == 1) {
      loop();
    } else {
      moveForward();
    }
  }
  else {
    Serial.println("turn left");
    moveRight();
    delay(1000);
    moveForward();
    delay(1200);
    moveLeft();
    delay(1400);
    //servo.write(30); // additional
    if (digitalRead(irLeft) == 1) {
      loop();
    } else {
      moveForward();
    }
  }
}








