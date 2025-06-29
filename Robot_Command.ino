#include <Servo.h>
#include <PID_v1.h>
#define IN1 11 // Left Front
#define IN2 10 // Left Back
#define IN3 4  // Right Front
#define IN4 5  // Right Back
#define SERVOPIN 6
#define trigPin 12
#define echoPin 13

Servo servo;
bool leftChecked = false;
bool rightChecked = false;

// Define PID variables
double Setpoint, Input, Output;
double Kp = 2.0, Ki = 5.0, Kd = 1.0;  // Example PID constants

// Create PID instance
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void servoRight() {
  servo.write(180);
  delay(1000);
}

void resetServo() {
  servo.write(90);
  delay(1000);
}

void servoLeft() {
  servo.write(0);
  delay(1000);
}

void resetWheels() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void goForward() {
  resetWheels();
  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, HIGH);
}

void goBackwards() {
  resetWheels();
  digitalWrite(IN2, HIGH);
  digitalWrite(IN4, HIGH);
}

void turnRight() {
  resetWheels();
  digitalWrite(IN3, HIGH);
  delay(500);
}

void turnLeft() {
  resetWheels();
  digitalWrite(IN1, HIGH);
  delay(500);
}

int getDistance() {
  long duration;
  int distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;
  delay(300);
  return distance;
}

void setupWheels() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  resetWheels();
}

void setupServo() {
  servo.attach(SERVOPIN);
  servo.write(90);
}

void setupUltrasonic() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void setup() {
  setupWheels();
  setupServo();
  setupUltrasonic();
  // Serial.begin(9600);

  // Initialize the PID controller
  Setpoint = 30;  // Target distance (for example, set to 30 cm)
  myPID.SetMode(AUTOMATIC);  // Start the PID controller
  
}

void loop() {
  int distance = getDistance();
  //Serial.print("Distance: ");
  //Serial.println(distance);

  // Update the PID input with the current distance
  Input = distance;

  // Compute the PID output
  myPID.Compute();

  // Output the PID value
  //Serial.print("PID Output: ");
  //Serial.println(Output);

  if (distance > 30) {
    // If no obstacles, continue moving forward
    // //Serial.println("Moving Forward");
    goForward();
    leftChecked = false;
    rightChecked = false;
  } else {
    resetWheels();
    //Serial.println("Obstacle detected! Stopping...");
    resetServo();
    servoLeft();
    int leftDistance = getDistance();
    //Serial.print("Left Distance: ");
    //Serial.println(leftDistance);

    resetServo();
    servoRight();
    int rightDistance = getDistance();
    //Serial.print("Right Distance: ");
    //Serial.println(rightDistance);

    resetServo();

    // If both sides are blocked, go backwards for 1 second
    while (leftDistance < 30 && rightDistance < 30) {
      //Serial.println("Both sides blocked! Reversing...");
      goBackwards();
      delay(1000); // Move backwards for 1 second
      resetWheels();
      resetServo();
      servoLeft();
      leftDistance = getDistance();
      //Serial.print("Left Distance: ");
      //Serial.println(leftDistance);
      resetServo();
      servoRight();
      rightDistance = getDistance();
      // //Serial.print("Right Distance: ");
      // //Serial.println(rightDistance);
      resetServo();
    }

    if (rightDistance > leftDistance) {
      int dist = getDistance();
      //Serial.print("Current distance: ");
      //Serial.print(dist);
      while (dist < 30) {  // Keep turning right until the path is clear
        //Serial.println("Turning Right...");
        turnRight();
        dist = getDistance();
      }
    } else {
      int dist = getDistance();
      //Serial.print("Current distance: ");
      //Serial.print(dist);
      while (dist < 30) {  // Keep turning left until the path is clear
        //Serial.println("Turning Left...");
        turnLeft();
        dist = getDistance();
      }
    }

    // After turning, continue forward
    //Serial.println("Path clear, moving forward...");
    goForward();
    delay(500);
  }
}
