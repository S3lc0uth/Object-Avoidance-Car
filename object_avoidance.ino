#include <Servo.h>

// Pin configurations
#define SERVO_PIN 3
#define TRIG_PIN 11
#define ECHO_PIN 12

#define RIGHT_MOTOR_EN 5
#define RIGHT_MOTOR_IN1 7
#define RIGHT_MOTOR_IN2 8

#define LEFT_MOTOR_EN 6
#define LEFT_MOTOR_IN1 9
#define LEFT_MOTOR_IN2 10

// Constants
#define STOP_DISTANCE 20     // Stop if obstacle is within 20 cm
#define SERVO_CENTER 20     // Center position (forward)
#define SERVO_LEFT 70       // Left position (~45° left)
#define SERVO_RIGHT 60     // Right position (~45° right)
#define SCAN_DELAY 300       // Delay after moving servo
#define MOTOR_SPEED 200      // Motor speed (0–255)
#define TURN_TIME 500        // Turn duration in ms

Servo servoMotor;

int distance = 0;
int leftDistance = 0;
int rightDistance = 0;

void setup() {
  Serial.begin(9600);
  
  // Initialize servo
  servoMotor.attach(SERVO_PIN);
  servoMotor.write(SERVO_CENTER); // Face forward
  delay(1000);  // Wait for servo to stabilize

  // Initialize ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize motor pins
  pinMode(RIGHT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
}

void loop() {
  distance = getDistance();
  Serial.print("Front Distance: ");
  Serial.println(distance);

  if (distance <= STOP_DISTANCE) {
    // Obstacle detected — stop and scan
    stopMotors();
    delay(200);

    leftDistance = scanDirection(SERVO_LEFT);
    rightDistance = scanDirection(SERVO_RIGHT);

    // Return to center before deciding
    servoMotor.write(SERVO_CENTER);
    delay(SCAN_DELAY);

    if (leftDistance > rightDistance && leftDistance > STOP_DISTANCE) {
      Serial.println("Turning Left");
      turnLeft();
    } else if (rightDistance > STOP_DISTANCE) {
      Serial.println("Turning Right");
      turnRight();
    } else {
      Serial.println("Backing up and U-turn");
      moveBackward();
      delay(TURN_TIME);
      turnRight(); // You can alternate left/right
      delay(TURN_TIME * 2);
    }
  } else {
    // Path is clear — move forward with servo facing forward
    servoMotor.write(SERVO_CENTER);
    moveForward();
  }

  delay(100); // Stability
}

int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;

  return distance;
}

int scanDirection(int angle) {
  servoMotor.write(angle);
  delay(SCAN_DELAY);
  int d = getDistance();
  Serial.print("Scan @ ");
  Serial.print(angle);
  Serial.print("°: ");
  Serial.print(d);
  Serial.println(" cm");
  return d;
}

// Motor functions
void moveForward() {
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(RIGHT_MOTOR_EN, MOTOR_SPEED);

  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_EN, MOTOR_SPEED);
}

void moveBackward() {
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
  analogWrite(RIGHT_MOTOR_EN, MOTOR_SPEED);

  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  analogWrite(LEFT_MOTOR_EN, MOTOR_SPEED);
}

void turnRight() {
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
  analogWrite(RIGHT_MOTOR_EN, MOTOR_SPEED);

  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_EN, MOTOR_SPEED);

  delay(TURN_TIME);
  stopMotors();
}

void turnLeft() {
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(RIGHT_MOTOR_EN, MOTOR_SPEED);

  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  analogWrite(LEFT_MOTOR_EN, MOTOR_SPEED);

  delay(TURN_TIME);
  stopMotors();
}

void stopMotors() {
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
  analogWrite(RIGHT_MOTOR_EN, 0);

  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_EN, 0);
}
