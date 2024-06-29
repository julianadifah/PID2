// Define pin numbers
const int trigPin = 9;
const int echoPin = 10;
const int motorPin1 = 2;
const int motorPin2 = 3;
const int motorPin3 = 4;
const int motorPin4 = 5;
const int enablePinA = 6;
const int enablePinB = 7;

// PID constants
float kp = 2.0;
float ki = 0.5;
float kd = 1.0;

// PID variables
float previousError = 0;
float integral = 0;

// Setpoint (desired distance in cm)
float setpoint = 200.0;

void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(enablePinA, OUTPUT);
  pinMode(enablePinB, OUTPUT);
}

void loop() {
  // Read distance from ultrasonic sensor
  float distance = getDistance();

  if (distance <= setpoint) {
    // If an obstacle is detected within the set distance, turn right
    turnRight();
    delay(500); // Turn right for 500ms (adjust as needed)
  } else {
    // Calculate error
    float error = setpoint - distance;

    // Calculate integral
    integral += error;

    // Calculate derivative
    float derivative = error - previousError;

    // Calculate control variable
    float output = kp * error + ki * integral + kd * derivative;

    // Update previous error
    previousError = error;

    // Control motors
    controlMotors(output);

    // Print distance and output for debugging
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print(" cm, Output: ");
    Serial.println(output);
  }

  // Delay for stability
  delay(100);
}

float getDistance() {
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Set the trigPin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance
  float distance = (duration * 0.034) / 2;

  return distance;
}

void controlMotors(float output) {
  // Ensure output is within bounds
  output = constrain(output, -255, 255);

  if (output > 0) {
    // Move forward
    analogWrite(enablePinA, output);
    analogWrite(enablePinB, output);
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, HIGH);
    digitalWrite(motorPin4, LOW);
  } else {
    // Move backward
    analogWrite(enablePinA, -output);
    analogWrite(enablePinB, -output);
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, HIGH);
  }
}

void turnRight() {
  // Set motors to turn right
  analogWrite(enablePinA, 200); // Set speed for right turn (adjust as needed)
  analogWrite(enablePinB, 200); // Set speed for right turn (adjust as needed)
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH);
}
