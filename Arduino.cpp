// === Motor Pins ===
const int ENA = 9;     // PWM - Left motor
const int IN1 = 8;
const int IN2 = 7;


const int ENB = 10;    // PWM - Right motor
const int IN3 = 6;
const int IN4 = 5;


// === IR Sensor Pins ===
const int irPins[5] = {A0, A1, A2, A3, A4};
int irValues[5];


// === PID Variables ===
float Kp = 60 ;   // Proportional
float Ki = 0.005;  // Integral
float Kd = 10;   // Derivative


float error = 0, previousError = 0, integral = 0, derivative = 0;
int baseSpeed = 150; // Adjust as needed (0-255)


// === Sensor Weights for Position Calculation ===
int weights[5] = {2.5, 1.5, 0, -1.5, -2.5}; // For position error calculation


void setup() {
  // Motor pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);


  // IR sensor pins
  for (int i = 0; i < 5; i++) pinMode(irPins[i], INPUT);


  Serial.begin(9600);
}


void loop() {
  readIRSensors();


  // Calculate weighted position error
  int position = 0;
  int totalActive = 0;
  for (int i = 0; i < 5; i++) {
    if (irValues[i] == 1) {
      position += weights[i];
      totalActive++;
    }
  }


  if (totalActive > 0) {
    error = (float)position / totalActive;
  } else {
    // Line lost: use previous error direction
    if (previousError > 0) error = 2.5;
    else error = -2.5;
  }


  // PID calculations
  integral += error;
  derivative = error - previousError;
  float correction = Kp * error + Ki * integral + Kd * derivative;


  // Motor speeds
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;


  // Clamp speeds to 0-255
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);


  // Drive motors
  driveMotors(leftSpeed, rightSpeed);


  previousError = error;
  //delay(1); // Small delay for stability
}


void readIRSensors() {
  for (int i = 0; i < 5; i++) {
    int sensorReading = analogRead(irPins[i]);
   
    if (sensorReading < 300) {
      irValues[i] = 0; // WHITE line detected
    } else {
      irValues[i] = 1; // BLACK surface
    }
  }
}




void driveMotors(int leftSpeed, int rightSpeed) {
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, leftSpeed);


  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, rightSpeed);
}
