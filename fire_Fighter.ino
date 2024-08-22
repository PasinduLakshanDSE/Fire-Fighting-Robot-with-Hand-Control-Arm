#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MLX90614.h>
#include <SoftwareSerial.h>

// Define Ultrasonic Sensor Pins
#define TRIG_PIN A3
#define ECHO_PIN A2

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

// Define pins for flame sensors
#define FLAME_SENSOR_LEFT 4
#define FLAME_SENSOR_RIGHT 12
#define FLAME_SENSOR_FRONT 11

#define LM1 6  // Left motor forward
#define LM2 7  // Left motor backward
#define RM1 8  // Right motor forward
#define RM2 9  // Right motor backward
#define PUMP_PIN A4 // Water pump

#define enA 5  // Enable pin for left motor
#define enB 10 // Enable pin for right motor

#define BUZZER_PIN 3  // Buzzer pin

#define BLUETOOTH_RX 13
#define BLUETOOTH_TX 2

#define SERVO_PIN A0 // Servo motor pin

// Distance threshold to stop the robot (in cm)
#define DISTANCE_THRESHOLD 20

Servo myServo;

void setup() {
  // Initialize serial communication for Bluetooth
  Serial.begin(9600);

  // Motor pins setup
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);

  // Sensors and actuators setup
  pinMode(FLAME_SEN  SOR_LEFT, INPUT);
  pinMode(FLAME_SENSOR_RIGHT, INPUT);
  pinMode(FLAME_SENSOR_FRONT, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);

  // Ultrasonic sensor setup
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Initialize actuators to be off
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(PUMP_PIN, LOW);

  // Initialize MLX90614 sensor
  mlx.begin();

  // Initialize servo motor
  myServo.attach(SERVO_PIN);
  myServo.write(90); // Set initial position to 90 degrees
}

void loop() {
  // Read flame sensor values (digital)
  bool leftFlame = digitalRead(FLAME_SENSOR_LEFT);
  bool rightFlame = digitalRead(FLAME_SENSOR_RIGHT);
  bool frontFlame = digitalRead(FLAME_SENSOR_FRONT);

  // Measure distance using the ultrasonic sensor
  long distance = measureDistance();  // This will also print the distance to the Serial Monitor

  // Robot movement logic
  if (distance <= DISTANCE_THRESHOLD) {  // Obstacle detected within threshold
    stopRobot();
    if (!frontFlame) {  // Flame detected in front after stopping
      extinguishFire();
    }
  } else if (!frontFlame) {  // Flame detected in front
    moveForward();
  } else if (!leftFlame) {  // Flame detected on the left
    moveRight();
  } else if (!rightFlame) {  // Flame detected on the right
    moveLeft();
  } else {  // No flame detected
    stopRobot();
  }

  // Optional: Add Bluetooth control if needed
  if (Serial.available()) {
    char command = Serial.read();
    controlRobotViaBluetooth(command);
  }
}

void moveForward() {
  Serial.println("Moving Forward");
  analogWrite(enA, 120); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
  analogWrite(enB, 120);
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, HIGH);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, HIGH);
}

void moveLeft() {
  Serial.println("Turning Left");
  analogWrite(enA, 200); // Write The Duty Cycle 0 to 255 Enable Pin A for Motor1 Speed 
  analogWrite(enB, 200);
  digitalWrite(LM1, LOW); // Left Motor backward Pin 
  digitalWrite(LM2, LOW); // Left Motor forward Pin 
  digitalWrite(RM1, LOW); // Right Motor forward Pin 
  digitalWrite(RM2, HIGH); // Right Motor backward Pin 
}

void moveRight() {
  Serial.println("Turning Right");
  analogWrite(enA, 200); // Reduce speed for turning
  analogWrite(enB, 200); // Reduce speed for turning
  digitalWrite(LM1, LOW); // Left Motor backward Pin 
  digitalWrite(LM2, HIGH); // Left Motor forward Pin 
  digitalWrite(RM1, LOW); // Right Motor forward Pin 
  digitalWrite(RM2, LOW); // Right Motor backward Pin 
}

void stopRobot() {
  Serial.println("Stopping Robot");
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, LOW);
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, LOW);
}

void extinguishFire() {
  Serial.println("Extinguishing Fire");

  // Activate buzzer and pump for fire extinguishing
  digitalWrite(BUZZER_PIN, HIGH);
  digitalWrite(PUMP_PIN, HIGH);

  // Rotate the servo motor while spraying water
  for (int angle = 0; angle <= 180; angle += 10) {
    myServo.write(angle);  // Rotate servo to the specified angle
    delay(200);            // Wait for the servo to reach the position
  }
  
  // Reverse the rotation
  for (int angle = 180; angle >= 0; angle -= 10) {
    myServo.write(angle);
    delay(200);
  }

  // Turn off pump and buzzer
  digitalWrite(PUMP_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  myServo.write(90); // Reset servo to neutral position (optional)
}

void controlRobotViaBluetooth(char command) {
  switch (command) {
    case 'F': moveForward(); break;
    case 'L': moveLeft(); break;
    case 'R': moveRight(); break;
    case 'S': stopRobot(); break;
    default: stopRobot(); break;
  }
}

long measureDistance() {
  // Trigger the ultrasonic pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the duration of the echo pulse
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance in cm (speed of sound = 34300 cm/s)
  long distance = duration * 0.034 / 2;

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}
