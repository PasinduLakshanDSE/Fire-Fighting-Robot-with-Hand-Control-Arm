#include <Servo.h>

Servo servo1, servo2, servo3, servo4;  // Define servos

void setup() {
  Serial.begin(115200);  // Start serial communication at 115200 baud
  servo1.attach(5);  // Attach servos to pins
  servo2.attach(6);
  servo3.attach(7);
  servo4.attach(8);
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();  // Read incoming command
    switch (command) {
      case '1':  // Command to move servo1
        servo1.write(90);  // Move servo to 90 degrees
        break;
      case '2':  // Command to move servo2
        servo2.write(90);
        break;
      case '3':  // Command to move servo3
        servo3.write(90);
        break;
      case '4':  // Command to move servo4
        servo4.write(90);
        break;
      case '0':  // Reset all servos to default position
        servo1.write(0);
        servo2.write(0);
        servo3.write(0);
        servo4.write(0);
        break;
      default:  // Handle unknown commands
        Serial.println("Invalid command received");
        break;
    }
  }
}
