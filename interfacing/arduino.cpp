#include <Wire.h>
#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;

int MIN_POSITION = 0;
int MAX_POSITION = 180;
int STEP = 10;

// pins for the servos
const int servo1Pin = 9;
const int servo2Pin = 10;
const int servo3Pin = 11;

// servo positions
int position1 = 90;
int position2 = 90;
int position3 = 90;

void setup() {
  // I2C address to match the Python code
  Wire.begin(0x8);
  Wire.onReceive(receiveEvent);

  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);

  servo1.write(position1);
  servo2.write(position2);
  servo3.write(position3);
}

void loop() {}

void receiveEvent(int howMany) {
  if (howMany >= 2) {
    int servoNumber = Wire.read();
    int direction = Wire.read();

    int *positionPtr = nullptr;
    Servo *servoPtr = nullptr;

    switch (servoNumber) {
      case 1:
        servoPtr = &servo1;
        positionPtr = &position1;
        break;
      case 2:
        servoPtr = &servo2;
        positionPtr = &position2;
        break;
      case 3:
        servoPtr = &servo3;
        positionPtr = &position3;
        break;
      default:
        return;
    }

    // Adjust position based on direction
    if (direction == 1) {
      // Move up
      *positionPtr += STEP;
      if (*positionPtr > MAX_POSITION) *positionPtr = MAX_POSITION;
    } else if (direction == 0) {
      // Move down
      *positionPtr -= STEP;
      if (*positionPtr < MIN_POSITION) *positionPtr = MIN_POSITION;
    }

    servoPtr->write(*positionPtr);
  }
}
