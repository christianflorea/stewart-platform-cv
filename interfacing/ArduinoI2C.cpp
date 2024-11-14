#include <Wire.h>
#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;

// ABSOLUTE_MIN is the minimum position for the servos to prevent platform damage
const int ABSOLUTE_MIN = 85;
const int MAX_POSITION = 180;
const byte STEP = 0x2;

const int servo1Pin = 5;
const int servo2Pin = 7;
const int servo3Pin = 6;

const int limitSwitch1Pin = 2;
const int limitSwitch2Pin = 3;
const int limitSwitch3Pin = 4;

int position1 = 135;
int position2 = 135;
int position3 = 135;

// Minimum positions for servos (to be determined during homing)
int minPosition1 = ABSOLUTE_MIN;
int minPosition2 = ABSOLUTE_MIN;
int minPosition3 = ABSOLUTE_MIN;

// volatile needed for changing variables duing while loop
volatile bool homingRequested = false;
volatile byte receivedServoByte = 0xFF;
volatile byte receivedDirectionByte = 0xFF;
volatile byte receivedStepByte = 0xFF;

// this was needed to prevent race conditions
bool processingCommand = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Setup started");

  // I2C communication
  Wire.begin(0x8);
  Wire.onReceive(receiveEvent);

  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);

  pinMode(limitSwitch1Pin, INPUT_PULLUP);
  pinMode(limitSwitch2Pin, INPUT_PULLUP);
  pinMode(limitSwitch3Pin, INPUT_PULLUP);

  servo1.write(position1);
  servo2.write(position2);
  servo3.write(position3);

  Serial.println("Setup done");
}

void loop() {
  // homing
  if (homingRequested && !processingCommand) {
    homingRequested = false;
    processingCommand = true;
    homeAllServos();
    processingCommand = false;
  }

  // move servo commands
  if (receivedServoByte != 0xFF && !homingRequested && !processingCommand) {
    processingCommand = true;
    handleCommand(receivedServoByte, receivedDirectionByte, receivedStepByte);
    receivedServoByte = 0xFF;
    receivedDirectionByte = 0xFF;
    receivedStepByte = 0xFF;
    processingCommand = false;
  }
}

// interrupt to handle incoming I2C data
// check that at least 3 bytes are available
// first byte is the servo number
// second byte is the direction
// third byte is the step
// if servo number is 0, home all servos
void receiveEvent(int bytes) {
  if (bytes >= 3) {
    byte servoByte = Wire.read();
    byte directionByte = Wire.read();
    byte stepByte = Wire.read();

    if (servoByte == 0x0) {
      homingRequested = true;
    } else {
      receivedServoByte = servoByte;
      receivedDirectionByte = directionByte;
      receivedStepByte = stepByte;
    }
  } else {
    Serial.println("Received insufficient bytes for command.");
  }
}


// function to handle servo movement commands
void handleCommand(byte servoByte, byte directionByte, byte stepByte) {
  Servo* servoPtr = nullptr;
  int* positionPtr = nullptr;
  int* minPositionPtr = nullptr;

  switch (servoByte) {
    case 1:
      servoPtr = &servo1;
      positionPtr = &position1;
      minPositionPtr = &minPosition1;
      break;
    case 2:
      servoPtr = &servo2;
      positionPtr = &position2;
      minPositionPtr = &minPosition2;
      break;
    case 3:
      servoPtr = &servo3;
      positionPtr = &position3;
      minPositionPtr = &minPosition3;
      break;
    default:
      Serial.println("Invalid servo number");
      return;
  }

  if (directionByte == 0x1) {
    // Move up
    *positionPtr += stepByte;
    if (*positionPtr > MAX_POSITION) *positionPtr = MAX_POSITION;
  } else if (directionByte == 0x0) {
    // Move down
    *positionPtr -= stepByte;
    if (*positionPtr < *minPositionPtr) *positionPtr = *minPositionPtr;
  } else {
    Serial.println("Invalid direction");
    return;
  }

  servoPtr->write(*positionPtr);
  Serial.print("Servo ");
  Serial.print(servoByte);
  Serial.print(" moved by ");
  Serial.print(stepByte);
  Serial.println(" steps.");
  Serial.print("New position: ");
  Serial.println(*positionPtr);
}

// home all servos simultaneously
void homeAllServos() {
  Serial.println("Homing all servos simultaneously.");

  Servo* servos[3] = {&servo1, &servo2, &servo3};
  int* positions[3] = {&position1, &position2, &position3};
  int* minPositions[3] = {&minPosition1, &minPosition2, &minPosition3};
  int limitSwitchPins[3] = {limitSwitch1Pin, limitSwitch2Pin, limitSwitch3Pin};
  bool limitSwitchActivated[3] = {false, false, false};

  // initial positions to 135
  for (int i = 0; i < 3; i++) {
    *positions[i] = 135;
    servos[i]->write(*positions[i]);
  }

  // loop to move servos down until limit switches are hit
  while (!(limitSwitchActivated[0] && limitSwitchActivated[1] && limitSwitchActivated[2])) {
    for (int i = 0; i < 3; i++) {
      if (!limitSwitchActivated[i]) {
        if (digitalRead(limitSwitchPins[i]) == HIGH) {
          handleCommand(i + 1, 0x0, STEP);
          delay(10);
        } else {
          limitSwitchActivated[i] = true;
          *minPositions[i] = *positions[i];
          Serial.print("Servo ");
          Serial.print(i + 1);
          Serial.print(" limit switch activated at position ");
          Serial.println(*minPositions[i]);
        }
      }
    }
    delay(50);
  }

  // move servos up by 10 degrees to release limit switches
  for (int i = 0; i < 3; i++) {
    if (limitSwitchActivated[i]) {
      handleCommand(i + 1, 0x1, 0xA);
      Serial.print("Servo ");
      Serial.print(i + 1);
      Serial.print(" homed. Min position set to ");
      Serial.println(*minPositions[i]);
    }
  }

  // Move servos up a bit more
  for (int i = 0; i < 7; i++) {
    for (int j = 0; j < 3; j++) {
      handleCommand(j + 1, 0x1, 0x5);
    }
    delay(60);
  }
}
