#include <Wire.h>
#include <Servo.h>

// Function prototypes
void receiveEvent(int bytes);
void handleCommand(byte servoByte, byte directionByte, byte stepByte);
void setServoPosition(byte servoByte, byte angleByte);
void handleBulkCommand(byte* angles);
void setBulkServoPosition(byte servoByte, byte angleByte);
void homeAllServos();

// Servo objects
Servo servo1;
Servo servo2;
Servo servo3;

// Constants
const int ABSOLUTE_MIN = 85;
const int MAX_POSITION = 180;

// Servo pins
const int servo1Pin = 5;
const int servo2Pin = 7;
const int servo3Pin = 6;

// Limit switch pins
const int limitSwitch1Pin = 2;
const int limitSwitch2Pin = 3;
const int limitSwitch3Pin = 4;

// Current positions
int position1 = 170;
int position2 = 170;
int position3 = 170;

// Minimum positions
int minPosition1 = ABSOLUTE_MIN;
int minPosition2 = ABSOLUTE_MIN;
int minPosition3 = ABSOLUTE_MIN;

// Flags and variables
volatile bool homingRequested = false;
volatile byte receivedCommandType = 0xFF;
volatile byte receivedData[6] = {0}; // Max bytes needed for bulk command
volatile int receivedBytes = 0;

// Prevent race conditions
bool processingCommand = false;

void setup() {
  Serial.begin(9600);
  Serial.println("Setup started");

  // I2C communication
  Wire.begin(0x8);
  Wire.onReceive(receiveEvent);

  // Attach servos
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);

  // Set limit switch pins
  pinMode(limitSwitch1Pin, INPUT_PULLUP);
  pinMode(limitSwitch2Pin, INPUT_PULLUP);
  pinMode(limitSwitch3Pin, INPUT_PULLUP);

  // Initialize servo positions
  servo1.write(position1);
  servo2.write(position2);
  servo3.write(position3);

  Serial.println("Setup done");
}

void loop() {
  // Homing
  if (homingRequested && !processingCommand) {
    homingRequested = false;
    processingCommand = true;
    homeAllServos();
    processingCommand = false;
  }

  // Handle received commands
  if (receivedCommandType != 0xFF && !homingRequested && !processingCommand) {
    processingCommand = true;

    if (receivedCommandType == 0x00) {
      // Manual move command: servoByte, directionByte, stepByte
      if (receivedBytes >= 3) {
        handleCommand(receivedData[0], receivedData[1], receivedData[2]);
      } else {
        Serial.println("Insufficient data for manual move command.");
      }
    }
    else if (receivedCommandType == 0x10) {
      // Bulk servo command: angle1, angle2, angle3
      if (receivedBytes >= 3) {
        handleBulkCommand(&receivedData[0]);
      } else {
        Serial.println("Insufficient data for bulk servo command.");
      }
    }
    else if (receivedCommandType == 0x01) {
      // Automated single servo command: servoByte, angleByte
      if (receivedBytes >= 2) {
        setServoPosition(receivedData[0], receivedData[1]);
      } else {
        Serial.println("Insufficient data for automated single servo command.");
      }
    }
    else {
      Serial.println("Unknown command type received.");
    }

    // Reset received command
    receivedCommandType = 0xFF;
    receivedBytes = 0;
    processingCommand = false;
  }
}

// Interrupt to handle incoming I2C data
void receiveEvent(int bytes) {
  if (bytes < 1) {
    Serial.println("Received no data.");
    return;
  }

  // Read the first byte to determine command type
  byte commandType = Wire.read();
  receivedCommandType = commandType;
  receivedBytes = 0;

  // Read the rest of the bytes based on command type
  if (commandType == 0x00) {
    // Manual move command expects 3 more bytes: servoByte, directionByte, stepByte
    if (bytes >= 4) {
      for (int i = 0; i < 3; i++) {
        if (Wire.available()) {
          receivedData[i] = Wire.read();
          receivedBytes++;
        }
      }
    } else {
      Serial.println("Insufficient bytes for manual move command.");
      receivedCommandType = 0xFF;
    }
  }
  else if (commandType == 0x10) {
    // Bulk servo command expects 3 more bytes: angle1, angle2, angle3
    if (bytes >= 4) {
      for (int i = 0; i < 3; i++) {
        if (Wire.available()) {
          receivedData[i] = Wire.read();
          receivedBytes++;
        }
      }
    } else {
      Serial.println("Insufficient bytes for bulk servo command.");
      receivedCommandType = 0xFF;
    }
  }
  else if (commandType == 0x01) {
    // Automated single servo command expects 2 more bytes: servoByte, angleByte
    if (bytes >= 3) {
      for (int i = 0; i < 2; i++) {
        if (Wire.available()) {
          receivedData[i] = Wire.read();
          receivedBytes++;
        }
      }
    } else {
      Serial.println("Insufficient bytes for automated single servo command.");
      receivedCommandType = 0xFF;
    }
  }
  else {
    Serial.println("Unknown command type received.");
    // Optionally, read remaining bytes to clear the buffer
    while (Wire.available()) {
      Wire.read();
    }
    receivedCommandType = 0xFF;
  }
}

// Function to handle manual move commands
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
      Serial.println("Invalid servo number in handleCommand.");
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
    Serial.println("Invalid direction in handleCommand.");
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

// Function to handle automated single servo command
void setServoPosition(byte servoByte, byte angleByte) {
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
      Serial.println("Invalid servo number in setServoPosition.");
      return;
  }

  int desiredPosition = angleByte;
  int minPositionWithSafety = *minPositionPtr + 35;
  desiredPosition = constrain(desiredPosition, minPositionWithSafety, MAX_POSITION);

  servoPtr->write(desiredPosition);
  *positionPtr = desiredPosition;

  Serial.print("Servo ");
  Serial.print(servoByte);
  Serial.print(" set to ");
  Serial.print(desiredPosition);
  Serial.println(" degrees.");
}

// Function to handle bulk servo commands
void handleBulkCommand(byte* angles) {
  // angles[0] -> servo1, angles[1] -> servo2, angles[2] -> servo3
  Serial.println("Bulk servo command received.");

  // Update each servo
  setBulkServoPosition(1, angles[0]);
  setBulkServoPosition(2, angles[1]);
  setBulkServoPosition(3, angles[2]);

  Serial.println("Bulk servo command executed.");
}

// Function to set bulk servo position
void setBulkServoPosition(byte servoByte, byte angleByte) {
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
      Serial.println("Invalid servo number in setBulkServoPosition.");
      return;
  }

  int desiredPosition = angleByte;
  int minPositionWithSafety = *minPositionPtr + 35;
  desiredPosition = constrain(desiredPosition, minPositionWithSafety, MAX_POSITION);

  servoPtr->write(desiredPosition);
  *positionPtr = desiredPosition;

  Serial.print("Servo ");
  Serial.print(servoByte);
  Serial.print(" set to ");
  Serial.print(desiredPosition);
  Serial.println(" degrees (Bulk).");
}

// Function to home all servos
void homeAllServos() {
  Serial.println("Homing all servos simultaneously.");

  Servo* servos[3] = {&servo1, &servo2, &servo3};
  int* positions[3] = {&position1, &position2, &position3};
  int* minPositions[3] = {&minPosition1, &minPosition2, &minPosition3};
  int limitSwitchPins[3] = {limitSwitch1Pin, limitSwitch2Pin, limitSwitch3Pin};
  bool limitSwitchActivated[3] = {false, false, false};

  // Initial positions to 110
  for (int i = 0; i < 3; i++) {
    *positions[i] = 110;
    servos[i]->write(*positions[i]);
  }

  // Move servos down until limit switches are hit
  while (!(limitSwitchActivated[0] && limitSwitchActivated[1] && limitSwitchActivated[2])) {
    for (int i = 0; i < 3; i++) {
      if (!limitSwitchActivated[i]) {
        if (digitalRead(limitSwitchPins[i]) == HIGH) {
          handleCommand(i + 1, 0x0, 1);
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

  // Move servos up by 10 degrees to release limit switches
  for (int i = 0; i < 3; i++) {
    if (limitSwitchActivated[i]) {
      handleCommand(i + 1, 0x1, 10); // 10 instead of 0xA for clarity
      Serial.print("Servo ");
      Serial.print(i + 1);
      Serial.print(" homed. Min position set to ");
      Serial.println(*minPositions[i]);
    }
  }

  // Move servos up a bit more
  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 3; j++) {
      handleCommand(j + 1, 0x1, 5);
    }
    delay(60);
  }
}
