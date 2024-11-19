#include <Wire.h>
#include <Servo.h>

// Function prototypes
void receiveEvent(int bytes);
void handleManualMove(byte servoByte, byte directionByte, byte stepByte);
void handleBulkMove(byte angle1, byte angle2, byte angle3);
void handleHoming();
void setServoPosition(byte servoNumber, int angle);
void homeAllServos();

// Constants
const int ABSOLUTE_MIN = 85;
const int MAX_POSITION = 180;

// Servo objects and arrays for easy indexing
Servo servo1;
Servo servo2;
Servo servo3;
Servo* servos[3] = {&servo1, &servo2, &servo3};

// Servo pins
const int servoPins[3] = {5, 7, 6};

// Limit switch pins
const int limitSwitchPins[3] = {2, 3, 4};

// Current positions and minimum positions
int positions[3] = {170, 170, 170};
int minPositions[3] = {ABSOLUTE_MIN, ABSOLUTE_MIN, ABSOLUTE_MIN};

// Flags and variables
volatile bool homingRequested = false;
volatile bool commandReceived = false;
volatile byte receivedCommandCode;
volatile byte receivedData[3];
bool processingCommand = false;

// Timing variables
volatile unsigned long commandReceivedTime = 0;
unsigned long servoMovedTime = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Setup started");

  // I2C communication
  Wire.begin(0x8);  // Arduino I2C address set to 0x08
  Wire.onReceive(receiveEvent);

  // Attach servos and initialize positions
  for (int i = 0; i < 3; i++) {
    servos[i]->attach(servoPins[i]);
    servos[i]->write(positions[i]);
  }

  // Set limit switch pins
  for (int i = 0; i < 3; i++) {
    pinMode(limitSwitchPins[i], INPUT_PULLUP);
  }

  Serial.println("Setup done");
}

void loop() {
  // Process received command
  if (commandReceived) {
    commandReceived = false; // Reset the flag

    // Record the time before executing the command
    unsigned long commandProcessingStartTime = micros();

    if (receivedCommandCode == 0) {
      handleHoming();
    } else if (receivedCommandCode >= 1 && receivedCommandCode <= 3) {
      handleManualMove(receivedCommandCode, receivedData[0], receivedData[1]);
    } else if (receivedCommandCode == 4) {
      handleBulkMove(receivedData[0], receivedData[1], receivedData[2]);
    } else {
      Serial.print("Unknown command code received: ");
      Serial.println(receivedCommandCode);
    }

    // The servoMovedTime is set within the servo movement functions
    // Calculate and display the time difference
    unsigned long timeToMove = servoMovedTime - commandReceivedTime;
    Serial.print("Time from I2C receive to servo move: ");
    Serial.print(timeToMove);
    Serial.println(" microseconds");
  }

  // Homing
  if (homingRequested && !processingCommand) {
    homingRequested = false;
    processingCommand = true;
    homeAllServos();
    processingCommand = false;
  }

  // No additional processing needed in loop
}

// Interrupt to handle incoming I2C data
void receiveEvent(int bytes) {
  if (bytes < 1) {
    return; // Do not use Serial.print() here
  }

  receivedCommandCode = Wire.read();
  commandReceivedTime = micros(); // Record the time when data is received

  if (receivedCommandCode == 0) {
    if (bytes >= 3) {
      Wire.read(); // Placeholder
      Wire.read(); // Placeholder
      commandReceived = true;
    }
  } else if (receivedCommandCode >= 1 && receivedCommandCode <= 3) {
    if (bytes >= 3) {
      receivedData[0] = Wire.read(); // directionByte
      receivedData[1] = Wire.read(); // stepByte
      commandReceived = true;
    }
  } else if (receivedCommandCode == 4) {
    if (bytes >= 4) {
      receivedData[0] = Wire.read(); // angle1
      receivedData[1] = Wire.read(); // angle2
      receivedData[2] = Wire.read(); // angle3
      commandReceived = true;
    }
  } else {
    // Unknown command, clear buffer
    while (Wire.available()) {
      Wire.read();
    }
  }
}

// Function to handle manual servo move commands
void handleManualMove(byte servoByte, byte directionByte, byte stepByte) {
  Serial.print("Manual Move Command received for Servo ");
  Serial.print(servoByte);
  Serial.print(" with Direction ");
  Serial.print(directionByte);
  Serial.print(" and Step ");
  Serial.println(stepByte);

  if (servoByte < 1 || servoByte > 3) {
    Serial.println("Invalid servo number in handleManualMove.");
    return;
  }

  int index = servoByte - 1;

  if (directionByte == 0x1) {
    // Move up
    positions[index] += stepByte;
    if (positions[index] > MAX_POSITION) positions[index] = MAX_POSITION;
  } else if (directionByte == 0x0) {
    // Move down
    positions[index] -= stepByte;
    if (positions[index] < minPositions[index]) positions[index] = minPositions[index];
  } else {
    Serial.println("Invalid direction in handleManualMove.");
    return;
  }

  // Record the time right before moving the servo
  servoMovedTime = micros();

  servos[index]->write(positions[index]);
  Serial.print("Servo ");
  Serial.print(servoByte);
  Serial.print(" moved to ");
  Serial.println(positions[index]);
}

// Function to handle bulk servo move commands
void handleBulkMove(byte angle1, byte angle2, byte angle3) {
  Serial.println("Bulk Move Command received.");
  Serial.print("Angle1: ");
  Serial.print(angle1);
  Serial.print(", Angle2: ");
  Serial.print(angle2);
  Serial.print(", Angle3: ");
  Serial.println(angle3);

  byte angles[3] = {angle1, angle2, angle3};
  for (int i = 0; i < 3; i++) {
    setServoPosition(i + 1, angles[i]);
  }

  Serial.println("Bulk Move Command executed.");
}

// Function to set servo position
void setServoPosition(byte servoNumber, int angle) {
  if (servoNumber < 1 || servoNumber > 3) {
    Serial.println("Invalid servo number in setServoPosition.");
    return;
  }

  int index = servoNumber - 1;

  // Ensure angle is within safe limits
  angle = constrain(angle, minPositions[index], MAX_POSITION);

  positions[index] = angle;

  // Record the time right before moving the servo
  servoMovedTime = micros();

  servos[index]->write(positions[index]);

  Serial.print("Servo ");
  Serial.print(servoNumber);
  Serial.print(" set to ");
  Serial.println(positions[index]);
}

// Function to handle homing command
void handleHoming() {
  Serial.println("Homing command received.");
  homingRequested = true;
}

// Function to home all servos
void homeAllServos() {
  Serial.println("Homing all servos simultaneously.");

  bool limitSwitchActivated[3] = {false, false, false};

  // Set initial positions to 110
  for (int i = 0; i < 3; i++) {
    positions[i] = 110;
    servos[i]->write(positions[i]);
  }

  // Move servos down until limit switches are hit
  while (!(limitSwitchActivated[0] && limitSwitchActivated[1] && limitSwitchActivated[2])) {
    for (int i = 0; i < 3; i++) {
      if (!limitSwitchActivated[i]) {
        if (digitalRead(limitSwitchPins[i]) == HIGH) {
          // Move down by 1 step
          positions[i] -= 1;
          if (positions[i] < minPositions[i]) positions[i] = minPositions[i];
          servos[i]->write(positions[i]);
          delay(10);
        } else {
          limitSwitchActivated[i] = true;
          minPositions[i] = positions[i];
          Serial.print("Servo ");
          Serial.print(i + 1);
          Serial.print(" limit switch activated at position ");
          Serial.println(minPositions[i]);
        }
      }
    }
    delay(50);
  }

  // Move servos up by 10 degrees to release limit switches
  for (int i = 0; i < 3; i++) {
    if (limitSwitchActivated[i]) {
      positions[i] += 10;
      if (positions[i] > MAX_POSITION) positions[i] = MAX_POSITION;
      servos[i]->write(positions[i]);
      Serial.print("Servo ");
      Serial.print(i + 1);
      Serial.print(" homed. Min position set to ");
      Serial.println(minPositions[i]);
    }
  }

  // Move servos up a bit more
  for (int i = 0; i < 9; i++) {
    for (int j = 0; j < 3; j++) {
      positions[j] += 5;
      if (positions[j] > MAX_POSITION) positions[j] = MAX_POSITION;
      servos[j]->write(positions[j]);
    }
    delay(60);
  }

  Serial.println("Homing process completed.");
}
