#include <Wire.h>
#include <Servo.h>

// Function prototypes
void receiveEvent(int bytes);
void handleManualMove(byte servoByte, byte directionByte, byte stepByte);
void handleBulkMove(byte angle1, byte angle2, byte angle3);
void handleHoming();
void setServoPosition(byte servoNumber, int angle);

// Constants
const int ABSOLUTE_MIN = 80;
const int MAX_POSITION = 180;

// Servo objects and arrays for easy indexing
Servo servo1;
Servo servo2;
Servo servo3;
Servo* servos[3] = {&servo1, &servo2, &servo3};

// Servo pins
const int servoPins[3] = {5, 6, 9};

// Limit switch pins
const int limitSwitchPins[3] = {2, 3, 4};

// Current positions and minimum positions
int positions[3] = {170, 170, 170};
int minPositions[3] = {ABSOLUTE_MIN, ABSOLUTE_MIN, ABSOLUTE_MIN};
int SERVO_OFFSETS[3] = {0, -1, 5};

// Flags and variables
volatile bool commandReceived = false;
volatile byte receivedCommandCode;
volatile byte receivedData[3];

// Timing variables
volatile unsigned long commandReceivedTime = 0;
unsigned long servoMovedTime = 0;

// Homing state machine variables
enum HomingState {
  HOMING_IDLE,
  HOMING_START,
  HOMING_MOVE_DOWN,
  HOMING_RELEASE_SWITCHES,
  HOMING_MOVE_UP,
  HOMING_DONE
};

HomingState homingState = HOMING_IDLE;
unsigned long lastHomingTime = 0;
const unsigned long homingInterval = 10; // Interval in milliseconds
bool limitSwitchActivated[3] = {false, false, false};
int moveUpCount = 0; // Moved outside the switch-case to persist value

void setup() {
  // Initialize Serial communication (can be commented out in production)
  Serial.begin(9600);

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
}

void loop() {
  // Process received command
  // print current position
  Serial.print("Servo 1: ");
  Serial.print(positions[0]);
  Serial.print(" Servo 2: ");
  Serial.print(positions[1]);
  Serial.print(" Servo 3: ");
  Serial.println(positions[2]);
  
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
    }
    // No else block needed; unknown commands are ignored

    // The servoMovedTime is set within the servo movement functions
    // You can uncomment the following lines for debugging
    /*
    unsigned long timeToMove = servoMovedTime - commandReceivedTime;
    Serial.print("Time from I2C receive to servo move: ");
    Serial.print(timeToMove);
    Serial.println(" microseconds");
    */
  }

  // Homing state machine
  if (homingState != HOMING_IDLE && homingState != HOMING_DONE) {
    unsigned long currentMillis = millis();

    if (currentMillis - lastHomingTime >= homingInterval) {
      lastHomingTime = currentMillis;

      // Process homing steps
      switch (homingState) {
        case HOMING_START:
          // Set initial positions to 110
          for (int i = 0; i < 3; i++) {
            positions[i] = 110;
            minPositions[i] = ABSOLUTE_MIN;
            servos[i]->write(positions[i]);
          }
          homingState = HOMING_MOVE_DOWN;
          break;

        case HOMING_MOVE_DOWN: {
          // Move servos down until limit switches are hit
          bool allLimitSwitchesActivated = true;
          for (int i = 0; i < 3; i++) {
            // delay(100);
            if (!limitSwitchActivated[i]) {
              if (digitalRead(limitSwitchPins[i]) == HIGH) {
                // Move down by 1 step
                positions[i] -= 1;
                if (positions[i] < minPositions[i] - SERVO_OFFSETS[i]) positions[i] = minPositions[i];
                servos[i]->write(positions[i]);
                allLimitSwitchesActivated = false;
              } else {
                limitSwitchActivated[i] = true;
                minPositions[i] = positions[i] + SERVO_OFFSETS[i];
                Serial.println("Limit switch activated for servo " + String(i+1));
                Serial.println("At position " + String(positions[i]));
              }
            }
          }
          if (allLimitSwitchesActivated) {
            homingState = HOMING_RELEASE_SWITCHES;
          }
          break;
        }

        case HOMING_RELEASE_SWITCHES:
          // Move servos up by 10 degrees to release limit switches
          for (int i = 0; i < 3; i++) {
            if (limitSwitchActivated[i]) {
              positions[i] = minPositions[i] + 10;
              if (positions[i] > MAX_POSITION) positions[i] = MAX_POSITION;
              servos[i]->write(positions[i]);
            }
          }
          homingState = HOMING_MOVE_UP;
          moveUpCount = 0; // Reset moveUpCount before moving up
          break;

        case HOMING_MOVE_UP:
          // Move servos up by 5 degrees, 9 times, to move up by 45 degrees
          if (moveUpCount < 9) {
            for (int i = 0; i < 3; i++) {
              positions[i] += 5;
              if (positions[i] > MAX_POSITION) positions[i] = MAX_POSITION;
              servos[i]->write(positions[i]);
            }
            moveUpCount++;
          } else {
            homingState = HOMING_DONE;
          }
          break;

        default:
          break;
      }
    }
  }

  // Reset homing state when done
  if (homingState == HOMING_DONE) {
    homingState = HOMING_IDLE;
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
  if (servoByte < 1 || servoByte > 3) {
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
    return;
  }

  // Record the time right before moving the servo
  servoMovedTime = micros();

  servos[index]->write(positions[index]);
}

// Function to handle bulk servo move commands
void handleBulkMove(byte angle1, byte angle2, byte angle3) {
  byte angles[3] = {angle1, angle2, angle3};
  for (int i = 0; i < 3; i++) {
    setServoPosition(i + 1, angles[i]);
  }
}

// Function to set servo position
void setServoPosition(byte servoNumber, int angle) {
  if (servoNumber < 1 || servoNumber > 3) {
    return;
  }

  int index = servoNumber - 1;

  // Ensure angle is within safe limits
  angle = constrain(angle, minPositions[index] + 30, MAX_POSITION);

  positions[index] = angle;

  // apply minPosition offset from 90 to angle
  positions[index] += (minPositions[index] - 90);

  // Record the time right before moving the servo
  servoMovedTime = micros();

  servos[index]->write(positions[index]);
}

// Function to handle homing command
void handleHoming() {
  homingState = HOMING_START;
  lastHomingTime = millis();
  for (int i = 0; i < 3; i++) {
    limitSwitchActivated[i] = false;
  }
  moveUpCount = 0; // Reset moveUpCount
}