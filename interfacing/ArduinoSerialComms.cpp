// THIS FILE IS FOR MOVING THE SERVOS USING THE SERIAL MONITOR

// TYPE 'h' TO HOME ALL SERVOS SIMULTANEOUSLY
// TYPE '{servo number}{direction (+ or -)}' TO MOVE THE SERVOS
// EXAMPLE: '1+' TO MOVE SERVO 1 UP BY A STEP (10 DEGREES)

#include <Wire.h>
#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;

const int MAX_POSITION = 180;
const int STEP = 10;

const int servo1Pin = 5;
const int servo2Pin = 7;
const int servo3Pin = 6;

const int limitSwitch1Pin = 2;
const int limitSwitch2Pin = 3;
const int limitSwitch3Pin = 4;

int position1 = 135;
int position2 = 135;
int position3 = 135;

int minPosition1 = 0;
int minPosition2 = 0;
int minPosition3 = 0;

void setup() {
  // Serial communication
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
  // For serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      handleCommand(command);
    }
  }
}

// For I2C receive event
void receiveEvent(int howMany) {
  char commandBuffer[10]; // Buffer to hold incoming command
  int index = 0;

  while (Wire.available() && index < sizeof(commandBuffer) - 1) {
    commandBuffer[index++] = Wire.read();
  }
  commandBuffer[index] = '\0'; // Null-terminate the string

  // Handle the command received via I2C
  handleCommand(String(commandBuffer));
}

// Function to handle servo movement commands
void handleCommand(String command) {
  command.trim(); // Remove any whitespace

  if (command.equals("h")) {
    homeAllServos();
    return;
  }

  if (command.length() == 2) {
    char servoChar = command.charAt(0);
    char dirChar = command.charAt(1);

    Servo* servoPtr = nullptr;
    int* positionPtr = nullptr;
    int minPosition = 0;

    switch (servoChar) {
      case '1':
        servoPtr = &servo1;
        positionPtr = &position1;
        minPosition = minPosition1;
        break;
      case '2':
        servoPtr = &servo2;
        positionPtr = &position2;
        minPosition = minPosition2;
        break;
      case '3':
        servoPtr = &servo3;
        positionPtr = &position3;
        minPosition = minPosition3;
        break;
      default:
        Serial.println("Invalid servo number");
        return;
    }

    if (dirChar == '+') {
      *positionPtr += STEP;
      if (*positionPtr > MAX_POSITION) *positionPtr = MAX_POSITION;
    } else if (dirChar == '-') {
      *positionPtr -= STEP;
      if (*positionPtr < minPosition) *positionPtr = minPosition;
    } else {
      Serial.println("Invalid direction");
      return;
    }

    servoPtr->write(*positionPtr);
    Serial.print("Servo ");
    Serial.print(servoChar);
    Serial.print(" moved to position ");
    Serial.println(*positionPtr);

  } else {
    Serial.println("Invalid command format");
  }
}

// Function to home all servos simultaneously
void homeAllServos() {
  Serial.println("Homing all servos simultaneously.");
  
  position1 = 135;
  position2 = 135;
  position3 = 135;
  
  servo1.write(position1);
  servo2.write(position2);
  servo3.write(position3);

  bool limitSwitchActivated1 = false;
  bool limitSwitchActivated2 = false;
  bool limitSwitchActivated3 = false;

  // Homing loop
  while (!(limitSwitchActivated1 && limitSwitchActivated2 && limitSwitchActivated3)) {
    // For servo1
    if (!limitSwitchActivated1) {
      if (digitalRead(limitSwitch1Pin) == HIGH) {
        position1 -= 1;
        if (position1 < 0) position1 = 0;
        servo1.write(position1);
      } else {
        limitSwitchActivated1 = true;
        minPosition1 = position1;
        Serial.print("Servo 1 limit switch activated at position ");
        Serial.println(minPosition1);
      }
    }

    // For servo2
    if (!limitSwitchActivated2) {
      if (digitalRead(limitSwitch2Pin) == HIGH) {
        position2 -= 1;
        if (position2 < 0) position2 = 0;
        servo2.write(position2);
      } else {
        limitSwitchActivated2 = true;
        minPosition2 = position2;
        Serial.print("Servo 2 limit switch activated at position ");
        Serial.println(minPosition2);
      }
    }

    // For servo3
    if (!limitSwitchActivated3) {
      if (digitalRead(limitSwitch3Pin) == HIGH) {
        position3 -= 1;
        if (position3 < 0) position3 = 0;
        servo3.write(position3);
      } else {
        limitSwitchActivated3 = true;
        minPosition3 = position3;
        Serial.print("Servo 3 limit switch activated at position ");
        Serial.println(minPosition3);
      }
    }

    delay(60); // Delay to allow servos to move

    // Check if any positions reached 0 without limit switch activation
    if (position1 == 0 && !limitSwitchActivated1) {
      Serial.println("Servo 1 reached minimum position without activating limit switch.");
      limitSwitchActivated1 = true; // Stop moving this servo
    }
    if (position2 == 0 && !limitSwitchActivated2) {
      Serial.println("Servo 2 reached minimum position without activating limit switch.");
      limitSwitchActivated2 = true;
    }
    if (position3 == 0 && !limitSwitchActivated3) {
      Serial.println("Servo 3 reached minimum position without activating limit switch.");
      limitSwitchActivated3 = true;
    }
  }

  // Move servos up by 10 degrees to release limit switches
  if (limitSwitchActivated1) {
    position1 += 10;
    if (position1 > MAX_POSITION) position1 = MAX_POSITION;
    servo1.write(position1);
    delay(60);
    Serial.print("Servo 1 homed. Min position set to ");
    Serial.println(minPosition1);
  }

  if (limitSwitchActivated2) {
    position2 += 10;
    if (position2 > MAX_POSITION) position2 = MAX_POSITION;
    servo2.write(position2);
    delay(60);
    Serial.print("Servo 2 homed. Min position set to ");
    Serial.println(minPosition2);
  }

  if (limitSwitchActivated3) {
    position3 += 10;
    if (position3 > MAX_POSITION) position3 = MAX_POSITION;
    servo3.write(position3);
    delay(60);
    Serial.print("Servo 3 homed. Min position set to ");
    Serial.println(minPosition3);
  }

  // Move servos up a bit more
  for (int i = 0; i < 7; i++) {
    position1 += 5;
    position2 += 5;
    position3 += 5;
    servo1.write(position1);
    servo2.write(position2);
    servo3.write(position3);
    delay(60);
  }
}
