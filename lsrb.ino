// Define motor control pins
#define rightMotorF 8
#define rightMotorB 9
#define rightMotorPWM 10
#define leftMotorF 12
#define leftMotorB 13
#define leftMotorPWM 11

// Define IR sensor pins
const int numSensors = 8;
int irSensors[numSensors] = {4, 3, A0, A1, A2, A3, A4, A5};

// PID parameters
float kp = 25;
float ki = 1;
float kd = 20;
int baseSpeed = 200;

// PID variables
long integral = 0;
int previousError = 0;

// Path recording
char path[100];
int pathLength = 0;
bool exploring = true;

void setup() {
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);

  for (int i = 0; i < numSensors; i++) {
    pinMode(irSensors[i], INPUT);
  }

  Serial.begin(9600);
}

void loop() {
  int sensorStates[numSensors];
  for (int i = 0; i < numSensors; i++) {
    sensorStates[i] = digitalRead(irSensors[i]);
  }

  if (exploring) {
    if (goalReached(sensorStates)) {
      exploring = false;
      Serial.println("Goal reached. Starting path optimization...");

      for (int i = 0; i < pathLength; i++) {
        shortPath();
      }

      printPath();
      delay(2000);
      retracePath();
    } else {
      handleJunction(sensorStates);
    }
  }

  int error = calculateError(sensorStates);
  int motorSpeedDifference = calculatePID(error);
  int leftSpeed = baseSpeed + motorSpeedDifference;
  int rightSpeed = baseSpeed - motorSpeedDifference;

  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  driveMotors(leftSpeed, rightSpeed);
  delay(1);
}

void handleJunction(int sensorStates[]) {
  if (sensorStates[0] == HIGH) {
    turnLeft90();
  } else if (sensorStates[7] == HIGH) {
    turnRight90();
  } else {
    bool allLow = true;
    for (int i = 0; i < numSensors; i++) {
      if (sensorStates[i] == HIGH) {
        allLow = false;
        break;
      }
    }
    if (allLow) {
      turn180();
    }
  }
}

void turnLeft90() {
  Serial.println("Turning Left");
  driveMotors(-150, 150);
  delay(400);
  path[pathLength++] = 'L';
  if (!exploring) shortPath();
  printPath();
}

void turnRight90() {
  Serial.println("Turning Right");
  driveMotors(150, -150);
  delay(400);
  path[pathLength++] = 'R';
  if (!exploring) shortPath();
  printPath();
}

void turn180() {
  Serial.println("Turning Around");
  driveMotors(150, -150);
  delay(800);
  path[pathLength++] = 'B';
  if (!exploring) shortPath();
  printPath();
}

bool goalReached(int sensorStates[]) {
  for (int i = 0; i < numSensors; i++) {
    if (sensorStates[i] == LOW) return false;
  }
  return true;
}

void retracePath() {
  Serial.println("Retracing optimized path...");
  for (int i = 0; i < pathLength; i++) {
    char move = path[i];
    Serial.print("Step "); Serial.print(i); Serial.print(": "); Serial.println(move);
    if (move == 'L') {
      turnLeft90();
    } else if (move == 'R') {
      turnRight90();
    } else if (move == 'B') {
      turn180();
    } else {
      driveMotors(150, 150);
      delay(300);
    }
  }
  driveMotors(0, 0);
  Serial.println("Retrace complete.");
}

void shortPath() {
  if (pathLength < 3) return;
  int shortDone = 0;
  if (path[pathLength - 3] == 'L' && path[pathLength - 1] == 'R') {
    pathLength -= 3; path[pathLength++] = 'B'; shortDone = 1;
  }
  if (path[pathLength - 3] == 'L' && path[pathLength - 1] == 'S' && shortDone == 0) {
    pathLength -= 3; path[pathLength++] = 'R'; shortDone = 1;
  }
  if (path[pathLength - 3] == 'R' && path[pathLength - 1] == 'L' && shortDone == 0) {
    pathLength -= 3; path[pathLength++] = 'B'; shortDone = 1;
  }
  if (path[pathLength - 3] == 'S' && path[pathLength - 1] == 'L' && shortDone == 0) {
    pathLength -= 3; path[pathLength++] = 'R'; shortDone = 1;
  }
  if (path[pathLength - 3] == 'S' && path[pathLength - 1] == 'S' && shortDone == 0) {
    pathLength -= 3; path[pathLength++] = 'B'; shortDone = 1;
  }
  if (path[pathLength - 3] == 'L' && path[pathLength - 1] == 'L' && shortDone == 0) {
    pathLength -= 3; path[pathLength++] = 'S'; shortDone = 1;
  }
  if (path[pathLength - 3] == 'R' && path[pathLength - 1] == 'R' && shortDone == 0) {
    pathLength -= 3; path[pathLength++] = 'S'; shortDone = 1;
  }
}

void printPath() {
  Serial.println("----------------");
  for (int x = 0; x < pathLength; x++) {
    Serial.print(x); Serial.print(": "); Serial.println(path[x]);
  }
  Serial.println("----------------");
}

int calculateError(int sensorStates[]) {
  int weights[numSensors] = {-1000, -1000, -500, 0, 0, 500, 1000, 1000};
  long weightedSum = 0;
  int activeSensors = 0;
  for (int i = 0; i < numSensors; i++) {
    if (sensorStates[i] == HIGH) {
      weightedSum += weights[i];
      activeSensors++;
    }
  }
  if (activeSensors == 0) return previousError;
  return weightedSum / activeSensors;
}

int calculatePID(int error) {
  integral += error;
  integral = constrain(integral, -1000, 1000);
  int derivative = error - previousError;
  float PID = (kp * error) + (ki * integral) + (kd * derivative);
  previousError = error;
  return (int)PID;
}

void driveMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, leftSpeed);
  } else {
    digitalWrite(leftMotorF, LOW);
    digitalWrite(leftMotorB, HIGH);
    analogWrite(leftMotorPWM, -leftSpeed);
  }

  if (rightSpeed >= 0) {
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, rightSpeed);
  } else {
    digitalWrite(rightMotorF, LOW);
    digitalWrite(rightMotorB, HIGH);
    analogWrite(rightMotorPWM, -rightSpeed);
  }
}
