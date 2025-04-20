
#define rightMotorF 8
#define rightMotorB 9
#define rightMotorPWM 10
#define leftMotorF 12
#define leftMotorB 13
#define leftMotorPWM 11

const int numSensors = 8;
int irSensors[numSensors] = {4, 3, A0, A1, A2, A3, A4, A5};

float kp = 25;
float ki = 1;
float kd = 20;

int baseSpeed = 200;
long integral = 0;
int previousError = 0;

char path[100];
int pathLength = 0;
bool explorationComplete = false;
unsigned long goalDetectionStart = 0;
bool goalTimerStarted = false;

const int MAX_NODES = 50;
bool graph[MAX_NODES][MAX_NODES];
int coordinates[MAX_NODES][2];
int currentNode = 0;
int nodeCounter = 1;
int x = 0, y = 0;
int direction = 0; // 0 = UP, 1 = RIGHT, 2 = DOWN, 3 = LEFT

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
  memset(graph, 0, sizeof(graph));
  coordinates[0][0] = x;
  coordinates[0][1] = y;
}

void loop() {
  int sensorStates[numSensors];
  for (int i = 0; i < numSensors; i++) {
    sensorStates[i] = digitalRead(irSensors[i]);
  }

  if (!explorationComplete) {
    if (sensorStates[0] == HIGH) {
      turnLeft90();
      direction = (direction + 3) % 4;
      updatePosition();
    } else if (sensorStates[7] == HIGH) {
      turnRight90();
      direction = (direction + 1) % 4;
      updatePosition();
    } else if (allSensorsLow(sensorStates)) {
      turn180();
      direction = (direction + 2) % 4;
      updatePosition();
    } else {
      int error = calculateError(sensorStates);
      int motorSpeedDifference = calculatePID(error);

      int leftSpeed = baseSpeed + motorSpeedDifference;
      int rightSpeed = baseSpeed - motorSpeedDifference;

      leftSpeed = constrain(leftSpeed, 0, 255);
      rightSpeed = constrain(rightSpeed, 0, 255);

      driveMotors(leftSpeed, rightSpeed);
    }

    if (reachedGoal(sensorStates)) {
      if (!goalTimerStarted) {
        goalDetectionStart = millis();
        goalTimerStarted = true;
      } else if (millis() - goalDetectionStart >= 1000) {
        explorationComplete = true;
        bfsShortestPath();  // Run BFS for shortest path
        delay(1000);
      }
    } else {
      goalTimerStarted = false;
    }
  }
}

void updatePosition() {
  if (direction == 0) y++;
  else if (direction == 1) x++;
  else if (direction == 2) y--;
  else if (direction == 3) x--;

  int nextNode = nodeCounter++;
  coordinates[nextNode][0] = x;
  coordinates[nextNode][1] = y;
  graph[currentNode][nextNode] = true;
  currentNode = nextNode;
}

bool allSensorsLow(int sensorStates[]) {
  for (int i = 0; i < numSensors; i++) {
    if (sensorStates[i] == HIGH) return false;
  }
  return true;
}

bool reachedGoal(int sensorStates[]) {
  for (int i = 0; i < numSensors; i++) {
    if (sensorStates[i] == LOW) return false;
  }
  return true;
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
  int error = weightedSum / activeSensors;
  return error;
}

int calculatePID(int error) {
  integral += error;
  integral = constrain(integral, -1000, 1000);
  int derivative = error - previousError;
  float PID = (kp * error) + (ki * integral) + (kd * derivative);
  previousError = error;
  return (int) PID;
}

void driveMotors(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    digitalWrite(leftMotorF, HIGH);
    digitalWrite(leftMotorB, LOW);
    analogWrite(leftMotorPWM, leftSpeed);
  } else {
    digitalWrite(leftMotorF, LOW);
    digitalWrite(leftMotorB, HIGH);
    analogWrite(leftMotorPWM, -leftSpeed);
  }
  if (rightSpeed > 0) {
    digitalWrite(rightMotorF, HIGH);
    digitalWrite(rightMotorB, LOW);
    analogWrite(rightMotorPWM, rightSpeed);
  } else {
    digitalWrite(rightMotorF, LOW);
    digitalWrite(rightMotorB, HIGH);
    analogWrite(rightMotorPWM, -rightSpeed);
  }
}

void turnLeft90() {
  digitalWrite(leftMotorF, LOW);
  digitalWrite(leftMotorB, HIGH);
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  delay(300);
}

void turnRight90() {
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  digitalWrite(rightMotorF, LOW);
  digitalWrite(rightMotorB, HIGH);
  delay(300);
}

void turn180() {
  digitalWrite(leftMotorF, LOW);
  digitalWrite(leftMotorB, HIGH);
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  delay(600);
}

void moveToNode(int from, int to) {
  int dx = coordinates[to][0] - coordinates[from][0];
  int dy = coordinates[to][1] - coordinates[from][1];

  int targetDirection;
  if (dx == 1) targetDirection = 1;
  else if (dx == -1) targetDirection = 3;
  else if (dy == 1) targetDirection = 0;
  else if (dy == -1) targetDirection = 2;

  int turn = (targetDirection - direction + 4) % 4;
  if (turn == 1) turnRight90();
  else if (turn == 2) turn180();
  else if (turn == 3) turnLeft90();

  direction = targetDirection;
  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(leftMotorPWM, baseSpeed);
  analogWrite(rightMotorPWM, baseSpeed);
  delay(300);
}

void bfsShortestPath() {
  Serial.println("\n--- BFS Shortest Path from Start to Goal ---");
  int queue[MAX_NODES], front = 0, rear = 0;
  int parent[MAX_NODES];
  bool visited[MAX_NODES] = {false};
  for (int i = 0; i < MAX_NODES; i++) parent[i] = -1;

  queue[rear++] = 0;
  visited[0] = true;

  while (front < rear) {
    int current = queue[front++];
    if (current == currentNode) break;
    for (int i = 0; i < MAX_NODES; i++) {
      if (graph[current][i] && !visited[i]) {
        queue[rear++] = i;
        visited[i] = true;
        parent[i] = current;
      }
    }
  }

  int path[MAX_NODES];
  int length = 0;
  for (int node = currentNode; node != -1; node = parent[node]) {
    path[length++] = node;
  }

  Serial.print("Path: ");
  for (int i = length - 1; i >= 0; i--) {
    Serial.print(path[i]);
    if (i != 0) Serial.print(" -> ");
  }
  Serial.println();

  moveAlongPath(path, length);
}

void moveAlongPath(int path[], int length) {
  for (int i = length - 1; i > 0; i--) {
    moveToNode(path[i], path[i - 1]);
  }
}
