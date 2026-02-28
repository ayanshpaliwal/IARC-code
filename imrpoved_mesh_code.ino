#include <BluetoothSerial.h>
#include <EEPROM.h>
#include <Wire.h>
#include <MPU6050_light.h>

BluetoothSerial SerialBT;
MPU6050 mpu(Wire);

#define EEPROM_SIZE 512
#define PATH_START_ADDR 0
#define PATH_LENGTH_ADDR 500

const int LF = 13, LB = 12, RF = 14, RB = 27;

#define ENCODER_LEFT 19
#define ENCODER_RIGHT 18

volatile long leftCount = 0;
volatile long rightCount = 0;

static volatile uint32_t lastLeftUs = 0;
static volatile uint32_t lastRightUs = 0;
const uint32_t ENCODER_DEBOUNCE_US = 10;

const int ENCODER_TARGET = 60;

const int L4 = 4, L3 = 15, L2 = 34, L1 = 35,
          R1 = 32, R2 = 33, R3 = 25, R4 = 26;

const int START_BTN = 5;
const int RUN_LED = 2;

bool ACTIVE_LOW = false;

int baseSpeed = 250;
int maxSpeed = 255;
int replayBaseSpeed = 250;
float Kp = -400.0; //increase IDK

int SPEED_BOOST = 50;
unsigned long straightStartTime = 0;
unsigned long STRAIGHT_BOOST_TIME = 2000;
bool speedBoostActive = false;

const unsigned long HOLD_MS = 25;
const unsigned long STRAIGHT_TO = 10;
const unsigned long RECENTER_MS = 100;
const unsigned long RECENTER_MS_REPLAY = 100;
const unsigned long GAP_MS = 20;

const int END_WHITE_MIN = 7;
int TURN_SPEED = 200;

#define GYRO_TOLERANCE 1
#define GYRO_TIMEOUT 2000
float targetYaw = 0;
float startYaw = 0;

char path[256];
int pathLength = 0;
char replayPath[256];
int replayLength = 0;
int replayIndex = 0;

bool isReplayMode = false;
bool isRightHandMode = false; 
bool savedThisHalt = false;  

void IRAM_ATTR leftEncoderISR() {
  uint32_t now = micros();
  if (now - lastLeftUs >= ENCODER_DEBOUNCE_US) {
    leftCount++;
    lastLeftUs = now;
  }
}
void IRAM_ATTR rightEncoderISR() {
  uint32_t now = micros();
  if (now - lastRightUs >= ENCODER_DEBOUNCE_US) {
    rightCount++;
    lastRightUs = now;
  }
}

inline void motorsAllLow() {
  pinMode(LF, OUTPUT);
  digitalWrite(LF, LOW);
  pinMode(LB, OUTPUT);
  digitalWrite(LB, LOW);
  pinMode(RF, OUTPUT);
  digitalWrite(RF, LOW);
  pinMode(RB, OUTPUT);
  digitalWrite(RB, LOW);
}
inline int clamp255(int v) {
  return v < 0 ? 0 : (v > 255 ? 255 : v);
}
inline void drive(int L, int R) {
  L = clamp255(L);
  R = clamp255(R);
  analogWrite(LF, L);
  analogWrite(LB, 0);
  analogWrite(RF, R);
  analogWrite(RB, 0);
}
inline void forwardCruise() {
  int speed = isReplayMode ? replayBaseSpeed : baseSpeed;
  if (speedBoostActive) speed += SPEED_BOOST;
  drive(speed, speed);
}
inline void spinLeft(int pwm) {
  pwm = clamp255(pwm);
  analogWrite(LF, 0);
  analogWrite(LB, pwm);
  analogWrite(RF, pwm);
  analogWrite(RB, 0);
}
inline void spinRight(int pwm) {
  pwm = clamp255(pwm);
  analogWrite(LF, pwm);
  analogWrite(LB, 0);
  analogWrite(RF, 0);
  analogWrite(RB, pwm);
}
inline int onBlack(int pin) {
  int v = digitalRead(pin);
  return ACTIVE_LOW ? !v : v;
}

inline void readEncoders(long &l, long &r) {
  noInterrupts();
  l = leftCount;
  r = rightCount;
  interrupts();
}
inline void resetEncoders() {
  noInterrupts();
  leftCount = 0;
  rightCount = 0;
  lastLeftUs = micros();
  lastRightUs = micros();
  interrupts();
}

int bL4, bL3, bL2, bL1, bR1, bR2, bR3, bR4;
void readSensors() {
  bL4 = onBlack(L4);
  bL3 = onBlack(L3);
  bL2 = onBlack(L2);
  bL1 = onBlack(L1);
  bR1 = onBlack(R1);
  bR2 = onBlack(R2);
  bR3 = onBlack(R3);
  bR4 = onBlack(R4);
}
float getYaw() {
  mpu.update();
  return -mpu.getAngleZ();
}
float shortestAngleDiff(float target, float current) {
  float diff = target - current;
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;
  return diff;
}
bool gyroReachedTarget() {
  float currentYaw = getYaw();
  float error = shortestAngleDiff(targetYaw, currentYaw);
  return (abs(error) <= GYRO_TOLERANCE);
}

void printSnap(const char *tag) {
  char buf[128];
  snprintf(buf, sizeof(buf),
           "%s L4:%d L3:%d L2:%d L1:%d | R1:%d R2:%d R3:%d R4:%d",
           tag, bL4, bL3, bL2, bL1, bR1, bR2, bR3, bR4);
  Serial.println(buf);
  SerialBT.println(buf);
}

inline bool allBlack() {
  return (bL4 && bL3 && bL2 && bL1 && bR1 && bR2 && bR3 && bR4);
}
inline bool allWhite() {
  return (!bL4 && !bL3 && !bL2 && !bL1 && !bR1 && !bR2 && !bR3 && !bR4);
}
inline bool leftSideWhiteAny() {
  return (!bL3 && !bL4);
}
inline bool rightSideWhiteAny() {
  return (!bR3 && !bR4);
}
inline bool leftSideBlack() {
  return (bL3 || bL4);
}
inline bool rightSideBlack() {
  return (bR3 || bR4);
}
inline bool centerAnyWhite() {
  return (!bL2 || !bL1 || !bR1 || !bR2);
}
inline bool leftAvail() {
  return leftSideWhiteAny();
}
inline bool rightAvail() {
  return rightSideWhiteAny();
}
inline bool straightAvail() {
  return centerAnyWhite();
}

bool isEndZoneConfirmed(unsigned samples = 3) {
  int whiteCount = (!bL4) + (!bL3) + (!bL2) + (!bL1) + (!bR1) + (!bR2) + (!bR3) + (!bR4);
  Serial.println(whiteCount);
  if (whiteCount >= 7) {
    return true;
  }
  return false;
}

void followInner4_P() {
  int wL2 = !bL2, wL1 = !bL1, wR1 = !bR1, wR2 = !bR2;
  if (!wL2 && !wL1 && !wR1 && !wR2) {
    drive(100, 100);
    return;
  }
  int pos = (wL2 ? -3 : 0) + (wL1 ? -1 : 0) + (wR1 ? +1 : 0) + (wR2 ? +3 : 0);
  int corr = (int)(Kp * (float)pos / 10.0);
  int curBase = isReplayMode ? replayBaseSpeed : baseSpeed;
  int curMax = isReplayMode ? replayBaseSpeed : maxSpeed;

  if (speedBoostActive) {
    curBase += SPEED_BOOST;
    curMax += SPEED_BOOST;
  }

  int L = clamp255(curBase - corr), R = clamp255(curBase + corr);
  if (L > curMax) L = curMax;
  if (R > curMax) R = curMax;
  drive(L, R);
}

int lastTrig = 0, stableTrig = 0;
unsigned long seenAt = 0;
int sidesTriggerNow() {
  if (allBlack()) return 2;
  bool L = leftAvail();
  bool R = rightAvail();
  if ((L || R) && centerAnyWhite()) return 1;
  return 0;
}
bool junctionTriggered() {
  int now = sidesTriggerNow();
  if (now == 0) {
    stableTrig = 0;
    lastTrig = 0;
    return false;
  }
  if (now != lastTrig) {
    lastTrig = now;
    seenAt = millis();
    return false;
  }
  if (stableTrig != now && millis() - seenAt >= HOLD_MS) stableTrig = now;
  return (stableTrig != 0);
}
enum { IDLE = -1,
       FOLLOW = 0,
       NUDGE = 1,
       TURN_L = 2,
       TURN_R = 3,
       UTURN = 4,
       GO_STRAIGHT = 6,
       RECENTER = 7,
       HALT = 9 };
int state = IDLE;
unsigned long stateStart = 0;

bool sawLeft = false, sawRight = false;
bool hasStraight = false;
unsigned long lastDecisionMs = 0;

bool sampleCenterPresence(unsigned samples = 5, unsigned perDelayMs = 3) {
  unsigned yes = 0;
  for (unsigned i = 0; i < samples; i++) {
    readSensors();
    if (straightAvail()) yes++;
    delay(perDelayMs);
  }
  return (yes * 2 >= samples);
}

void optimizePathLeft();
void optimizePathRight();

void addMove(char move) {
  path[pathLength++] = move;
  if (pathLength >= 3 && path[pathLength - 2] == 'B') {
    if (isRightHandMode) {
      optimizePathRight();
    } else {
      optimizePathLeft();
    }
  }
}

void optimizePathLeft() {
  if (pathLength < 3) return;
  char a = path[pathLength - 3];
  char b = path[pathLength - 2];
  char c = path[pathLength - 1];
  if (b != 'B') return;
  char replacement = ' ';
  if (a == 'L' && c == 'R') replacement = 'B';
  else if (a == 'L' && c == 'S') replacement = 'R';
  else if (a == 'R' && c == 'L') replacement = 'B';
  else if (a == 'S' && c == 'L') replacement = 'R';
  else if (a == 'S' && c == 'S') replacement = 'B';
  else if (a == 'L' && c == 'L') replacement = 'S';
  else if (a == 'R' && c == 'R') replacement = 'S';

  if (replacement != ' ') {
    path[pathLength - 3] = replacement;
    pathLength -= 2;
  }
}
void optimizePathRight() {
  if (pathLength < 3) return;
  char a = path[pathLength - 3];
  char b = path[pathLength - 2];
  char c = path[pathLength - 1];

  if (b != 'B') return;

  char replacement = ' ';
  if (a == 'L' && c == 'R') replacement = 'B';
  else if (a == 'L' && c == 'S') replacement = 'R';
  else if (a == 'R' && c == 'L') replacement = 'B';
  else if (a == 'S' && c == 'L') replacement = 'R';
  else if (a == 'S' && c == 'S') replacement = 'B';
  else if (a == 'L' && c == 'L') replacement = 'S';
  else if (a == 'R' && c == 'R') replacement = 'S';
  else if (a == 'S' && c == 'R') replacement = 'L';
  else if (a == 'R' && c == 'S') replacement = 'L';

  if (replacement != ' ') {
    path[pathLength - 3] = replacement;
    pathLength -= 2;
  }
}

void savePathToEEPROM() {
  EEPROM.write(PATH_LENGTH_ADDR, pathLength);
  for (int i = 0; i < pathLength; i++) EEPROM.write(PATH_START_ADDR + i, path[i]);
  EEPROM.commit();
  Serial.println("Path saved to EEPROM!");
  SerialBT.println("Path saved to EEPROM!");
}

void loadPathFromEEPROM() {
  replayLength = EEPROM.read(PATH_LENGTH_ADDR);
  if (replayLength > 256 || replayLength == 255) replayLength = 0;
  for (int i = 0; i < replayLength; i++) replayPath[i] = EEPROM.read(PATH_START_ADDR + i);
}

void waitForStartPress() {
  while (digitalRead(START_BTN) == HIGH) { delay(5); }

  unsigned long pressStart = millis();
  while (digitalRead(START_BTN) == LOW) { delay(10); }
  unsigned long pressDuration = millis() - pressStart;

  if (pressDuration >= 7000) {
    isReplayMode = false;
    isRightHandMode = false;
    digitalWrite(RUN_LED, HIGH);
    delay(1000); 
    digitalWrite(RUN_LED, LOW);
    pathLength = 0;
  } 
  else if (pressDuration >= 3000) {
    isReplayMode = false;
    isRightHandMode = true;
    for (int i = 0; i < 2; i++) {
      digitalWrite(RUN_LED, HIGH);
      delay(300);
      digitalWrite(RUN_LED, LOW);
      delay(300);
    }
    pathLength = 0;
  } 
  else {
    isReplayMode = true;
    isRightHandMode = false;
    for (int i = 0; i < 5; i++) {
      digitalWrite(RUN_LED, HIGH);
      delay(100);
      digitalWrite(RUN_LED, LOW);
      delay(100);
    }
    loadPathFromEEPROM();
  }
}

void waitForGyroCalibration() {
  digitalWrite(RUN_LED, HIGH);
  mpu.calcOffsets();
  digitalWrite(RUN_LED, LOW);
}

void enter(int s) {
  state = s;
  stateStart = millis();

  if (s == NUDGE) resetEncoders();
  if (s == HALT) savedThisHalt = false;

  if (s == FOLLOW || s == GO_STRAIGHT) {
    if (straightStartTime == 0) straightStartTime = millis();
  } else {
    straightStartTime = 0;
    speedBoostActive = false;
  }

  if (s == TURN_L) {
    startYaw = getYaw();
    targetYaw = startYaw - 68.5; //decrease if it over turns
  } else if (s == TURN_R) {
    startYaw = getYaw();
    targetYaw = startYaw + 68.5; //decrease if it over turns
  } else if (s == UTURN) {
    startYaw = getYaw();
    targetYaw = startYaw + 203; //increase if it over turns i think
  }
}

void setup() {
  pinMode(23, OUTPUT);
  digitalWrite(23, HIGH);

  motorsAllLow();
  delay(30);
  Serial.begin(115200);
  SerialBT.begin("Iphone SE");
  EEPROM.begin(EEPROM_SIZE);

  pinMode(ENCODER_LEFT, INPUT);
  pinMode(ENCODER_RIGHT, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT), rightEncoderISR, RISING);

  Wire.begin(21, 22, 400000);
  byte status = mpu.begin();
  if (status != 0) {
    while (1) delay(100);
  }
  waitForGyroCalibration();

  pinMode(L4, INPUT);
  pinMode(L3, INPUT);
  pinMode(L2, INPUT);
  pinMode(L1, INPUT);
  pinMode(R1, INPUT);
  pinMode(R2, INPUT);
  pinMode(R3, INPUT);
  pinMode(R4, INPUT);

  pinMode(START_BTN, INPUT_PULLUP);
  pinMode(RUN_LED, OUTPUT);
  digitalWrite(RUN_LED, LOW);

  waitForStartPress();
  enter(FOLLOW);
}

void loop() {
  if (state == HALT) {
    motorsAllLow();
    digitalWrite(RUN_LED, HIGH);

    if (!isReplayMode && !savedThisHalt) {
      savePathToEEPROM();
      savedThisHalt = true;
    }

    if (digitalRead(START_BTN) == LOW) {
      digitalWrite(RUN_LED, LOW);
      waitForStartPress(); 
      enter(FOLLOW);
    }
    delay(50);
    return;
  }

  readSensors();

  if (state == FOLLOW) {
    if (straightStartTime > 0 && !speedBoostActive && (millis() - straightStartTime >= STRAIGHT_BOOST_TIME)) {
      speedBoostActive = true;
    }

    if (isReplayMode && replayIndex >= replayLength) {
      int whiteCount = (!bL4) + (!bL3) + (!bL2) + (!bL1) + (!bR1) + (!bR2) + (!bR3) + (!bR4);
      if (whiteCount >= END_WHITE_MIN || allWhite()) {
        enter(HALT);
        return;
      }
    }

    if (junctionTriggered()) {
      if (millis() - lastDecisionMs < GAP_MS) {
        followInner4_P();
        return;
      }

      sawLeft = leftAvail();
      sawRight = rightAvail();
      lastDecisionMs = millis();
      enter(NUDGE);
      return;
    }

    followInner4_P();
    return;
  }

  if (state == NUDGE) {
    forwardCruise();

    static unsigned long lastDbg = 0;
    if (millis() - lastDbg > 100) {
      long lc, rc;
      readEncoders(lc, rc);
      Serial.printf("NUDGE enc L=%ld R=%ld\n", lc, rc);
      lastDbg = millis();
    }

    long lc, rc;
    readEncoders(lc, rc);

    if (lc >= ENCODER_TARGET) {
      hasStraight = sampleCenterPresence(5, 3);
      readSensors();
      if (isEndZoneConfirmed()) {
        Serial.println("END ZONE DETECTED after sampling!");
        SerialBT.println("END ZONE DETECTED after sampling!");
        printSnap("ENDZONE");
        motorsAllLow();
        delay(100);
        enter(HALT);
        return;
      }

      if (isReplayMode) {
        if (replayIndex < replayLength) {
          char nextMove = replayPath[replayIndex++];
          if (nextMove == 'L') enter(TURN_L);
          else if (nextMove == 'R') enter(TURN_R);
          else if (nextMove == 'S') enter(GO_STRAIGHT);
          else if (nextMove == 'B') enter(UTURN);
          else enter(UTURN);
        } else {
          enter(HALT);
        }
      } else {
        if (isRightHandMode) {
          if (sawRight) {
            addMove('R');
            enter(TURN_R);
          } else if (hasStraight) {
            addMove('S');
            enter(GO_STRAIGHT);
          } else if (sawLeft) {
            addMove('L');
            enter(TURN_L);
          } else {
            addMove('B');
            enter(UTURN);
          }
        } else {
          if (sawLeft && hasStraight) {
            addMove('L');
            enter(TURN_L);
          } else if (!sawLeft && sawRight && hasStraight) {
            addMove('S');
            enter(GO_STRAIGHT);
          } else if (sawLeft && !hasStraight) {
            addMove('L');
            enter(TURN_L);
          } else if (sawRight && !hasStraight) {
            addMove('R');
            enter(TURN_R);
          } else if (sawLeft && sawRight && !hasStraight) {
            addMove('L');
            enter(TURN_L);
          } else if (!sawLeft && !sawRight && hasStraight) {
            addMove('S');
            enter(GO_STRAIGHT);
          } else {
            addMove('B');
            enter(UTURN);
          }
        }
      }
    }

    if (millis() - stateStart > 1200) {
      if (!isReplayMode) addMove('B');
      enter(UTURN);
    }
    return;
  }

  if (state == GO_STRAIGHT) {
    forwardCruise();
    bool sidesBlackOK = leftSideBlack() && rightSideBlack();
    if ((centerAnyWhite() && sidesBlackOK) || (millis() - stateStart >= STRAIGHT_TO)) enter(RECENTER);
    return;
  }

  if (state == TURN_L) {
    spinLeft(TURN_SPEED);
    if (gyroReachedTarget() || millis() - stateStart >= GYRO_TIMEOUT) enter(RECENTER);
    return;
  }

  if (state == TURN_R) {
    spinRight(TURN_SPEED);
    if (gyroReachedTarget() || millis() - stateStart >= GYRO_TIMEOUT) enter(RECENTER);
    return;
  }

  if (state == UTURN) {
    spinLeft(TURN_SPEED);
    if (gyroReachedTarget() || millis() - stateStart >= (unsigned long)(GYRO_TIMEOUT * 1.5)) enter(RECENTER);
    return;
  }

  if (state == RECENTER) {
    followInner4_P();
    unsigned long recenterTime = isReplayMode ? RECENTER_MS_REPLAY : RECENTER_MS;
    if (millis() - stateStart >= recenterTime) enter(FOLLOW);
    return;
  }
}
