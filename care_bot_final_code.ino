#include <Wire.h> // I2C 통신을 위한 필수 라이브러리
#include <Adafruit_PWMServoDriver.h> // PCA9685 라이브러리

// PCA9685 객체 생성 (기본 I2C 주소 0x40 사용)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- 서보 관련 설정 ---
int servoChannels[8] = {0, 2, 4, 6, 8, 10, 12, 14};

// 서보의 현재 각도를 저장하는 배열 (사용자 정의 초기값)
int servoAngles[8] = {70, 95, 90, 65, 80, 90, 40, 50};
// idx: 0(오른쪽팔), 1(오른쪽어깨), 2(오른쪽팔꿈치)
// idx: 3(왼쪽팔), 4(왼쪽어깨), 5(왼쪽팔꿈치), 6(허리R), 7(허리L)

#define SERVOMIN 150 
#define SERVOMAX 600 
#define SERVO_FREQ 50 

// --- 스텝모터 관련 설정 ---
#define DIR1 2
#define STEP1 3
#define DIR2 4
#define STEP2 5

void setup() {
  Serial1.begin(9600);
  Serial.begin(9600);
  Serial.println("PCA9685 서보 제어기 (안전 제한 적용됨)");

  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ); 

  for (int i = 0; i < 8; i++) {
    int pulse = map(servoAngles[i], 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(servoChannels[i], 0, pulse);
    delay(10);
  }

  pinMode(DIR1, OUTPUT);
  pinMode(STEP1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(STEP2, OUTPUT);
}

void loop() {
  static String cmd = "";
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      cmd.trim();
      Serial.println(cmd);

      // --- 서보 명령  ---
      //오른팔
      if (cmd == "RD") moveServo(0, -10);
      else if (cmd == "RU") moveServo(0, +5);
      //오른어깨
      else if (cmd == "RSU") moveServo(1, +5);
      else if (cmd == "RSD") moveServo(1, -5);
      //왼팔
      else if (cmd == "LU") moveServo(3, -5);
      else if (cmd == "LD") moveServo(3, +5); // 3번 서보 (왼쪽 어깨)
      //왼쪽 어깨
      else if (cmd == "LSU") moveServo(4, -5);
      else if (cmd == "LSD") moveServo(4, +5);
      //팔꿈치
      else if (cmd == "AC") moveDualServoOpposite(2, 5, +5);
      else if (cmd == "AE") moveDualServoOpposite(2, 5, -5);
      else if (cmd == "PU") {
        moveServo(0, +5);
        delay(10);
        moveServo(3, -5);
        delay(10);
        moveDualServoOpposite(2, 5, +5);
      }
      else if (cmd == "PD") {
        moveServo(0, -5);
        delay(10);
        moveServo(3, +5);
        delay(10);
        moveDualServoOpposite(2, 5, -5);
      }
      else if (cmd == "B1") moveDualServoOpposite(6, 7, -5);
      else if (cmd == "F1") moveDualServoOpposite(6, 7, +5);
      // 세밀 조정을 위한 소문자 명령 (추가 제안)
      else if (cmd == "b1") moveDualServoOpposite(6, 7, -1);
      else if (cmd == "f1") moveDualServoOpposite(6, 7, +1);

      // --- 스텝모터 명령 ---
      else if (cmd == "F") moveSteppers(HIGH, LOW, 100);
      else if (cmd == "B") moveSteppers(LOW, HIGH, 100);
      else if (cmd == "L") moveSteppers(LOW, LOW, 100);
      else if (cmd == "R") moveSteppers(HIGH, HIGH, 100);


      cmd = ""; // 명령 처리 후 초기화
    } else {
      cmd += c;
    }
  }
}

// --- 스텝모터 제어 함수 ---
void moveSteppers(bool dir1, bool dir2, int steps) {
  digitalWrite(DIR1, dir1);
  digitalWrite(DIR2, dir2);
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP1, HIGH);
    digitalWrite(STEP2, HIGH);
    delayMicroseconds(500);
    digitalWrite(STEP1, LOW);
    digitalWrite(STEP2, LOW);
    delayMicroseconds(500);
  }
}

// --- 서보 제어 함수  ---
void moveServo(int idx, int delta) {
  if (idx < 0 || idx >= 8) return;

  // 1. 목표 각도를 우선 계산
  int newAngle = servoAngles[idx] + delta;

  // 2. 각 서보의 최소/최대 범위를 설정
  int minAngle = 20; // 모든 서보의 기본 최소 각도
  int maxAngle = 160; // 모든 서보의 기본 최대 각도

  if (idx == 4) { // 3번 서보 (왼쪽 어깨)인 경우
    maxAngle = 85; // 최대 각도를 85로 제한
  }
  
  if (idx == 1) { // 1번 서보 (오른쪽 어깨)인 경우
    // '떨어지지 않게' = - 방향(RSD)
    // 따라서 minAngle을 85로 제한합니다.
    minAngle = 85;
  }
  // 4. 계산된 각도를 min/max 범위 안으로 제한(constrain)
  servoAngles[idx] = constrain(newAngle, minAngle, maxAngle);

  // 5. 각도를 펄스(틱) 값으로 변환
  int pulse = map(servoAngles[idx], 0, 180, SERVOMIN, SERVOMAX);

  // 6. PCA9685에 명령 전송
  int channel = servoChannels[idx]; 
  pwm.setPWM(channel, 0, pulse);

  // 7. 시리얼 로그
  Serial.print("Servo ");
  Serial.print(idx);
  Serial.print(" (Channel "); 
  Serial.print(channel);      
  Serial.print(") -> Angle: ");
  Serial.println(servoAngles[idx]);
}

void moveDualServoOpposite(int idx1, int idx2, int delta) {
  moveServo(idx1, delta);
  moveServo(idx2, -delta);
}

void moveDualServo(int idx1, int idx2, int delta) {
  moveServo(idx1, delta);
  moveServo(idx2, delta);
}
