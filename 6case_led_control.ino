/*
 * 프로젝트명: 지능형 화재 대피 유도 시스템 (FR-Table 기반)
 * MCU: Arduino UNO
 * 설명: 4개 구역의 화재 감지 센서와 블루투스 신호를 바탕으로
 * 요구사항(FR-05, 06, 07)에 정의된 고유 로직에 따라
 * 5개 구역(A,B,C,D,CS)의 2색 LED를 제어합니다.
 *
 * --- LED 작동 방식 ---
 * - 공통 애노드 (Common Anode)
 * - LED 공통 핀: 아두이노 5V
 * - LOW 신호 = 켜짐 (ON)
 * - HIGH 신호 = 꺼짐 (OFF)
 *
 * --- 핵심 로직 (FR-05, 06, 07) ---
 * 1. 평시 (0곳): ALL OFF
 * 2. 1~3곳 화재:
 * - A, C, CS = GREEN (항상 안전)
 * - B = RED (if 107 or 109) / else GREEN
 * - D = RED (if 102 or 104) / else GREEN
 * 3. 4곳 화재: ALL RED
 */

// 1. 라이브러리 포함
#include <SoftwareSerial.h>

// 2. 핀 번호 정의 (핀맵)
// ==========================================================
// HC-06 블루투스 핀
const int BT_RX_PIN = 11; // 아두이노 RX <- HC-06 TX
const int BT_TX_PIN = 12; // 아두이노 TX -> HC-06 RX (★★★ 전압분배 저항 필수 ★★★)

// 불꽃 감지 센서 핀 (입력)
const int SENSOR_102 = 2; // 1구역
const int SENSOR_109 = 3; // 2구역
const int SENSOR_104 = 5; // 3구역
const int SENSOR_107 = 6; // 4구역

// LED 루트 A (출력)
const int LED_A_RED = A0;
const int LED_A_GREEN = A1;

// LED 루트 B (출력)
const int LED_B_RED = A2;
const int LED_B_GREEN = A3;

// LED 루트 C (출력)
const int LED_C_RED = A4;
const int LED_C_GREEN = A5;

// LED 루트 D (출력)
const int LED_D_RED = 7;
const int LED_D_GREEN = 8;

// LED 센터스팟 (CS) (출력)
const int LED_CS_RED = 9;
const int LED_CS_GREEN = 10;


// 3. 전역 변수 및 객체 선언
// ==========================================================
// 블루투스 시리얼 객체 선언
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN);

// 센서 감지 상태 정의 (Active LOW)
const int FIRE_DETECTED = LOW;
const int NO_FIRE = HIGH;

// LED 제어 로직 정의 (공통 애노드)
const int LED_ON = LOW;
const int LED_OFF = HIGH;

// LED 상태 상수 (가독성)
const int STATE_OFF = 0;
const int STATE_RED = 1;
const int STATE_GREEN = 2;

// LED 핀 배열 (setLED 함수에서 사용)
// 순서: 0=A, 1=B, 2=C, 3=D, 4=CS
const int ledRedPins[5] = {LED_A_RED, LED_B_RED, LED_C_RED, LED_D_RED, LED_CS_RED};
const int ledGreenPins[5] = {LED_A_GREEN, LED_B_GREEN, LED_C_GREEN, LED_D_GREEN, LED_CS_GREEN};

// 화재 상태 저장 변수
// 순서: 0=102호, 1=109호, 2=104호, 3=107호
bool btFireState[4] = {false, false, false, false}; // 블루투스 가상 화재 상태
bool lastCombinedFireState[4] = {false, false, false, false}; // 깜빡임 방지용


// 4. setup() 함수 (초기 설정)
// ==========================================================
void setup() {
  // 시리얼 모니터 시작 (디버깅용)
  Serial.begin(9600);
  Serial.println("Fire Escape System Booting...");

  // 블루투스 시리얼 시작
  btSerial.begin(9600);

  // 핀 모드 설정: 불꽃 감지 센서 4개 (입력)
  pinMode(SENSOR_102, INPUT);
  pinMode(SENSOR_109, INPUT);
  pinMode(SENSOR_104, INPUT);
  pinMode(SENSOR_107, INPUT);

  // 핀 모드 설정: 2색 LED 핀 10개 (출력)
  for (int i = 0; i < 5; i++) {
    pinMode(ledRedPins[i], OUTPUT);
    pinMode(ledGreenPins[i], OUTPUT);
  }

  // 시스템 초기 상태(평시)로 설정 (모든 LED 끄기)
  resetToSafeMode();

  Serial.println("System Ready.");
}

// 5. loop() 함수 (메인 루프)
// ==========================================================
void loop() {
  // 1. 블루투스 입력 확인
  checkBluetooth();

  // 2. 물리적 화재 센서 값 읽기
  bool physicalFireState[4];
  checkFireSensors(physicalFireState);

  // 3. 최종 화재 상태 결정 (물리 센서 OR 블루투스 신호)
  bool combinedFireState[4];
  bool stateChanged = false; // 상태 변경 감지 플래그

  for (int i = 0; i < 4; i++) {
    combinedFireState[i] = physicalFireState[i] || btFireState[i];

    // 이전 상태와 비교하여 변경점이 있는지 확인 (깜빡임 방지)
    if (combinedFireState[i] != lastCombinedFireState[i]) {
      stateChanged = true;
    }
  }

  // 4. 시스템 상태가 변경되었을 때만 LED 업데이트
  if (stateChanged) {
    updateSystemOutputs(combinedFireState); // LED 업데이트 함수 호출

    // 5. 현재 상태를 '이전 상태'로 저장
    for (int i = 0; i < 4; i++) {
      lastCombinedFireState[i] = combinedFireState[i];
    }
  }

  // 시스템 안정화를 위한 짧은 딜레이
  delay(100);
}

// 6. 사용자 정의 함수 (기능별 분리)
// ==========================================================

/**
 * @brief 4개의 물리적 화재 센서 상태를 읽어 배열에 저장합니다.
 * @param pSensors (출력) 센서 상태를 저장할 bool 배열 (크기 4)
 */
void checkFireSensors(bool pSensors[]) {
  // Active LOW 방식: 감지되면 LOW(0), 평시는 HIGH(1)
  // 순서: 0=102호, 1=109호, 2=104호, 3=107호
  pSensors[0] = (digitalRead(SENSOR_102) == FIRE_DETECTED);
  pSensors[1] = (digitalRead(SENSOR_109) == FIRE_DETECTED);
  pSensors[2] = (digitalRead(SENSOR_104) == FIRE_DETECTED);
  pSensors[3] = (digitalRead(SENSOR_107) == FIRE_DETECTED);
}

/**
 * @brief 블루투스 시리얼 입력을 확인하고 btFireState를 업데이트합니다.
 */
void checkBluetooth() {
  if (btSerial.available()) {
    char data = btSerial.read();
    Serial.print("Bluetooth data received: ");
    Serial.println(data);

    switch (data) {
      case '1': // 102호(1구역) 가상 화재
        btFireState[0] = true;
        break;
      case '2': // 109호(2구역) 가상 화재
        btFireState[1] = true;
        break;
      case '3': // 104호(3구역) 가상 화재
        btFireState[2] = true;
        break;
      case '4': // 107호(4구역) 가상 화재
        btFireState[3] = true;
        break;
      case '0': // 시스템 리셋 (블루투스 신호만 리셋)
        for (int i = 0; i < 4; i++) {
          btFireState[i] = false;
        }
        Serial.println("Bluetooth triggers reset.");
        break;
    }
  }
}

/**
 * @brief ★★★ [핵심 로직] ★★★
 * 최종 화재 상태를 바탕으로 5개 LED 구역을 FR-05,06,07 로직으로 업데이트합니다.
 * @param fireState 최종 화재 상태 bool 배열 (크기 4)
 * [0]=102호, [1]=109호, [2]=104호, [3]=107호
 */
void updateSystemOutputs(bool fireState[]) {
  int fireCount = 0;
  for (int i = 0; i < 4; i++) {
    if (fireState[i]) {
      fireCount++;
    }
  }

  // --- 시나리오 1: 평시 (화재 0곳) ---
  if (fireCount == 0) {
    resetToSafeMode(); // 모든 LED 끄기
    Serial.println("SYSTEM: SAFE (All LEDs OFF)");
    return; // 함수 종료
  }

  // --- 시나리오 2: 최악 상황 (화재 4곳 모두) (FR-07) ---
  if (fireCount == 4) {
    Serial.println("SYSTEM: DANGER! (All LEDs RED)");
    // A, B, C, D, CS 모두 RED
    for (int i = 0; i < 5; i++) {
      setLED(i, STATE_RED);
    }
    return; // 함수 종료
  }

  // --- 시나리오 3: 1~3곳 화재 (FR-05, FR-06) ---
  Serial.println("SYSTEM: Partial Fire Detected.");
  
  // LED A: 항상 GREEN (FR-05)
  setLED(0, STATE_GREEN); // 0 = A

  // LED C: 항상 GREEN (FR-05)
  setLED(2, STATE_GREEN); // 2 = C

  // LED CS: 항상 GREEN (FR-06)
  setLED(4, STATE_GREEN); // 4 = CS

  // LED D: 102호(fireState[0]) 또는 104호(fireState[2]) 화재 시 RED (FR-05)
  if (fireState[0] || fireState[2]) {
    setLED(3, STATE_RED); // 3 = D
  } else {
    setLED(3, STATE_GREEN);
  }

  // LED B: 107호(fireState[3]) 또는 109호(fireState[1]) 화재 시 RED (FR-05)
  if (fireState[3] || fireState[1]) {
    setLED(1, STATE_RED); // 1 = B
  } else {
    setLED(1, STATE_GREEN);
  }
}


/**
 * @brief 지정된 2색 LED의 상태를 설정합니다 (공통 애노드 로직).
 * @param ledIndex LED 인덱스 (0:A, 1:B, 2:C, 3:D, 4:CS)
 * @param state 원하는 상태 (0: OFF, 1: RED, 2: GREEN)
 */
void setLED(int ledIndex, int state) {
  int redPin = ledRedPins[ledIndex];
  int greenPin = ledGreenPins[ledIndex];

  // 공통 애노드(Common Anode) 로직:
  // HIGH = 끄기, LOW = 켜기
  switch (state) {
    case STATE_OFF: // 0: 끄기
      digitalWrite(redPin, LED_OFF);
      digitalWrite(greenPin, LED_OFF);
      break;
    case STATE_RED: // 1: 빨간색
      digitalWrite(redPin, LED_ON);
      digitalWrite(greenPin, LED_OFF);
      break;
    case STATE_GREEN: // 2: 초록색
      digitalWrite(redPin, LED_OFF);
      digitalWrite(greenPin, LED_ON);
      break;
  }
}

/**
 * @brief 시스템을 평시(안전) 상태로 초기화합니다.
 * (모든 5개 구역 LED 끄기)
 */
void resetToSafeMode() {
  for (int i = 0; i < 5; i++) {
    setLED(i, STATE_OFF);
  }
}