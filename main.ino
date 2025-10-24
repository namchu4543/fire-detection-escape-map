/*
 * 프로젝트명: 동적 화재 피난 유도 시스템
 * MCU: Arduino UNO
 * 설명: 4개 구역의 화재 감지 센서와 블루투스 신호를 바탕으로
 * LCD에는 대피 방향을, 2색 LED에는 구역별 위험(빨강)/안전(초록)을
 * 표시하는 시스템입니다.
 *
 * ★★★ 새로운 대피 로직 (N-1) ★★★
 * - 평시: 모든 LED 꺼짐 / LCD "System OK"
 * - 화재 발생 시 (예: A구역):
 * - LED A: 빨간색 (위험)
 * - LED B, C, D: 초록색 (안전한 대피로)
 * - LCD: "Fire in A!" / "Evac via B,C,D"
 * - 2개 이상 화재 시 (예: A, B 구역):
 * - LED A, B: 빨간색
 * - LED C, D: 초록색
 * - LCD: "Fire in A,B!" / "Evac via C,D"
 * - 4개 구역 전체 화재 시:
 * - LED A, B, C, D: 빨간색
 * - LCD: "DANGER! ALL ZONES" / "STAY PUT! EVAC!"
 *
 * --- 최종 핀맵 (확정) ---
 * - I2C LCD: A4(SDA), A5(SCL)
 * - HC-06 BT: D11(RX), D12(TX) (D12<->HC-06 RX핀 전압분배 필수)
 * - 불꽃 센서 A,B,C,D: D7, D8, D6, D5
 * - 2색 LED A (R/G): D9, D10
 * - 2색 LED B (R/G): D4, D3
 * - 2색 LED C (R/G): D2, A0
 * - 2색 LED D (R/G): A1, A2
 *
 * --- LED 상세 ---
 * - 4개 모두 '공통 애노드(Common Anode)' 타입으로 가정.
 * - HIGH (5V) 신호 = 꺼짐 (OFF)
 * - LOW (GND) 신호 = 켜짐 (ON)
 */

// 1. 라이브러리 포함
// ==========================================================
#include <Wire.h>               // I2C 통신을 위한 라이브러리
#include <LiquidCrystal_I2C.h>  // I2C LCD 제어 라이브러리
#include <SoftwareSerial.h>     // 블루투스 통신을 위한 소프트웨어 시리얼 라이브러리

// 2. 핀 번호 정의 (핀맵)
// ==========================================================
// 불꽃 감지 센서 핀 (A, B, C, D)
const int FLAME_SENSOR_A = 7;
const int FLAME_SENSOR_B = 8;
const int FLAME_SENSOR_C = 6;
const int FLAME_SENSOR_D = 5;

// HC-06 블루투스 핀 (아두이노 기준 RX, TX)
const int BT_RX_PIN = 11; // 아두이노 RX <- HC-06 TX
const int BT_TX_PIN = 12; // 아두이노 TX -> HC-06 RX (★★★ 전압분배 저항 필수 ★★★)

// 2색 LED 핀 (A, B, C, D)
// ★★★ 중요: 핀맵의 '색상1'을 RED, '색상2'를 GREEN으로 가정합니다.
//           만약 색상이 반대로 나오면 아래 두 줄의 값을 서로 바꾸세요.
const int LED_A_RED = 9;   // LED A (Red)
const int LED_A_GREEN = 10;  // LED A (Green)

const int LED_B_RED = 4;   // LED B (Red)
const int LED_B_GREEN = 3;   // LED B (Green)

const int LED_C_RED = 2;   // LED C (Red)
const int LED_C_GREEN = A0;  // LED C (Green) - A0 = 14번핀

const int LED_D_RED = A1;  // LED D (Red) - A1 = 15번핀
const int LED_D_GREEN = A2;  // LED D (Green) - A2 = 16번핀


// 3. 전역 변수 및 객체 선언
// ==========================================================
// LCD 객체 선언 (주소, 칸수, 줄수)
// ★★★ LCD 주소가 0x27이 아닐 수 있습니다. (0x3F 등)
// I2C Scanner 예제로 실제 주소를 꼭 확인하세요!
LiquidCrystal_I2C lcd(0x27, 16, 2);

// 블루투스 시리얼 객체 선언
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN);

// 센서 감지 상태 정의 (Active LOW: 불꽃 감지 시 LOW 신호)
const int FIRE_DETECTED = LOW;
const int NO_FIRE = HIGH;

// LCD 메시지 상수 (평시)
const char* MSG_OK_1 = "  System OK   ";
const char* MSG_OK_2 = "  Safe Zone   ";

// LED 제어를 위한 핀 배열 (setLED 함수에서 사용)
const int ledRedPins[4] = {LED_A_RED, LED_B_RED, LED_C_RED, LED_D_RED};
const int ledGreenPins[4] = {LED_A_GREEN, LED_B_GREEN, LED_C_GREEN, LED_D_GREEN};

// LED 상태 상수 (가독성)
const int LED_OFF = 0;
const int LED_RED = 1;
const int LED_GREEN = 2;

// 화재 상태 저장 변수
bool btFireState[4] = {false, false, false, false};       // 블루투스에 의한 가상 화재 상태 (A,B,C,D)
bool lastCombinedFireState[4] = {false, false, false, false}; // 이전 루프의 최종 화재 상태 (LCD 깜빡임 방지용)


// 4. setup() 함수 (초기 설정)
// ==========================================================
void setup() {
  // 시리얼 모니터 시작 (디버깅용)
  Serial.begin(9600);
  Serial.println("Fire Escape System Booting...");

  // 블루투스 시리얼 시작 (HC-06 기본 보드레이트: 9600)
  btSerial.begin(9600);

  // LCD 초기화 및 백라이트 켜기
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // 핀 모드 설정: 불꽃 감지 센서 (입력)
  pinMode(FLAME_SENSOR_A, INPUT);
  pinMode(FLAME_SENSOR_B, INPUT);
  pinMode(FLAME_SENSOR_C, INPUT);
  pinMode(FLAME_SENSOR_D, INPUT);

  // 핀 모드 설정: 2색 LED 핀 8개 (출력)
  for (int i = 0; i < 4; i++) {
    pinMode(ledRedPins[i], OUTPUT);
    pinMode(ledGreenPins[i], OUTPUT);
  }

  // 시스템 초기 상태(평시)로 설정
  resetToSafeMode();

  Serial.println("System Ready.");
}

// 5. loop() 함수 (메인 루프)
// ==========================================================
void loop() {
  // 1. 블루투스 입력 확인 (가상 화재 신호 업데이트)
  checkBluetooth();

  // 2. 물리적 화재 센서 값 읽기
  bool physicalFireState[4];
  checkFireSensors(physicalFireState);

  // 3. 최종 화재 상태 결정 (물리 센서 OR 블루투스 신호)
  // (하나라도 true이면 화재로 간주)
  bool combinedFireState[4];
  bool stateChanged = false; // 상태 변경 감지 플래그
  
  for (int i = 0; i < 4; i++) {
    combinedFireState[i] = physicalFireState[i] || btFireState[i];
    
    // 이전 상태와 비교하여 변경점이 있는지 확인 (LCD 깜빡임 방지)
    if (combinedFireState[i] != lastCombinedFireState[i]) {
      stateChanged = true;
    }
  }

  // 4. 시스템 상태가 변경되었을 때만 LED 및 LCD 업데이트
  if (stateChanged) {
    updateSystemOutputs(combinedFireState); // LED 및 LCD 업데이트 함수 호출

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
  pSensors[0] = (digitalRead(FLAME_SENSOR_A) == FIRE_DETECTED);
  pSensors[1] = (digitalRead(FLAME_SENSOR_B) == FIRE_DETECTED);
  pSensors[2] = (digitalRead(FLAME_SENSOR_C) == FIRE_DETECTED);
  pSensors[3] = (digitalRead(FLAME_SENSOR_D) == FIRE_DETECTED);
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
      case 'a': // A구역 가상 화재
        btFireState[0] = true;
        break;
      case 'b': // B구역 가상 화재
        btFireState[1] = true;
        break;
      case 'c': // C구역 가상 화재
        btFireState[2] = true;
        break;
      case 'd': // D구역 가상 화재
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
 * @brief ★★★ [로직 수정됨] ★★★
 * 최종 화재 상태를 바탕으로 LED와 LCD를 N-1 방식으로 업데이트합니다.
 * @param fireState 최종 화재 상태 bool 배열 (크기 4)
 */
void updateSystemOutputs(bool fireState[]) {
  bool anyFire = false;
  int fireCount = 0;

  // 1. 화재 발생 여부 및 화재 지점 수 확인
  for (int i = 0; i < 4; i++) {
    if (fireState[i]) {
      anyFire = true;
      fireCount++;
    }
  }

  // 2. 시나리오 0: 화재 없음 (평시)
  if (!anyFire) {
    resetToSafeMode(); // 평시 상태로 리셋 (LED Off, LCD "System OK")
    return; // 함수 종료
  }

  // --- (여기부터는 화재가 1곳 이상 발생한 경우) ---

  // 3. LED 업데이트 (새로운 N-1 로직 적용)
  Serial.print("Fire State: ");
  for (int i = 0; i < 4; i++) {
    if (fireState[i]) {
      // 화재가 발생한 구역 = 빨간색
      setLED(i, LED_RED);
      Serial.print("R ");
    } else {
      // 화재가 없는 구역 = 초록색 (안전한 대피로)
      setLED(i, LED_GREEN);
      Serial.print("G ");
    }
  }
  Serial.println();

  // 4. LCD 업데이트 (새로운 N-1 로직 적용)
  lcd.clear();

  if (fireCount == 4) {
    // 시나리오 2: 4곳 전체 화재
    lcd.setCursor(0, 0); lcd.print("DANGER! ALL ZONES");
    lcd.setCursor(0, 1); lcd.print("STAY PUT! EVAC!");
  } else {
    // 시나리오 1: 1~3곳 화재
    String line1 = "Fire in ";
    String line2 = "Evac via ";

    for (int i = 0; i < 4; i++) {
      char zone = 'A' + i; // 'A', 'B', 'C', 'D'
      
      if (fireState[i]) {
        // 화재 구역을 line1에 추가
        if (line1.length() > 9) line1 += ","; // "Fire in " 다음
        line1 += zone;
      } else {
        // 안전 구역을 line2에 추가
        if (line2.length() > 9) line2 += ","; // "Evac via " 다음
        line2 += zone;
      }
    }
    line1 += "!"; // "Fire in A,B!"

    lcd.setCursor(0, 0); lcd.print(line1);
    lcd.setCursor(0, 1); lcd.print(line2);
  }
}


/**
 * @brief 지정된 2색 LED의 상태를 설정합니다 (공통 애노드 로직).
 * @param ledIndex LED 인덱스 (0:A, 1:B, 2:C, 3:D)
 * @param state 원하는 상태 (0: OFF, 1: RED, 2: GREEN)
 */
void setLED(int ledIndex, int state) {
  int redPin = ledRedPins[ledIndex];
  int greenPin = ledGreenPins[ledIndex];

  // 공통 애노드(Common Anode) 로직:
  // HIGH = 끄기, LOW = 켜기
  switch (state) {
    case LED_OFF: // 0: 끄기
      digitalWrite(redPin, HIGH);
      digitalWrite(greenPin, HIGH);
      break;
    case LED_RED: // 1: 빨간색
      digitalWrite(redPin, LOW);   // 빨강 켜기
      digitalWrite(greenPin, HIGH); // 초록 끄기
      break;
    case LED_GREEN: // 2: 초록색
      digitalWrite(redPin, HIGH); // 빨강 끄기
      digitalWrite(greenPin, LOW);   // 초록 켜기
      break;
  }
}

/**
 * @brief 시스템을 평시(안전) 상태로 초기화합니다.
 * (모든 LED 끄기, LCD에 "System OK" 표시)
 */
void resetToSafeMode() {
  Serial.println("Resetting to Safe Mode.");
  
  // 모든 LED 끄기
  for (int i = 0; i < 4; i++) {
    setLED(i, LED_OFF);
  }

  // LCD 평시 메시지 표시
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(MSG_OK_1);
  lcd.setCursor(0, 1);
  lcd.print(MSG_OK_2);
}