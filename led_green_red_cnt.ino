/*
 * 핀맵 기반 5개 구역 순차 점등 테스트 (A->B->C->D->Center)
 * - ★★★ 각 그룹이 (빨강 1초 -> 초록 1초) 순서로 켜짐 ★★★
 * - LED 타입: 공통 애노드 (Common Anode)
 * - LED 공통 핀: 아두이노 5V (GND 아님!)
 * - 로직: LOW 신호로 켜짐 (ON), HIGH 신호로 꺼짐 (OFF)
 */

// --- LED 로직 정의 (공통 애노드) ---
const int LED_ON = LOW;
const int LED_OFF = HIGH;

// --- 핀 정의 (제공된 핀맵 기준) ---
// (색상1 = Red, 색상2 = Green/Yellow로 가정)
// Center LED
const int CENTER_1 = 8; // Red
const int CENTER_2 = 6; // Green/Yellow

// LED A
const int A_1 = 9; // Red
const int A_2 = 10; // Green/Yellow

// LED B
const int B_1 = 4; // Red
const int B_2 = 3; // Green/Yellow

// LED C
const int C_1 = 2; // Red
const int C_2 = A0; // Green/Yellow

// LED D
const int D_1 = A1; // Red
const int D_2 = A2; // Green/Yellow

// --- 점등 간격 설정 (ms) ---
const int blinkDelay = 1000; // 1000ms = 1초

// 핀들을 배열로 관리하면 코드가 깔끔해집니다.
const int ledPins[] = {CENTER_1, CENTER_2, A_1, A_2, B_1, B_2, C_1, C_2, D_1, D_2};
const int numLedPins = 10; // 핀의 총 개수 (코드 수정됨)

void setup() {
  // --- 모든 LED 핀을 출력으로 설정 ---
  // (총 10개의 핀)
  for (int i = 0; i < numLedPins; i++) { // (코드 수정됨)
    pinMode(ledPins[i], OUTPUT);
  }

  // --- 시작할 때 모든 LED 끄기 ---
  allLedsOff();
}

void loop() {
  // 1. A 켜기 (빨강 1초 -> 초록 1초)
  allLedsOff(); // (이전 Center 끄기)
  digitalWrite(A_1, LED_ON);  // A Red ON
  delay(blinkDelay);

  digitalWrite(A_1, LED_OFF); // A Red OFF
  digitalWrite(A_2, LED_ON);  // A Green ON
  delay(blinkDelay);

  // 2. B 켜기 (빨강 1초 -> 초록 1초)
  allLedsOff(); // (A 끄기)
  digitalWrite(B_1, LED_ON);  // B Red ON
  delay(blinkDelay);

  digitalWrite(B_1, LED_OFF); // B Red OFF
  digitalWrite(B_2, LED_ON);  // B Green ON
  delay(blinkDelay);

  // 3. C 켜기 (빨강 1초 -> 초록 1초)
  allLedsOff(); // (B 끄기)
  digitalWrite(C_1, LED_ON);  // C Red ON
  delay(blinkDelay);

  digitalWrite(C_1, LED_OFF); // C Red OFF
  digitalWrite(C_2, LED_ON);  // C Green ON
  delay(blinkDelay);

  // 4. D 켜기 (빨강 1초 -> 초록 1초)
  allLedsOff(); // (C 끄기)
  digitalWrite(D_1, LED_ON);  // D Red ON
  delay(blinkDelay);

  digitalWrite(D_1, LED_OFF); // D Red OFF
  digitalWrite(D_2, LED_ON);  // D Green ON
  delay(blinkDelay);

  // 5. Center 켜기 (빨강 1초 -> 초록 1초)
  allLedsOff(); // (D 끄기)
  digitalWrite(CENTER_1, LED_ON);  // Center Red ON
  delay(blinkDelay);

  digitalWrite(CENTER_1, LED_OFF); // Center Red OFF
  digitalWrite(CENTER_2, LED_ON);  // Center Green ON
  delay(blinkDelay);
  
  // (loop()가 다시 시작되며 allLedsOff()가 Center를 끄고 1번(A)으로 돌아감)
}

/**
 * @brief 모든 LED 핀을 끕니다. (HIGH 신호 전송)
 */
void allLedsOff() {
  for (int i = 0; i < numLedPins; i++) { // (코드 수정됨)
    digitalWrite(ledPins[i], LED_OFF);
  }
}