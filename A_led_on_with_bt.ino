/*
 * LED A (D9, D10) 블루투스 제어 테스트
 * - 블루투스(HC-06) 핀: D11(RX), D12(TX)
 * - LED A 핀: D9, D10
 * - LED 로직: 공통 애노드 (LOW = ON, HIGH = OFF)
 */

#include <SoftwareSerial.h>

// --- 핀 정의 (제공된 핀맵 기준) ---

// HC-06 블루투스 핀
const int BT_RX_PIN = 11; // 아두이노 RX <- HC-06 TX
const int BT_TX_PIN = 12; // 아두이노 TX -> HC-06 RX (★★★ 전압분배 저항 필수 ★★★)

// 블루투스 시리얼 객체 선언
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN);

// LED A 핀
const int A_RED = 9;
const int A_YELLOW = 10; // (또는 초록색/색상2)

// --- LED 로직 정의 (공통 애노드) ---
const int LED_ON = LOW;
const int LED_OFF = HIGH;

// --- 나머지 LED 핀 (작동 안 하도록 끌 때 사용) ---
const int CENTER_1 = 8;
const int CENTER_2 = 6;
const int B_1 = 4;
const int B_2 = 3;
const int C_1 = 2;
const int C_2 = A0;
const int D_1 = A1;
const int D_2 = A2;
const int ledOffPins[] = {CENTER_1, CENTER_2, B_1, B_2, C_1, C_2, D_1, D_2};
const int numOffPins = 8;


void setup() {
  // 시리얼 모니터 시작 (디버깅용)
  Serial.begin(9600);
  Serial.println("Bluetooth LED A Test");

  // 블루투스 시리얼 시작
  btSerial.begin(9600);

  // --- LED A 핀 출력 설정 ---
  pinMode(A_RED, OUTPUT);
  pinMode(A_YELLOW, OUTPUT);

  // --- 나머지 핀들도 출력으로 설정 (확실하게 끄기 위해) ---
  for (int i = 0; i < numOffPins; i++) {
    pinMode(ledOffPins[i], OUTPUT);
  }

  // --- 모든 LED 끄고 시작 ---
  allLedsOff();
}

void loop() {
  
  // 블루투스(HC-06)로부터 데이터가 수신되었는지 확인
  if (btSerial.available()) {
    char data = btSerial.read(); // 수신된 데이터 1바이트를 읽음

    // 시리얼 모니터에 수신된 값 출력 (디버깅용)
    Serial.print("Data Received: ");
    Serial.println(data);

    // 수신된 값에 따라 LED A 제어
    switch (data) {
      case '1':
        // '1'을 받으면 LED A 켜기
        Serial.println("LED A ON");
        digitalWrite(A_RED, LED_ON);
        digitalWrite(A_YELLOW, LED_ON);
        break;

      case '0':
        // '0'을 받으면 LED A 끄기
        Serial.println("LED A OFF");
        digitalWrite(A_RED, LED_OFF);
        digitalWrite(A_YELLOW, LED_OFF);
        break;
        
      // (다른 문자는 무시)
    }
  }
}

/**
 * @brief 모든 10개의 LED 핀을 끕니다. (HIGH 신호 전송)
 */
void allLedsOff() {
  // A 끄기
  digitalWrite(A_RED, LED_OFF);
  digitalWrite(A_YELLOW, LED_OFF);
  
  // 나머지 끄기
  for (int i = 0; i < numOffPins; i++) {
    digitalWrite(ledOffPins[i], LED_OFF);
  }
}