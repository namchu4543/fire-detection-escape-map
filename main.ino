/*
  블루투스(HC-06) 2색 LED 제어 시스템
  
  - 로직: 타임아웃(Timeout) 기반
    '\n' 문자 없이, 문자 수신이 잠시(COMMAND_TIMEOUT_MS) 멈추면
    그동안 수집된 문자열을 하나의 명령어로 간주하여 처리합니다.
    (예: 'O', 'N', '6' 순서로 수신 -> 50ms 대기 -> "ON6" 명령어 처리)
*/

#include <SoftwareSerial.h>

// 1. 블루투스 핀 정의 (RX:11, TX:12)
const int BT_RX_PIN = 11; // 아두이노 RX핀 (HC-06의 TX핀과 연결)
const int BT_TX_PIN = 12; // 아두이노 TX핀 (HC-06의 RX핀과 연결, 저항분배!)
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN); // (아두이노 RX, TX)

// 2. 불꽃 감지 센서 핀
const int FLAME_PIN = 7;

// 3. LED 핀 정의 (R: Red, G: Green)
const int LED_A_R_PIN = 9;
const int LED_A_G_PIN = 10;
const int LED_B_R_PIN = 4;
const int LED_B_G_PIN = 3;
const int LED_C_R_PIN = 2;
const int LED_C_G_PIN = A0;
const int LED_D_R_PIN = A1;
const int LED_D_G_PIN = A2;
const int LED_CS_R_PIN = 8;
const int LED_CS_G_PIN = 6;

// 4. LED 상태 식별을 위한 상수
const int STATE_OFF = 0;   // 꺼짐
const int STATE_RED = 1;   // 빨강
const int STATE_GREEN = 2; // 초록

// 5. 블루투스 수신용 변수
String btCommand = ""; 
unsigned long lastCharTime = 0; // 마지막 문자를 수신한 시간
const long COMMAND_TIMEOUT_MS = 50; // 50ms(0.05초) 동안 새 문자가 없으면 타임아웃

// --------------------------------------------------
//  setup() : 초기 설정
// --------------------------------------------------
void setup() {
  Serial.begin(9600);
  Serial.println("시스템 준비 완료. (타임아웃 로직)");
  btSerial.begin(9600);
  
  pinMode(FLAME_PIN, INPUT);
  
  pinMode(LED_A_R_PIN, OUTPUT);
  pinMode(LED_A_G_PIN, OUTPUT);
  pinMode(LED_B_R_PIN, OUTPUT);
  pinMode(LED_B_G_PIN, OUTPUT);
  pinMode(LED_C_R_PIN, OUTPUT);
  pinMode(LED_C_G_PIN, OUTPUT);
  pinMode(LED_D_R_PIN, OUTPUT);
  pinMode(LED_D_G_PIN, OUTPUT);
  pinMode(LED_CS_R_PIN, OUTPUT);
  pinMode(LED_CS_G_PIN, OUTPUT);

  setAllLedsOff();
}

// --------------------------------------------------
//  loop() : 메인 루프 (타임아웃 로직 적용)
// --------------------------------------------------
void loop() {
  
  // --- 1. 문자 수신 처리 ---
  if (btSerial.available()) {
    char c = btSerial.read(); // 1글자 읽기
    
    // 디버깅: 수신된 문자 즉시 표시
    Serial.print("DEBUG -> 수신: '");
    Serial.print(c);
    Serial.print("' (");
    Serial.print((int)c);
    Serial.println(")");

    // '\n'이나 '\r' 같은 제어 문자는 무시하고, 화면에 보이는 문자만 명령어에 추가
    if (c >= ' ') { 
      btCommand += c; 
      lastCharTime = millis(); // (중요) 마지막 문자 수신 시간 갱신
    }
  }

  // --- 2. 타임아웃 확인 처리 ---
  // (btSerial.available() 바깥에 있어야 함. 수신이 없을 때 타임아웃을 확인)
  
  // 조건: (명령어가 1글자 이상 있고) AND (현재시간 - 마지막문자시간 > 타임아웃시간)
  if (btCommand.length() > 0 && (millis() - lastCharTime > COMMAND_TIMEOUT_MS)) {
    
    Serial.print("명령어 완성 (타임아웃): ");
    Serial.println(btCommand);

    // 완성된 문자열(예: "ON6")로 명령어 처리 함수 호출
    processCommand(btCommand);
    
    // 다음 명령어를 받기 위해 버퍼를 비웁니다.
    btCommand = ""; 
  }

  // (선택) 불꽃 감지 센서 로직
  // ...
}

// --------------------------------------------------
//  processCommand() : 수신된 명령어 처리
// (로그에서 확인된 "ONx"와 기존 "Nx/Fx"를 모두 처리하도록 강화)
// --------------------------------------------------
void processCommand(String command) {
  if (command == "N1" || command == "ON1") { 
    setLedA(STATE_RED);
  } else if (command == "F1" || command == "OFF1") {
    setLedA(STATE_GREEN);
  } else if (command == "N2" || command == "ON2") {
    setLedB(STATE_RED);
  } else if (command == "F2" || command == "OFF2") {
    setLedB(STATE_GREEN);
  } else if (command == "N3" || command == "ON3") {
    setLedC(STATE_RED);
  } else if (command == "F3" || command == "OFF3") {
    setLedC(STATE_GREEN);
  } else if (command == "N4" || command == "ON4") {
    setLedD(STATE_RED);
  } else if (command == "F4" || command == "OFF4") {
    setLedD(STATE_GREEN);
  } else if (command == "N5" || command == "ON5") {
    setLedCS(STATE_RED);
  } else if (command == "F5" || command == "OFF5") {
    setLedCS(STATE_GREEN);
  } else if (command == "N6" || command == "ON6") { // 평소 ON (ALL GREEN)
    setAllLedsGreen();
  } else if (command == "F6" || command == "OFF6") { // 평소 OFF (ALL OFF)
    setAllLedsOff();
  } else {
    Serial.print("알 수 없는 명령어: ");
    Serial.println(command);
  }
}

// --------------------------------------------------
//  LED 제어 헬퍼 함수 (이하 동일)
// --------------------------------------------------

void setLedA(int state) {
  if (state == STATE_RED) {
    digitalWrite(LED_A_R_PIN, LOW);
    digitalWrite(LED_A_G_PIN, HIGH);
  } else if (state == STATE_GREEN) {
    digitalWrite(LED_A_R_PIN, HIGH);
    digitalWrite(LED_A_G_PIN, LOW);
  } else { // STATE_OFF
    digitalWrite(LED_A_R_PIN, HIGH);
    digitalWrite(LED_A_G_PIN, HIGH);
  }
}

void setLedB(int state) {
  if (state == STATE_RED) {
    digitalWrite(LED_B_R_PIN, LOW);
    digitalWrite(LED_B_G_PIN, HIGH);
  } else if (state == STATE_GREEN) {
    digitalWrite(LED_B_R_PIN, HIGH);
    digitalWrite(LED_B_G_PIN, LOW);
  } else {
    digitalWrite(LED_B_R_PIN, HIGH);
    digitalWrite(LED_B_G_PIN, HIGH);
  }
}

void setLedC(int state) {
  if (state == STATE_RED) {
    digitalWrite(LED_C_R_PIN, LOW);
    digitalWrite(LED_C_G_PIN, HIGH);
  } else if (state == STATE_GREEN) {
    digitalWrite(LED_C_R_PIN, HIGH);
    digitalWrite(LED_C_G_PIN, LOW);
  } else {
    digitalWrite(LED_C_R_PIN, HIGH);
    digitalWrite(LED_C_G_PIN, HIGH);
  }
}

void setLedD(int state) {
  if (state == STATE_RED) {
    digitalWrite(LED_D_R_PIN, LOW);
    digitalWrite(LED_D_G_PIN, HIGH);
  } else if (state == STATE_GREEN) {
    digitalWrite(LED_D_R_PIN, HIGH);
    digitalWrite(LED_D_G_PIN, LOW);
  } else {
    digitalWrite(LED_D_R_PIN, HIGH);
    digitalWrite(LED_D_G_PIN, HIGH);
  }
}

void setLedCS(int state) {
  if (state == STATE_RED) {
    digitalWrite(LED_CS_R_PIN, LOW);
    digitalWrite(LED_CS_G_PIN, HIGH);
  } else if (state == STATE_GREEN) {
    digitalWrite(LED_CS_R_PIN, HIGH);
    digitalWrite(LED_CS_G_PIN, LOW);
  } else {
    digitalWrite(LED_CS_R_PIN, HIGH);
    digitalWrite(LED_CS_G_PIN, HIGH);
  }
}

void setAllLedsGreen() {
  setLedA(STATE_GREEN);
  setLedB(STATE_GREEN);
  setLedC(STATE_GREEN);
  setLedD(STATE_GREEN);
  setLedCS(STATE_GREEN);
}

void setAllLedsOff() {
  setLedA(STATE_OFF);
  setLedB(STATE_OFF);
  setLedC(STATE_OFF);
  setLedD(STATE_OFF);
  setLedCS(STATE_OFF);
}
