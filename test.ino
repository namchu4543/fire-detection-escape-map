/*
  기본 LED 깜빡이기 예제
  설명: 1초 간격으로 LED가 켜졌다 꺼집니다.
*/
#define OUTPUT 0


#define HIGH 1
#define LOW 0

const int LED_PIN = 13;  // 보드 내장 LED 핀 (대부분 13번)

void setup() {
  pinMode(LED_PIN, OUTPUT);  // LED 핀을 출력 모드로 설정
}

void loop() {
  digitalWrite(LED_PIN, HIGH);  // LED 켜기
  delay(1000);                  // 1초 대기 (1000ms)
  
  digitalWrite(LED_PIN, LOW);   // LED 끄기
  delay(1000);                  // 1초 대기
}