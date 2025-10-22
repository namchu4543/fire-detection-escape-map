/************************************************************
 * 프로젝트: LED 기반 화재감지형 피난안내도 (확장형 기본 틀, v2)
 * 파일: main.ino (단일 파일)
 * 의도:
 *  - setup/loop 최소화 + 태스크(논블로킹) 분리
 *  - 센서 안정화(이동평균) + 히스테리시스 임계
 *  - 상태머신(정상/주의/위험)으로 렌더/부저/경로로직 일원화
 *  - 네오픽셀/추가 센서(Flame, DHT, RTC/SD) 손쉽게 확장
 ************************************************************/

/* ====================== 기능 스위치(빌드 옵션) ======================= */
// 필요시 1로 바꾸고 라이브러리 추가 설치
#define USE_NEOPIXEL   0   // 1: Adafruit_NeoPixel 사용, 0: 보드 LED로 대체
#define USE_FLAME      0   // 1: 불꽃센서 사용 예시
#define USE_DHT        0   // 1: DHT22 사용 예시

/* ===================== 보드/플랫폼 호환 가드 ======================== */
#ifndef IRAM_ATTR
  #define IRAM_ATTR /* AVR(UNO 등)에서는 무시 */
#endif

/* =========================== 핀 설정 ================================ */
// 프로젝트에 맞게 수정
const uint8_t PIN_LED_STRIP = 6;     // WS2812B 데이터핀 (예정)
const uint8_t PIN_BUZZER    = 9;     // 부저
const uint8_t PIN_BTN_INT   = 2;     // 인터럽트 버튼(예: INT0)
const uint8_t PIN_MQ2_AO    = A0;    // MQ-2 아날로그 입력
const uint8_t PIN_STATUSLED = 13;    // 보드 내장 LED(동작 확인)

/*재근이 ㅇㅇㅇㅇㅇㅇㅇㅇㅇ*/

#if USE_FLAME
const uint8_t PIN_FLAME_DO  = 3;     // 불꽃센서 디지털 출력 (예시)
#endif

/* ========================= 타이밍(논블로킹) ========================= */
const uint32_t INTERVAL_READ_SENSORS = 100;   // 센서 읽기
const uint32_t INTERVAL_UPDATE_LOGIC = 100;   // 상태/경로
const uint32_t INTERVAL_RENDER_LEDS  = 25;    // LED 애니메이션
const uint32_t INTERVAL_BEEP_ENGINE  = 10;    // 부저 패턴 엔진
const uint32_t INTERVAL_LOG_OUTPUT   = 500;   // 시리얼 로그

uint32_t tReadSensors=0, tUpdateLogic=0, tRenderLEDs=0, tBeep=0, tLogOutput=0;

/* ========================= 전역 상태/구조체 ========================= */
// 시스템 모드(예: 수동/자동)
enum SystemMode : uint8_t { MODE_AUTO=0, MODE_MANUAL=1 };

// 경보 상태머신
enum AlarmState : uint8_t { STATE_NORMAL=0, STATE_CAUTION=1, STATE_DANGER=2 };

// 센서 버퍼(이동평균)
const uint8_t MQ2_WIN = 8;     // 이동평균 창 크기
int mq2Buf[MQ2_WIN] = {0};
uint8_t mq2Idx = 0;
long mq2Sum = 0;
int g_mq2Raw = 0;              // 최근 읽은 생(raw)
int g_mq2Avg = 0;              // 이동평균

#if USE_FLAME
bool g_flame = false;          // 불꽃 감지 여부
#endif

// 히스테리시스 임계 (예시값, 프로젝트에 맞게 보정)
const int MQ2_CAUTION_ON  = 320;
const int MQ2_CAUTION_OFF = 300;
const int MQ2_DANGER_ON   = 380;
const int MQ2_DANGER_OFF  = 360;

// 경로/차단 (데모)
bool g_blocked   = false;  // 차단 영역
int  g_pathDir   = +1;     // -1 좌 / +1 우 (데모 토글)
AlarmState g_state = STATE_NORMAL;
SystemMode g_mode  = MODE_AUTO;

// 버튼 인터럽트
volatile bool     g_btnInterruptFlag = false;
volatile uint32_t g_btnLastTick      = 0;     // 디바운스

/* ========================= 네오픽셀(옵션) =========================== */
#if USE_NEOPIXEL
  #include <Adafruit_NeoPixel.h>
  const uint16_t NUM_PIXELS = 30; // 세그먼트/존 구성에 맞춰 조정
  Adafruit_NeoPixel strip(NUM_PIXELS, PIN_LED_STRIP, NEO_GRB + NEO_KHZ800);
  // ▷ 세그먼트/존 테이블을 배열로 만들어서 "경로만 녹색 러닝" 등의 연출에 활용
  //   예: const uint8_t SEG_EXIT[] = {0,1,2,3,4,5};
#endif

/* =========================== 부저 패턴 ============================== */
/*
 * 간단 패턴 엔진(논블로킹)
 * - 상태에 따라 다른 패턴을 구성(주파수, 구간, 반복)
 * - tone()은 비동기지만 지연을 넣지 않고 스케줄만 관리
 */
struct BeepStep { uint16_t freq; uint16_t dur; uint16_t gap; };
const BeepStep PATTERN_NORMAL[]  = {{0,   0,  0}};                      // 무음
const BeepStep PATTERN_CAUTION[] = {{2000,120,180}, {0,0,400}};         // 비프-쉼 반복
const BeepStep PATTERN_DANGER[]  = {{2500,160,90}, {2500,160,300}};     // 빠른 이중 비프

const BeepStep* g_beepPtr = PATTERN_NORMAL;
uint8_t g_beepIdx = 0;
uint32_t g_beepTick = 0;
bool g_beepOn = false;

/* ========================= 전방 선언 ================================ */
// 초기화
void initPins();
void initSerial();
void initInterrupts();
void initNeopixelIfEnabled();

// 태스크
void readSensorsTask();
void updateLogicTask();
void renderLedsTask();
void beepEngineTask();
void logTask();

// 상태처리
void updateStateFromSensors();
void applyStateSideEffects(AlarmState s);

// 유틸
bool timePassed(uint32_t &tPrev, uint32_t interval);

// ISR
void IRAM_ATTR isrButton();

/* ============================ setup ================================ */
void setup() {
  initPins();
  initSerial();
  initInterrupts();
  initNeopixelIfEnabled();

  Serial.println(F("System Initialized."));
  digitalWrite(PIN_STATUSLED, HIGH);
  delay(80);
  digitalWrite(PIN_STATUSLED, LOW);
}

/* ============================= loop ================================ */
void loop() {
  // 1) 인터럽트 이벤트(최우선)
  if (g_btnInterruptFlag) {
    g_btnInterruptFlag = false;
    // TODO: 버튼 행동 정의
    //  - 짧게: 수동/자동 토글
    //  - 길게: 시스템 리셋 등
    g_mode = (g_mode==MODE_AUTO) ? MODE_MANUAL : MODE_AUTO;
    Serial.print(F("[BTN] mode="));
    Serial.println(g_mode==MODE_AUTO?"AUTO":"MANUAL");
  }

  // 2) 태스크 스케줄
  if (timePassed(tReadSensors, INTERVAL_READ_SENSORS)) readSensorsTask();
  if (timePassed(tUpdateLogic, INTERVAL_UPDATE_LOGIC)) updateLogicTask();
  if (timePassed(tRenderLEDs,  INTERVAL_RENDER_LEDS )) renderLedsTask();
  if (timePassed(tBeep,        INTERVAL_BEEP_ENGINE))  beepEngineTask();
  if (timePassed(tLogOutput,   INTERVAL_LOG_OUTPUT ))  logTask();
}

/* ========================== 초기화 구현 ============================= */
void initPins() {
  pinMode(PIN_STATUSLED, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BTN_INT, INPUT_PULLUP);
  // MQ-2: 아날로그 입력이므로 pinMode 불필요
#if USE_FLAME
  pinMode(PIN_FLAME_DO, INPUT); // 필요시 INPUT_PULLUP
#endif
}

void initSerial() {
  Serial.begin(115200);
  while(!Serial) { ; } // 일부 보드에서만 의미
}

void initInterrupts() {
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_INT), isrButton, FALLING);
}

void initNeopixelIfEnabled() {
#if USE_NEOPIXEL
  strip.begin();
  strip.setBrightness(80); // TODO: 환경에 맞게 조정
  strip.show();            // 모두 OFF
#endif
}

/* =========================== 태스크 구현 ============================ */
void readSensorsTask() {
  // ① MQ-2 가스센서 읽기 + 이동평균
  int raw = analogRead(PIN_MQ2_AO);
  g_mq2Raw = raw;
  mq2Sum -= mq2Buf[mq2Idx];
  mq2Buf[mq2Idx] = raw;
  mq2Sum += raw;
  mq2Idx = (mq2Idx + 1) % MQ2_WIN;
  g_mq2Avg = (int)(mq2Sum / MQ2_WIN);

#if USE_FLAME
  // ② 불꽃센서(선택)
  g_flame = digitalRead(PIN_FLAME_DO) == LOW; // 센서 모델에 따라 Active Low
#endif

#if USE_DHT
  // ③ DHT22 (라이브러리 포함 시)
  //    float t = dht.readTemperature();
  //    float h = dht.readHumidity();
  //    if (!isnan(t) && !isnan(h)) { ... 위험 판정 반영 ... }
#endif
}

void updateLogicTask() {
  // ① 센서 -> 상태머신 갱신
  updateStateFromSensors();

  // ② 차단/경로(데모): 실제는 지도/세그먼트 기반으로 계산
  g_blocked = (g_state == STATE_DANGER);

  // 방향 데모 토글
  static uint32_t lastToggle = 0;
  if (millis() - lastToggle > 2500) {
    g_pathDir = -g_pathDir;
    lastToggle = millis();
  }
}

void renderLedsTask() {
#if USE_NEOPIXEL
  // ===================== 네오픽셀 구현부 ======================
  // 상태별 컬러/패턴
  //  - NORMAL : 녹색 점등(약한 브리딩)
  //  - CAUTION: 주황색 점멸
  //  - DANGER : 빨강 점멸 + 출구 방향 러닝(녹색)
  static uint8_t phase = 0;
  phase++;

  strip.clear();

  if (g_state == STATE_NORMAL) {
    uint8_t bri = 50 + (uint8_t)(30.0 * (1+sin(phase*0.05))); // 간단 브리딩
    for (uint16_t i=0;i<strip.numPixels();++i) strip.setPixelColor(i, strip.Color(0, bri, 0));
  } else if (g_state == STATE_CAUTION) {
    bool blink = ((millis()/300)%2)==0;
    for (uint16_t i=0;i<strip.numPixels();++i) strip.setPixelColor(i, blink?strip.Color(255,80,0):0);
  } else { // DANGER
    bool blink = ((millis()/180)%2)==0;
    for (uint16_t i=0;i<strip.numPixels();++i) strip.setPixelColor(i, blink?strip.Color(255,0,0):0);
    // ▷ 출구 방향 러닝(간단 예시)
    static uint16_t head=0;
    head = (head + (g_pathDir>0?1:strip.numPixels()-1)) % strip.numPixels();
    strip.setPixelColor(head, strip.Color(0, 180, 0));
  }

  strip.show();
#else
  // =================== 네오픽셀 미사용(보드 LED) ===================
  static bool blink=false; blink=!blink;
  if (g_state == STATE_NORMAL) {
    digitalWrite(PIN_STATUSLED, HIGH);
  } else if (g_state == STATE_CAUTION) {
    digitalWrite(PIN_STATUSLED, blink?HIGH:LOW);        // 느린 점멸
  } else {
    digitalWrite(PIN_STATUSLED, (millis()/120)%2?HIGH:LOW); // 빠른 점멸
  }
#endif
}

void beepEngineTask() {
  // 상태에 따라 패턴 포인터 갱신
  const BeepStep* target =
    (g_state==STATE_NORMAL)? PATTERN_NORMAL :
    (g_state==STATE_CAUTION)?PATTERN_CAUTION : PATTERN_DANGER;

  if (target != g_beepPtr) {
    g_beepPtr = target;
    g_beepIdx = 0;
    g_beepOn  = false;
    g_beepTick= millis();
  }

  // 패턴 구동(논블로킹)
  const BeepStep step = g_beepPtr[g_beepIdx];
  if (step.freq==0 && step.dur==0 && step.gap==0) {
    noTone(PIN_BUZZER);
    return; // 무음 패턴
  }

  uint32_t now = millis();
  if (!g_beepOn) {
    // gap 진행 중
    if (now - g_beepTick >= step.gap) {
      // 톤 켜기
      if (step.freq>0 && step.dur>0) tone(PIN_BUZZER, step.freq, step.dur);
      g_beepOn = true;
      g_beepTick = now;
    }
  } else {
    // dur 경과 시 다음 스텝
    if (now - g_beepTick >= step.dur) {
      g_beepOn = false;
      g_beepIdx++;
      // 배열 끝이면 처음으로
      // (마지막 요소 다음에 {0,0,?} 같은 휴지 추가 시 자연 반복)
      if (g_beepPtr[g_beepIdx].freq==0 && g_beepPtr[g_beepIdx].dur==0 && g_beepPtr[g_beepIdx].gap==0) {
        g_beepIdx = 0;
      }
      g_beepTick = now;
    }
  }
}

void logTask() {
  Serial.print(F("[MQ2 raw/avg] "));
  Serial.print(g_mq2Raw); Serial.print(F("/")); Serial.print(g_mq2Avg);

#if USE_FLAME
  Serial.print(F("  [flame] ")); Serial.print(g_flame);
#endif

  Serial.print(F("  [state] "));
  Serial.print( (g_state==STATE_NORMAL)?"NORMAL":(g_state==STATE_CAUTION)?"CAUTION":"DANGER" );
  Serial.print(F("  [blocked] ")); Serial.print(g_blocked);
  Serial.print(F("  [pathDir] ")); Serial.print(g_pathDir);
  Serial.print(F("  [mode] ")); Serial.println(g_mode==MODE_AUTO?"AUTO":"MANUAL");
}

/* ========================== 상태/판정 로직 ========================== */
void updateStateFromSensors() {
  // ■ MQ-2 기준의 상태머신 (히스테리시스 적용)
  AlarmState prev = g_state;

  switch (g_state) {
    case STATE_NORMAL:
      if (g_mq2Avg >= MQ2_CAUTION_ON) g_state = STATE_CAUTION;
      if (g_mq2Avg >= MQ2_DANGER_ON)  g_state = STATE_DANGER;
      break;
    case STATE_CAUTION:
      if (g_mq2Avg < MQ2_CAUTION_OFF) g_state = STATE_NORMAL;
      if (g_mq2Avg >= MQ2_DANGER_ON)  g_state = STATE_DANGER;
      break;
    case STATE_DANGER:
      if (g_mq2Avg < MQ2_DANGER_OFF)  g_state = STATE_CAUTION;
      if (g_mq2Avg < MQ2_CAUTION_OFF) g_state = STATE_NORMAL;
      break;
  }

#if USE_FLAME
  // ■ 불꽃 감지 시 즉시 DANGER로 승격 (프로젝트 정책에 맞게)
  if (g_flame) g_state = STATE_DANGER;
#endif

  if (prev != g_state) {
    applyStateSideEffects(g_state);
    Serial.print(F("[STATE] ")); 
    Serial.println((g_state==STATE_NORMAL)?"NORMAL":(g_state==STATE_CAUTION)?"CAUTION":"DANGER");
  }
}

void applyStateSideEffects(AlarmState s) {
  // 상태 전환 시 1회성 부수효과(로그, 초기 비프, LED 밝기 정책 등)
  // 예: 위험 진입 시 즉시 경고음 1회
  if (s == STATE_DANGER) {
    tone(PIN_BUZZER, 3000, 200);
  }
}

/* ============================== ISR ================================ */
void IRAM_ATTR isrButton() {
  uint32_t now = millis();
  if (now - g_btnLastTick < 15) return; // 15ms 디바운스
  g_btnLastTick = now;
  g_btnInterruptFlag = true;
}

/* ============================== 유틸 =============================== */
bool timePassed(uint32_t &tPrev, uint32_t interval) {
  uint32_t now = millis();
  if (now - tPrev >= interval) { tPrev = now; return true; }
  return false;
}

/* ============================== TODO =============================== *
1) 센서 드라이버
   - MQ-2: 보정 곡선 적용(예: PPM 근사), 가열시간/안정화 고려
   - DHT22: 습도/온도 임계 연동(연기+고온 동시 시 DANGER 가중)
   - Flame: DO/아날로그 둘 다 지원(민감도 조정)

2) LED 렌더(네오픽셀 ON 시)
   - 세그먼트/존 테이블 설계: 출구/복도/차단 존 구분
   - 경로계산(테이블/BFS) 결과를 러닝 애니메이션로 반영
   - 밝기 제한/전력 계산(안전/소음 환경 고려)

3) 상태/정책
   - 상태 추가: FAULT(센서 에러), EVAC(피난 유도 강제) 등
   - 수동모드(MANUAL) 기능: 버튼 짧게/길게 구분, 초기화 루틴
   - 로깅 레벨(verbosity), SD/RTC 기록(타임스탬프 포함)

4) 안전/현장 고려
   - 부저 음량/주파수 인체공학 조정, 소방/피난 표준 검토
   - 화재 시 전원/연기로 인한 센서 드리프트 보정

5) 테스트
   - 유닛테스트성 함수 분리(판정/히스테리시스/패턴)
   - 시뮬레이션 입력(시리얼 명령으로 MQ-2 값 주입)
* ================================================================== */
