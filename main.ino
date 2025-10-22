/*
  화재감지 피난안내 – 동작 예제 (Arduino Mega)
  - 센서: Flame AO(A0), DO(D2, 인터럽트)
  - 통신: HC-06 (Serial1: RX1=D19, TX1=D18)
  - LED(2색): 내부 좌/우 (각 RED, GREEN), 외부 좌/우(녹색만)
  - 상태머신: NORMAL / CAUTION / DANGER
  - 명령: "ALARM Z1|Z2|Z3|Z4", "CLEAR", "MODE AUTO|MANUAL"
*/

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

/* ---------------- Pins ---------------- */
#define PIN_FLAME_AO    A0
#define PIN_FLAME_DO    2    // 인터럽트

// 내부 대피로(좌/우) – 2색 LED 가정 (MOSFET 게이트에 연결)
#define PIN_IL_G        5    // 내부 좌 GREEN
#define PIN_IL_R        6    // 내부 좌 RED
#define PIN_IR_G        7    // 내부 우 GREEN
#define PIN_IR_R        8    // 내부 우 RED

// 외부 방향지시(좌/우) – 녹색만 사용
#define PIN_EL_G        10   // 외부 좌 GREEN
#define PIN_ER_G        11   // 외부 우 GREEN

#define PIN_BUZZER      9    // 옵션

// 블루투스 (HC-06) – Mega의 Serial1 사용
// RX1=D19, TX1=D18 (하드웨어 고정)

/* --------------- Thresholds (tune) --------------- */
#define T_CAUTION_ON    320
#define T_CAUTION_OFF   300
#define T_DANGER_ON     380
#define T_DANGER_OFF    360

/* --------------- Timing (ms) --------------- */
const uint32_t INTERVAL_READ_SENSORS = 100;
const uint32_t INTERVAL_UPDATE_LOGIC = 100;
const uint32_t INTERVAL_RENDER_LEDS  = 30;
const uint32_t INTERVAL_BEEP_ENGINE  = 10;
const uint32_t INTERVAL_LOG_OUTPUT   = 500;

/* --------------- Globals --------------- */
enum AlarmState : uint8_t { STATE_NORMAL=0, STATE_CAUTION=1, STATE_DANGER=2 };
enum SystemMode  : uint8_t { MODE_AUTO=0,  MODE_MANUAL=1 };

volatile bool     g_irqFlag = false;    // DO 인터럽트 플래그
volatile uint32_t g_irqTick = 0;        // 디바운스용

AlarmState g_state = STATE_NORMAL;
SystemMode g_mode  = MODE_AUTO;

uint32_t tRead=0, tLogic=0, tLED=0, tBeep=0, tLog=0;

// MQ-2/Flame AO 이동평균
const uint8_t WIN = 8;
int   mqBuf[WIN]={0};
uint8_t mqIdx=0;
long  mqSum=0;
int   g_mqRaw=0, g_mqAvg=0;

// 시뮬레이션용 화재 구역 (Z1..Z4) – 좌계열: Z1/Z2, 우계열: Z3/Z4
int g_fireZone = 0; // 0=없음, 1~4

// 블루투스 입력 버퍼
String btLine;

/* --------------- Beep pattern --------------- */
struct BeepStep { uint16_t freq; uint16_t dur; uint16_t gap; };
const BeepStep PATTERN_NORMAL[]  = {{0,0,0}};
const BeepStep PATTERN_CAUTION[] = {{2000,120,180},{0,0,400}};
const BeepStep PATTERN_DANGER[]  = {{2500,160,90},{2500,160,300}};
const BeepStep* g_beepPtr = PATTERN_NORMAL;
uint8_t  g_beepIdx = 0;
uint32_t g_beepTick= 0;
bool     g_beepOn  = false;

/* --------------- Utilities --------------- */
bool timePassed(uint32_t &tPrev, uint32_t interval){
  uint32_t now = millis();
  if (now - tPrev >= interval){ tPrev = now; return true; }
  return false;
}

void IRAM_ATTR isrDO(){
  uint32_t now = millis();
  if (now - g_irqTick < 15) return;
  g_irqTick = now;
  g_irqFlag = true;
}

/* --------------- LED helpers --------------- */
// 2색 LED 제어(상호배타) – common cathode 기준(LOW=OFF/HIGH=ON)라면 로직 반전 필요
// 여기서는 MOSFET Low-side 스위칭(게이트 HIGH=ON) 기준으로 작성
void setLeft(bool redOn, bool greenOn){
  digitalWrite(PIN_IL_R, redOn   ? HIGH : LOW);
  digitalWrite(PIN_IL_G, greenOn ? HIGH : LOW);
}
void setRight(bool redOn, bool greenOn){
  digitalWrite(PIN_IR_R, redOn   ? HIGH : LOW);
  digitalWrite(PIN_IR_G, greenOn ? HIGH : LOW);
}
void setExt(bool leftG, bool rightG){
  digitalWrite(PIN_EL_G, leftG  ? HIGH : LOW);
  digitalWrite(PIN_ER_G, rightG ? HIGH : LOW);
}

/* --------------- State / Policy --------------- */
void applyStateSideEffects(AlarmState s){
  if (s==STATE_DANGER) tone(PIN_BUZZER, 3000, 200);
}

void updateStateFromSensors(){
  AlarmState prev = g_state;

  if (g_state==STATE_NORMAL){
    if (g_mqAvg >= T_CAUTION_ON) g_state=STATE_CAUTION;
    if (g_mqAvg >= T_DANGER_ON)  g_state=STATE_DANGER;
  } else if (g_state==STATE_CAUTION){
    if (g_mqAvg < T_CAUTION_OFF) g_state=STATE_NORMAL;
    if (g_mqAvg >= T_DANGER_ON)  g_state=STATE_DANGER;
  } else { // DANGER
    if (g_mqAvg < T_DANGER_OFF)  g_state=STATE_CAUTION;
    if (g_mqAvg < T_CAUTION_OFF) g_state=STATE_NORMAL;
  }

  // 디지털 인터럽트가 오면 즉시 DANGER
  if (g_irqFlag){ g_irqFlag=false; g_state=STATE_DANGER; }

  if (prev!=g_state){
    applyStateSideEffects(g_state);
    Serial.print(F("[STATE] "));
    Serial.println(g_state==STATE_NORMAL?"NORMAL":g_state==STATE_CAUTION?"CAUTION":"DANGER");
  }
}

/* --------------- Tasks --------------- */
void readSensorsTask(){
  int raw = analogRead(PIN_FLAME_AO);
  g_mqRaw = raw;
  mqSum -= mqBuf[mqIdx];
  mqBuf[mqIdx] = raw;
  mqSum += raw;
  mqIdx = (mqIdx+1)%WIN;
  g_mqAvg = (int)(mqSum/WIN);
}

void updateLogicTask(){
  if (g_mode==MODE_AUTO){
    updateStateFromSensors();
  }
  // g_fireZone은 BT 시뮬레이션 전용 (AUTO에서도 실제 DO가 들어오면 DANGER)
}

void renderLedsTask(){
  // 기본 정책:
  // - 내부: 화재 구역과 "연결"된 쪽은 RED, 반대는 GREEN
  // - 외부: 화재가 왼쪽계열(Z1/Z2)이면 오른쪽 외부 GREEN, 오른쪽계열(Z3/Z4)이면 왼쪽 GREEN
  // - NORMAL: 꺼짐(또는 상태표시만 GREEN 약하게), CAUTION: 저속점멸, DANGER: 고정점등(외부는 빠른 점멸 가능)

  static bool slowBlink = false;
  static bool fastBlink = false;
  static uint32_t tSlow=0, tFast=0;
  if (timePassed(tSlow, 500)) slowBlink = !slowBlink;   // 0.5Hz
  if (timePassed(tFast, 150)) fastBlink = !fastBlink;   // ~3.3Hz

  bool leftIsFire  = (g_fireZone==1 || g_fireZone==2);
  bool rightIsFire = (g_fireZone==3 || g_fireZone==4);

  // 내부 좌/우 색 결정
  bool L_R=false, L_G=false, R_R=false, R_G=false;
  if (g_state==STATE_NORMAL){
    // 기본 소등
    L_R=L_G=R_R=R_G=false;
  } else if (g_state==STATE_CAUTION){
    // 주의: 저속 점멸, 구역 연결 규칙은 유지
    if (leftIsFire){ L_R = slowBlink; R_G = slowBlink; }
    else if (rightIsFire){ R_R = slowBlink; L_G = slowBlink; }
    else{
      // 화재 구역 미지정 시 양쪽 녹색 저속 점멸
      L_G = R_G = slowBlink;
    }
  } else { // DANGER
    if (leftIsFire){ L_R = true;  R_G = true; }
    else if (rightIsFire){ R_R = true; L_G = true; }
    else{
      // 구역 미지정 DANGER면 양쪽 적색 점등(정책)
      L_R = R_R = true;
    }
  }
  setLeft(L_R, L_G);
  setRight(R_R, R_G);

  // 외부 방향지시 (녹색만)
  bool extL=false, extR=false;
  if (g_state==STATE_NORMAL){
    extL=extR=false;
  } else if (g_state==STATE_CAUTION){
    if (leftIsFire)      { extR = slowBlink; }
    else if (rightIsFire){ extL = slowBlink; }
  } else { // DANGER
    if (leftIsFire)      { extR = fastBlink; }  // 반대방향 빠른 점멸
    else if (rightIsFire){ extL = fastBlink; }
  }
  setExt(extL, extR);
}

void beepEngineTask(){
  // 상태별 패턴 포인터 선택
  const BeepStep* target =
    (g_state==STATE_NORMAL)? PATTERN_NORMAL :
    (g_state==STATE_CAUTION)?PATTERN_CAUTION : PATTERN_DANGER;

  if (target != g_beepPtr){
    g_beepPtr = target; g_beepIdx=0; g_beepOn=false; g_beepTick=millis();
  }

  const BeepStep step = g_beepPtr[g_beepIdx];
  if (step.freq==0 && step.dur==0 && step.gap==0){ noTone(PIN_BUZZER); return; }

  uint32_t now = millis();
  if (!g_beepOn){
    if (now - g_beepTick >= step.gap){
      if (step.freq>0 && step.dur>0) tone(PIN_BUZZER, step.freq, step.dur);
      g_beepOn = true; g_beepTick = now;
    }
  } else {
    if (now - g_beepTick >= step.dur){
      g_beepOn = false; g_beepIdx++;
      // 패턴 반복
      const BeepStep next = g_beepPtr[g_beepIdx];
      if (next.freq==0 && next.dur==0 && next.gap==0) g_beepIdx=0;
      g_beepTick = now;
    }
  }
}

void logTask(){
  Serial.print(F("[MQ raw/avg] "));
  Serial.print(g_mqRaw); Serial.print('/'); Serial.print(g_mqAvg);
  Serial.print(F("  [state] "));
  Serial.print(g_state==STATE_NORMAL?"NORMAL":g_state==STATE_CAUTION?"CAUTION":"DANGER");
  Serial.print(F("  [mode] "));
  Serial.print(g_mode==MODE_AUTO?"AUTO":"MANUAL");
  Serial.print(F("  [fireZone] Z")); Serial.println(g_fireZone);
}

/* --------------- Bluetooth parser (Serial1) --------------- */
void handleBtLine(const String &line){
  String s = line; s.trim();
  s.toUpperCase();

  if (s.startsWith("ALARM ")){
    // ALARM Z1..Z4
    int z = s.lastChar() - '0';
    if (z>=1 && z<=4){
      g_fireZone = z;
      g_state = (g_mode==MODE_MANUAL)? STATE_DANGER : g_state; // MANUAL이면 즉시 위험 처리
      Serial1.print(F("OK ALARM Z")); Serial1.println(z);
      Serial.print(F("OK ALARM Z")); Serial.println(z);
      return;
    }
  }
  if (s=="CLEAR"){
    g_fireZone = 0;
    if (g_mode==MODE_MANUAL) g_state = STATE_NORMAL;
    Serial1.println(F("OK CLEAR")); Serial.println(F("OK CLEAR"));
    return;
  }
  if (s=="MODE AUTO"){
    g_mode = MODE_AUTO;  Serial1.println(F("OK MODE AUTO"));  Serial.println(F("OK MODE AUTO"));  return;
  }
  if (s=="MODE MANUAL"){
    g_mode = MODE_MANUAL; Serial1.println(F("OK MODE MANUAL")); Serial.println(F("OK MODE MANUAL")); return;
  }
  if (s=="VERBOSE ON"){ /* 필요 시 로그레벨 변수 추가 */ Serial1.println(F("OK VERBOSE ON")); return; }
  if (s=="VERBOSE OFF"){ /* 필요 시 로그레벨 변수 추가 */ Serial1.println(F("OK VERBOSE OFF")); return; }

  Serial1.println(F("ERR UNKNOWN CMD"));
}

/* --------------- setup / loop --------------- */
void setup(){
  pinMode(PIN_FLAME_DO, INPUT_PULLUP); // 센서 모듈에 따라 Active Low/High 확인
  attachInterrupt(digitalPinToInterrupt(PIN_FLAME_DO), isrDO, FALLING);

  pinMode(PIN_IL_G, OUTPUT);
  pinMode(PIN_IL_R, OUTPUT);
  pinMode(PIN_IR_G, OUTPUT);
  pinMode(PIN_IR_R, OUTPUT);
  pinMode(PIN_EL_G, OUTPUT);
  pinMode(PIN_ER_G, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  // 초기 소등
  setLeft(false,false);
  setRight(false,false);
  setExt(false,false);
  noTone(PIN_BUZZER);

  Serial.begin(115200);
  Serial1.begin(9600); // HC-06 기본 9600
  while(!Serial){;}

  Serial.println(F("System Initialized."));
}

void loop(){
  // BT 수신(개행 기준)
  while (Serial1.available()){
    char c = Serial1.read();
    if (c=='\n' || c=='\r'){
      if (btLine.length()>0){ handleBtLine(btLine); btLine=""; }
    } else btLine += c;
  }

  if (timePassed(tRead,  INTERVAL_READ_SENSORS)) readSensorsTask();
  if (timePassed(tLogic, INTERVAL_UPDATE_LOGIC)) updateLogicTask();
  if (timePassed(tLED,   INTERVAL_RENDER_LEDS )) renderLedsTask();
  if (timePassed(tBeep,  INTERVAL_BEEP_ENGINE))  beepEngineTask();
  if (timePassed(tLog,   INTERVAL_LOG_OUTPUT ))  logTask();
}
