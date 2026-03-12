  // ===================================================
  // 
  //        計算処理と記録処理（階段昇降時開始）
  //
  // ===================================================

#include <EEPROM.h>
#include <QuickStats.h>
QuickStats stats; 

#define D 500

#define FLAG_PUMP 0x01
#define FLAG_YU   0x02

const int mainButtonPin = 49;   // メインボタン
const int relay1Pin     = 53;   // PUMP 用リレー
const int relay2Pin     = 51;   // YUHAFO 用リレー
const int valve1Pin     = 47;   // PUMP 用バルブ
const int valve2Pin     = 45;   // YUHAFO 用バルブ
const int ledPin        = 31;   // 計算値反映用LED

const int sensorPin = A0; // sensor
unsigned long lastSensorTime = 0;
const unsigned long sensorInterval = 200;
bool sensorwatching = false;


unsigned long yuAfterTime = 0;
unsigned long yuMaxRunTime = 15000;  // 15秒間で動作制限

float P = 0.0;
float P_current =0.0;
float P_error = 0.0;
float dP = 0.0;

//EEPROM
struct Log {
  uint16_t time_ms;
  uint16_t pressure;
  byte flags;
  uint8_t climb;
  byte reserved;
};

const unsigned long LOG_INTERVAL = 200;   // ms
const unsigned long MEASURE_TIME = 60000; // EEPROM記録時間6秒

unsigned long startTime;
unsigned long lastLogTime;
int addr = 0;
bool measuring = false;
int correctionforPSI = 0;
bool secondPump = false;
bool correctPSI = false;
bool getoffstart = false;


//計算用
const int win = 5;
const float STD_TH = 0.01; //決めた閾値
const float SLP_TH = 0.08; //決めた閾値

int dataIndex=0;
unsigned long lastTime = 0;
const unsigned long interval = 200;

float sampleslp[win];
float SLP;
float samplestd[win];
float STD;
float P_prev;              // 直前の圧力
bool hasprev = false;      // 最初の1点判定

bool statusclimb = false;

// ボタン用
int lastButton = HIGH;
int pressCount = 0;  // 1 or 2 (1=PUMP, 2=YUHAFO)
bool motorActive = false;
bool firstDetected = false;
int correction = 0;

// 状態機械用フラグ/ステップ/タイマー
int stopcount = 0;
bool pumpActive = false;
bool systemoff = false;
int pumpStep = 0;
unsigned long pumpStepTime = 0;

bool yuActive = false;
int yuStep = 0;
unsigned long yuStepTime = 0;

bool offActive = false;
int offStep = 0;
int offStep2 = 0;
unsigned long offStepTime = 0;
int offTargetRelay = -1;
int offTargetValve = -1;
int offTimeParam = 0;

// ---------- sensor----------

int idx = 0;
const int numreadings = 10;
float readings[numreadings];   // センサー値バッファ

void sensor() {

  int raw = analogRead(sensorPin);
  float voltage = raw * (5.0 / 1023.0);
  float P_now = ((voltage - 0.25)/4.5)*15.0;
  if (P_now < 0) P_now = 0;

  // 新しい値をバッファに入れる
  readings[idx] = P_now;
  idx = (idx + 1) % numreadings;

  // 中央値を計算
  float P_filtered = stats.median(readings, numreadings);
  P = P_filtered;

  Serial.print("Pressure (median): ");
  Serial.println(P);
}

// ---------- 計算用　----------

float calcA(float p_now, float p_prev) {
  return abs(p_now - p_prev) / 0.2;
}

float calcB(float *data, int n){
  float m = data[0];
  for(int i=0; i<n; i++){
    m = max(m, data[i]);
  }
  return m;
}

float calcC(float *data, int n){
  float sum = 0;
  for(int i=0; i<n; i++) sum +=data[i];
  float ave = sum/n;

  float var = 0;
  for(int i=0; i<n; i++){
    float d = data[i] - ave;
    var +=d*d;
  }
  var /= n;
  return sqrt(var);
}

// ---------- 非ブロッキングのPUMP ONシーケンス ----------
void startPumpOn() {
  // stop other sequences
  sensorwatching = false;
  yuActive = false;
  yuStep = 0;
  offActive = false;

  pumpActive = true;
  pumpStep = 1;
  pumpStepTime = millis();
  //Serial.println("startPumpOn");
}

bool pumpOnStep() {
  // PUMP sequence: steps 1-5 (originally: 1-2 HIGH, 3-4 LOW, 5 valve HIGH)
  unsigned long now = millis();
  switch(pumpStep) {
    case 1:
    case 3:
      // HIGH for relay
      digitalWrite(relay1Pin, HIGH);
      if (now - pumpStepTime >= 500) {
        pumpStepTime = now;
        pumpStep++;
        //Serial.print("pump step -> "); Serial.println(pumpStep);
      }
      break;
    case 2:
    case 4:
      // LOW for relay
      digitalWrite(relay1Pin, LOW);
      if (now - pumpStepTime >= 500) {
        pumpStepTime = now;
        pumpStep++;
        //Serial.print("pump step -> "); Serial.println(pumpStep);
      }
      break;
    case 5:
      // valve ON (valve HIGH per original)
      digitalWrite(valve1Pin, HIGH);
      //Serial.println("pump valve on (completed)");
      // keep relay low (or leave as is). Mark finished
      P_current = P;
      systemoff = true;
      return true;
  }
  return false;
}

// ---------- 非ブロッキングのYUHAFO ONシーケンス ----------
void startYuOn() {
  // stop other sequences
  pumpActive = false;
  pumpStep = 0;
  offActive = false;

  yuActive = true;
  yuStep = 1;
  yuStepTime = millis();
  yuAfterTime = millis();    // YUHAFO開始時刻を記録

  
  //Serial.println("startYuOn");
}

bool yuOnStep() {
  // YUHAFO sequence: case1: RHIGH for 3000ms, then pattern of LOW/HIGH with 500ms intervals, final valve HIGH
  unsigned long now = millis();
  switch(yuStep) {
    case 1:
      digitalWrite(relay2Pin, HIGH);
      if (now - yuStepTime >= 3000) {
        yuStepTime = now;
        yuStep++;
        //Serial.println("yu step -> 2");
      }
      break;
    case 2: case 4: case 6:
      digitalWrite(relay2Pin, LOW);
      if (now - yuStepTime >= 500) {
        yuStepTime = now;
        yuStep++;
        //Serial.print("yu step -> "); Serial.println(yuStep);
      }
      break;
    case 3: case 5:
      digitalWrite(relay2Pin, HIGH);
      if (now - yuStepTime >= 500) {
        yuStepTime = now;
        yuStep++;
        //Serial.print("yu step -> "); Serial.println(yuStep);
      }
      break;
    case 7:
      digitalWrite(valve2Pin, HIGH); // valve OPEN
      //Serial.println("yu valve on (completed)");
      //yuActive = false;
      
      return true;
  }
  return false;
}

// ---------- 非ブロッキングのOFFシーケンス ----------
void startOffSequence(int relayPin, int valvePin, int TIME) {
  // stop ON sequences
  pumpActive = false;
  yuActive = false;

  offActive = true;
  
  if(pressCount == 2){
    offStep = 0; 
    offStep2 = 1;
  }else{
    offStep = 1;
    offStep2 = 0;
  }
  offStepTime = millis();
  offTargetRelay = relayPin;
  offTargetValve = valvePin;
  offTimeParam = TIME;
  //Serial.print("startOffSequence relay="); Serial.print(relayPin);
  //Serial.print(" valve="); Serial.print(valvePin);
  //Serial.print(" TIME="); Serial.println(TIME);
}

bool offStepMachine() {
  if (!offActive) return false;
  unsigned long now = millis();
  if(offStep2 == 1 || offStep2 == 2){
    switch(offStep2){
     case 1:
      digitalWrite(offTargetRelay, HIGH);
      if (now - offStepTime >= 500) {
        offStepTime = now;
        offStep2++;
      }
      break;

      case 2:
      digitalWrite(offTargetRelay, LOW);
      if (now - offStepTime >= 500) {
        offStepTime = now;
        offStep2++;
        offStep = 1;
      }
      break;
    }
    return false; 
  }

  switch(offStep) {
    case 1:
      // relay HIGH for TIME
      digitalWrite(offTargetRelay, HIGH);
      if (now - offStepTime >= (unsigned long)offTimeParam) {
        offStepTime = now;
        offStep++;
        //Serial.println("off step -> 2");
      }
      break;
    case 2:
      // relay LOW, valve LOW, wait 500 then finish
      digitalWrite(offTargetRelay, LOW);
      digitalWrite(offTargetValve, LOW);
      if (now - offStepTime >= 500) {
        //Serial.println("off completed");
        offActive = false;
        return true;
      }
      break;
  }
  return false;
}

// ---------- ボタン／シリアルイベント検出 ----------
int ButtonPressed() {
  bool current = digitalRead(mainButtonPin);
  int event = 0;
  // 離した瞬間 = LOW -> HIGH
  if (lastButton == LOW && current == HIGH) {
    event = 1;
  }
  lastButton = current;
  return event;
}

//---------------motor settings---------------------
// === モーター1のピン設定 === 
const int IN2_1 = 11;
const int PWM_PIN_1 = 10;
const int ENCODER_A1 = 2;
const int ENCODER_B1 = 3;

// === モーター2のピン設定 ===
const int IN2_2 = 5;
const int PWM_PIN_2 = 8;
const int ENCODER_A2 = 19;
const int ENCODER_B2 = 18;

// === 定数・パラメータ ===
const float torqueConstant = 0.0234;
const float supplyVoltage = 24.0;
const float targetSpeed = 1.5; // [rad/s]

const float Kp1 = 0.07, Ki1 = 0.0;   //kp=0.1
const float Kp2 = 0.07,  Ki2 = 0.0;

// === 状態変数 ===
volatile int encoderCount1 = 0, encoderCount2 = 0;
float currentSpeed1 = 0.0, currentSpeed2 = 0.0;
float integral1 = 0.0, integral2 = 0.0 ;
unsigned long previousTime = 0;
double times = 0.0;

// === 割り込み処理 ===
void handleEncoderA1() {
  bool a = digitalRead(ENCODER_A1);
  bool b = digitalRead(ENCODER_B1);
  encoderCount1 += (a == b) ? 1 : -1;
}

void handleEncoderA2() {
  bool a = digitalRead(ENCODER_A2);
  bool b = digitalRead(ENCODER_B2);
  encoderCount2 += (a == b) ? 1 : -1;
}

byte lastSent = 0;


void idle(){
  byte now;
  if(motorActive) now = 2;
  else            now = 1;

  if(now != lastSent){
    Serial3.write(now);
    lastSent = now;
  }
}

// ---------- EEPROM settings ----------

void startMeasure() {
  addr = 0;
  startTime = millis();
  lastLogTime = startTime;
  measuring = true;
}

void logData(float psi) {
  if (addr + sizeof(Log) > EEPROM.length()) return;

  Log log;
  log.time_ms  = (millis() - startTime) / 10;   // 10ms単位
  log.pressure = (uint16_t)(psi * 100 + 0.5);
  log.flags    = (pumpActive ? FLAG_PUMP : 0)
               | (yuActive   ? FLAG_YU   : 0);
  log.climb = statusclimb ? 1 : 0;
  log.reserved = 0;

  // update()で寿命対策
  byte* p = (byte*)&log;
  for (int i = 0; i < sizeof(Log); i++) {
    EEPROM.update(addr + i, p[i]);
  }

  addr += sizeof(Log);
}




  // ==============================
  // 
  //        セットアップ
  //
  // ==============================
void setup() {
  pinMode(mainButtonPin, INPUT_PULLUP);
  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
  pinMode(valve1Pin, OUTPUT);
  pinMode(valve2Pin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // air_system 初期状態
  digitalWrite(relay1Pin, LOW);
  digitalWrite(relay2Pin, LOW);
  digitalWrite(valve1Pin, HIGH);
  digitalWrite(valve2Pin, HIGH);

  Serial.begin(9600);
  Serial3.begin(9600); //TX03
  //Serial.println("System ready.");


  //motor settings
  // モータ1
  pinMode(IN2_1, OUTPUT);
  pinMode(PWM_PIN_1, OUTPUT);
  pinMode(ENCODER_A1, INPUT);
  pinMode(ENCODER_B1, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), handleEncoderA1, CHANGE);
 // attachInterrupt(digitalPinToInterrupt(ENCODER_B1), handleEncoderB1, CHANGE);  //追加


  // モータ2
  pinMode(IN2_2, OUTPUT);
  pinMode(PWM_PIN_2, OUTPUT);
  pinMode(ENCODER_A2, INPUT);
  pinMode(ENCODER_B2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), handleEncoderA2, CHANGE);
 // attachInterrupt(digitalPinToInterrupt(ENCODER_B2), handleEncoderB2, CHANGE);


  // 初期出力ゼロ
  analogWrite(PWM_PIN_1, 0); digitalWrite(IN2_1, LOW);
  analogWrite(PWM_PIN_2, 0); digitalWrite(IN2_2, LOW);

}

  // ==============================
  // 
  //        メインループ
  //
  // ==============================

void loop() {

  idle();

//-----------motor move---------------------------------------------------
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= 5) {    //10msから20ms
    unsigned long deltaTime = currentTime - previousTime;
    previousTime = currentTime;
    times += 0.005;

    float dt = deltaTime / 1000.0f;   //1000.0
    const int pulsesPerRevolution1 = 24900;   //19000, 2490
    const int pulsesPerRevolution2 = 24900;   //19000, 2490
    motorActive = (!offActive && !yuActive && !pumpActive && getoffstart);
    
    if(motorActive == true){

      if (!firstDetected && motorActive) {
        //startMeasure();
        firstDetected = true;
      }
      
      // === モータ1（回転方向を反転）===
      int count1 = -1*encoderCount1;
      encoderCount1 = 0;
      currentSpeed1 = (count1 / (float)pulsesPerRevolution1) * 2.0 * PI / dt;
      float error1 = targetSpeed - currentSpeed1;
      integral1 += error1 * dt;
    
      float tau1 = (Kp1 * error1 + Ki1 * integral1);  // 反転
      float pwm1 = constrain((fabs(tau1) / torqueConstant) * 2.32 / supplyVoltage * 255, 0, 255);
      digitalWrite(IN2_1, (tau1 < 0) ? LOW : HIGH);
      analogWrite(PWM_PIN_1, pwm1);

      // === モータ2（そのまま）===
      int count2 = encoderCount2;
      encoderCount2 = 0;
      currentSpeed2 = (count2 / (float)pulsesPerRevolution2) * 2.0 * PI / dt;
      float error2 = targetSpeed - currentSpeed2;
      integral2 += error2 * dt;
      float tau2 = (Kp2 * error2 + Ki2 * integral2);
      float pwm2 = constrain((fabs(tau2) / torqueConstant) * 2.32 / supplyVoltage * 255, 0, 255);
      digitalWrite(IN2_2, (tau2 < 0) ? LOW : HIGH);
      analogWrite(PWM_PIN_2, pwm2);
    }else{
      analogWrite(PWM_PIN_1, 0);
      analogWrite(PWM_PIN_2, 0);

      digitalWrite(IN2_1, LOW);
      digitalWrite(IN2_2, LOW);

      integral1 = 0.0;
      integral2 = 0.0;
    }
  }
//-----------air system move--------------------------------------------------- 
  unsigned long now = millis();
  // センサー周期処理
  if (now - lastSensorTime >= sensorInterval) {
    sensor();
    lastSensorTime = now;

    //---calculate--
    static int idx =0;
    static int filled;

    //傾き計算
    if (hasprev){
      sampleslp[idx] = calcA( P, P_prev );
    }
    P_prev = P;
    hasprev = true;

    //5つセンサ値のサンプルをとる
    samplestd[idx] = P;

    //サンプリング周期作成
    idx = (idx + 1) % win;
    if (filled < win) filled++;
    
    if (filled == win) {
      SLP = calcB(sampleslp, win);   // 傾き最大
      STD = calcC(samplestd, win);   // 標準偏差

      if (SLP < SLP_TH && STD < STD_TH) {
        digitalWrite(ledPin, LOW); //平地
        statusclimb = true;
      }else{
        digitalWrite(ledPin, HIGH);
        statusclimb = false;
      }
    }else{
      return;
    }
    dataIndex++;

    //Correction
    if (correctPSI == true ){
      dP = abs(P_error - P);
      //Serial.println (P_error);
      //Serial.println (dP);
      if(dP< 0.01){
        correctionforPSI ++;
        //Serial.println (correction);
      }else{
        //Serial.println ("not reached");
      }
    }
    P_error = P;
    
  }
  

  // シーケンスを継続する（非ブロッキングで毎ループ進める）
  if (pumpActive) pumpOnStep();
  if (yuActive)   yuOnStep();
  if (offActive)  offStepMachine();

  
  if (ButtonPressed() && offActive == false && yuActive == false && pumpActive == false) {
    pressCount++;
    //Serial.print("Pressed -> pressCount = "); Serial.println(pressCount);
    if (pressCount > 2) pressCount = 1;

    if (pressCount == 1)  startPumpOn(); 
    else if (pressCount == 2) startYuOn();
  }

// ---------- 安全機能：YUHAFO 連続稼働時間制限 ----------
  if (yuActive) {
    if( P > 3.50 ){
      stopcount ++;
      Serial.println(stopcount);
    }
    if (millis() - yuAfterTime > yuMaxRunTime || stopcount > 2) {
      //Serial.println("Safety Stop: YUHAFO exceeded max run time.");

      // 強制 OFF シーケンス
      startOffSequence(relay2Pin, valve2Pin, 3000);

      // 状態をリセット
      yuActive = false;
      stopcount = 0;
      sensorwatching = true;
      secondPump = true; //2度目の空気抜き感知フラグ＝＞記録開始用
    }
  } 

  if(sensorwatching == true && P < 3.00 && yuActive == false && offActive == false){
    startYuOn();
  }

// ---------- 安全機能：PUMP 連続稼働時間制限 ----------

  if (pumpActive && systemoff == true && P < 0.50) {
      //Serial.println("Safety Stop: PUMP undered minimum PSI.");

      // 強制 OFF シーケンス
      startOffSequence(relay1Pin, valve1Pin, 500);

      if( P <= 0.60 && P >= 0.50 && (now - lastSensorTime) >= sensorInterval){
        correction ++;
        digitalWrite(valve1Pin, LOW);
      }else{
        digitalWrite(valve1Pin, HIGH);
      }

      if(correctPSI == true && correctionforPSI >=10 && secondPump){
        // 状態をリセット
        pumpActive = false;
        systemoff = false;
        correctPSI = false;
        correctionforPSI = 0;

        P_error = P; 
      }
      if(secondPump && !measuring){
        startMeasure();
        getoffstart = true;
      }
  }

//-----------eeprom---------------------------------------------------
unsigned long nowE = millis();

  if (nowE - startTime <= MEASURE_TIME && measuring == true) {
    if (nowE - lastLogTime >= LOG_INTERVAL) {
      lastLogTime = nowE;
      float psi = P;
      logData(psi);
    }
  }else{
    measuring = false;
  }

}
