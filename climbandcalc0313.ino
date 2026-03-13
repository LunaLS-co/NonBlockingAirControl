  // ===================================================
  // 
  //        Data recorde and calculate
  //
  // ===================================================

#include <EEPROM.h>
#include <QuickStats.h>
QuickStats stats; 

#define D 500

#define FLAG_PUMP 0x01
#define FLAG_YU   0x02

const int mainButtonPin = 49;   // Operation Button(For switch between stairclimb and flatdrive)
const int relay1Pin     = 53;   // For use air vent(product name: PUMP)
const int relay2Pin     = 51;   // For use air pump(product name: YUHAFO)
const int valve1Pin     = 47;   // Valve for air vent
const int valve2Pin     = 45;   // Valve for air pump
const int ledPin        = 31;   // LED for visualizing stair climbing condition

const int sensorPin = A0; // Pressure sensor
unsigned long lastSensorTime = 0;
const unsigned long sensorInterval = 200;
bool sensorwatching = false;


unsigned long yuAfterTime = 0;
unsigned long yuMaxRunTime = 15000;  // 15sec

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


//For data calculation
const int win = 5;
const float STD_TH = 0.01; //Threshold of standard deviation
const float SLP_TH = 0.08; //Threshold of slope

int dataIndex=0;
unsigned long lastTime = 0;
const unsigned long interval = 200;

float sampleslp[win];
float SLP;
float samplestd[win];
float STD;
float P_prev;              
bool hasprev = false;      // For recognize first point

bool statusclimb = false;

// For operating button
int lastButton = HIGH;
int pressCount = 0;  // 1 or 2 (1=PUMP, 2=YUHAFO)
bool motorActive = false;
bool firstDetected = false;
int correction = 0;

// Flag and timer's settings for air control machines
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

// ----------Pressure sensor(using QuickStats for simplification)----------

int idx = 0;
const int numreadings = 10;
float readings[numreadings];   // Sensor value buffer

void sensor() {

  int raw = analogRead(sensorPin);
  float voltage = raw * (5.0 / 1023.0);
  float P_now = ((voltage - 0.25)/4.5)*15.0;
  if (P_now < 0) P_now = 0;

  // Put new value into the buffer
  readings[idx] = P_now;
  idx = (idx + 1) % numreadings;

  // Calculate the median
  float P_filtered = stats.median(readings, numreadings);
  P = P_filtered;

  //Serial.print("Pressure (median): ");
  //Serial.println(P);
}

// ---------- Data calculation for road surface condition perception　----------

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

// ---------- Non loop blocking air vent sequence ----------
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
  // Air vent sequence: steps 1-5 (originally: 1-2 HIGH, 3-4 LOW, 5 valve HIGH)
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
      // keep relay low (or leave as is). Chenge flag to finished
      P_current = P;
      systemoff = true;
      return true;
  }
  return false;
}

// ---------- Non loop blocking air pump sequence ----------
void startYuOn() {
  // stop other sequences
  pumpActive = false;
  pumpStep = 0;
  offActive = false;

  yuActive = true;
  yuStep = 1;
  yuStepTime = millis();
  yuAfterTime = millis();    // Record the Yuhafo starting time

  
  //Serial.println("startYuOn");
}

bool yuOnStep() {
  // Air pump sequence: case1: RHIGH for 3000ms, then pattern of LOW/HIGH with 500ms intervals, final valve HIGH
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
      
      return true;
  }
  return false;
}

// ---------- Non loop blocking stop air machines seaquence ----------
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

// ---------- Operating button/Detect serial event ----------
int ButtonPressed() {
  bool current = digitalRead(mainButtonPin);
  int event = 0;
  // The moment release the button = LOW -> HIGH
  if (lastButton == LOW && current == HIGH) {
    event = 1;
  }
  lastButton = current;
  return event;
}

//---------------motor settings---------------------

//Omit

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
  log.time_ms  = (millis() - startTime) / 10;   // 10ms unit
  log.pressure = (uint16_t)(psi * 100 + 0.5);
  log.flags    = (pumpActive ? FLAG_PUMP : 0)
               | (yuActive   ? FLAG_YU   : 0);
  log.climb = statusclimb ? 1 : 0;
  log.reserved = 0;

  // Lifespan countermeasures in update()
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


  //motor settings --omit

  // 初期出力ゼロ
  analogWrite(PWM_PIN_1, 0); digitalWrite(IN2_1, LOW);
  analogWrite(PWM_PIN_2, 0); digitalWrite(IN2_2, LOW);

}

  // ============================================================================================
  // 
  //        Main loop (When switching road surface condition, insert the air control seaquences)
  //
  // ============================================================================================

void loop() {

//-----------Motor move---------------------------------------------------
  unsigned long currentTime = millis();
  if (currentTime - previousTime >= 5) {    //10ms to 20ms
    unsigned long deltaTime = currentTime - previousTime;
    previousTime = currentTime;
    times += 0.005;
    
    motorActive = (!offActive && !yuActive && !pumpActive && getoffstart);
    
    if(motorActive == true){

      if (!firstDetected && motorActive) {
        firstDetected = true;
      }
      //Motors moving process --omit
  }
//-----------Air system move--------------------------------------------------- 
  unsigned long now = millis();
  // Sensor cycle
  if (now - lastSensorTime >= sensorInterval) {
    sensor();
    lastSensorTime = now;

    //---For calculation--
    static int idx =0;
    static int filled;

    //Calculate the slope
    if (hasprev){
      sampleslp[idx] = calcA( P, P_prev );
    }
    P_prev = P;
    hasprev = true;

    //Take 5 sample value of the sensor
    samplestd[idx] = P;

    //Make the sampling time
    idx = (idx + 1) % win;
    if (filled < win) filled++;
    
    if (filled == win) {
      SLP = calcB(sampleslp, win);   // Maximum slope value
      STD = calcC(samplestd, win);   // Standard deviation value

      if (SLP < SLP_TH && STD < STD_TH) {
        digitalWrite(ledPin, LOW); //Flat road
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
  

  // Continue seaquences(Proceed every loop by non blocking)
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

// ---------- Safety features：Air vent Continuous operating time limit ----------
  if (yuActive) {
    if( P > 3.50 ){
      stopcount ++;
      Serial.println(stopcount);
    }
    if (millis() - yuAfterTime > yuMaxRunTime || stopcount > 2) {
      //Serial.println("Safety Stop: YUHAFO exceeded max run time.");

      // Force quit
      startOffSequence(relay2Pin, valve2Pin, 3000);

      // Reset conditions
      yuActive = false;
      stopcount = 0;
      sensorwatching = true;
      secondPump = true; //2度目の空気抜き感知フラグ＝＞記録開始用
    }
  } 

  if(sensorwatching == true && P < 3.00 && yuActive == false && offActive == false){
    startYuOn();
  }

// ---------- Safety features：Air pump Continuous operating time limit ----------

  if (pumpActive && systemoff == true && P < 0.50) {
      //Serial.println("Safety Stop: PUMP undered minimum PSI.");

      // Force quit
      startOffSequence(relay1Pin, valve1Pin, 500);

      if( P <= 0.60 && P >= 0.50 && (now - lastSensorTime) >= sensorInterval){
        correction ++;
        digitalWrite(valve1Pin, LOW);
      }else{
        digitalWrite(valve1Pin, HIGH);
      }

      if(correctPSI == true && correctionforPSI >=10 && secondPump){
        // Reset conditions
        pumpActive = false;
        systemoff = false;
        correctPSI = false;
        correctionforPSI = 0;

        P_error = P; 
      }
      if(secondPump && !measuring){
        startMeasure(); //Start recording in second air vent seaquence (Start the stairclimbing when performing the second air vent)
        getoffstart = true;
      }
  }

//-----------Data record by eeprom---------------------------------------------------
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
