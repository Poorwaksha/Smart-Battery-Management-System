#define BLYNK_TEMPLATE_ID "TMPL3Zz9wsv1f"
#define BLYNK_TEMPLATE_NAME "BMS"
#define BLYNK_AUTH_TOKEN "VcFpeDWIiqotoGclsjfHoTpuDW1gqgUg"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

char ssid[] = "Wokwi-GUEST";
char pass[] = "";

LiquidCrystal_I2C lcd(0x27, 16, 4);

// ---------------- Pins ----------------
const int cellPins[4] = {34, 35, 32, 33};
const int RELAY_PIN = 25;
const int BUZZER_PIN = 26;
const int FAULT_LED = 27;

// Status LEDs
const int GREEN_LED  = 14; // Healthy
const int YELLOW_LED = 12; // Warning
const int RED_LED    = 13; // Alert

// ---------------- Data ----------------
float cellV[4];
float prevCellV[4] = {0,0,0,0};
float packVoltage;
float packAvg;
float imbalancePct;
float soc;
int weakestCell, strongestCell;

// Thresholds
const float V_MIN = 2.8;
const float V_MAX = 4.2;
const float V_CRITICAL_LOW = 2.6;
const float V_CRITICAL_HIGH = 4.3;
const float IMBALANCE_MINOR = 5.0;   // %
const float IMBALANCE_CRITICAL = 15.0;

// Battery health states
enum HealthState { HEALTHY, MINOR_IMBALANCE, CRITICAL_IMBALANCE, PACK_FAILURE };
HealthState healthState = HEALTHY;

// Runtime mode
enum RuntimeMode { NORMAL, DEGRADED, FAILSAFE, SHUTDOWN };
RuntimeMode runtimeMode = NORMAL;

// Relay / fault state
bool relayOn = true;
bool buzzerActive = false;
unsigned long lastRelayChange = 0;
const unsigned long RELAY_CHATTER_GUARD = 3000; // ms

// Fault flags
bool faultSensor = false;
bool faultOvervoltage = false;
bool faultWeakCell = false;
bool faultRapidFluctuation = false;
bool faultFrozenADC = false;

// Fault log structure
struct FaultEntry {
  unsigned long timestamp;
  String description;
};
FaultEntry faultLog[10];
int faultLogIndex = 0;
int faultLogCount = 0;

// ---------------- Timing (non-blocking) ----------------
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_INTERVAL = 200;

unsigned long lastLCDUpdate = 0;
const unsigned long LCD_INTERVAL = 2000; // screen rotation
int lcdScreen = 0;
const int NUM_SCREENS = 4;

unsigned long lastBuzzerToggle = 0;
bool buzzerState = false;

unsigned long lastBlynkSend = 0;
const unsigned long BLYNK_MIN_INTERVAL = 500; // throttle

unsigned long lastFrozenCheck = 0;
float lastRawSum = -1;
unsigned long frozenSince = 0;

// WiFi reconnect
unsigned long lastWifiCheck = 0;
bool wifiWasConnected = false;
bool pendingTelemetrySync = false;
bool wifiConnectingMsgPrinted = false;

// ---------------- Helper: voltage conversion ----------------
float readCellVoltage(int pin) {
  int raw = analogRead(pin);
  float v = 2.5 + (raw / 4095.0) * (4.3 - 2.5);
  return v;
}

// ---------------- Fault logging ----------------
void logFault(String desc) {
  faultLog[faultLogIndex].timestamp = millis();
  faultLog[faultLogIndex].description = desc;
  faultLogIndex = (faultLogIndex + 1) % 10;
  if (faultLogCount < 10) faultLogCount++;
  Serial.println("[FAULT] " + desc);
}

// ---------------- Sensor reading & analytics ----------------
void readSensorsAndAnalyze() {
  float sum = 0;
  for (int i = 0; i < 4; i++) {
    int raw = analogRead(cellPins[i]);

    if (raw == 0 || raw >= 4095) {
      if (!faultSensor) {
        faultSensor = true;
        logFault("Sensor anomaly on Cell " + String(i+1) + " (raw=" + String(raw) + ")");
      }
    }

    cellV[i] = 2.5 + (raw / 4095.0) * (4.3 - 2.5);

    if (abs(cellV[i] - prevCellV[i]) > 0.3) {
      faultRapidFluctuation = true;
      logFault("Rapid fluctuation Cell " + String(i+1));
    } else {
      faultRapidFluctuation = false;
    }
    prevCellV[i] = cellV[i];
    sum += cellV[i];
  }

  if (abs(sum - lastRawSum) < 0.0005) {
    if (frozenSince == 0) frozenSince = millis();
    if (millis() - frozenSince > 5000) {
      if (!faultFrozenADC) {
        faultFrozenADC = true;
        logFault("Frozen ADC detected - identical readings >5s");
      }
    }
  } else {
    frozenSince = 0;
    faultFrozenADC = false;
  }
  lastRawSum = sum;

  packAvg = sum / 4.0;
  packVoltage = sum;
  soc = ((packAvg - V_MIN) / (V_MAX - V_MIN)) * 100.0;

  if (soc < 0) soc = 0;
  if (soc > 100) soc = 100;
  Serial.println("---------------");
  Serial.print("Pack Voltage: ");
  Serial.print(packVoltage);
  Serial.println(" V");
  Serial.print("SOC: ");
  Serial.print(soc);
  Serial.println("%");
  Serial.print("Imbalance: ");
  Serial.print(imbalancePct);
  Serial.println("%");
  Serial.print("Weakest Cell: C");
  Serial.println(weakestCell + 1);
  Serial.print("Strongest Cell: C");
  Serial.println(strongestCell + 1);

  weakestCell = 0; strongestCell = 0;
  for (int i = 1; i < 4; i++) {
    if (cellV[i] < cellV[weakestCell]) weakestCell = i;
    if (cellV[i] > cellV[strongestCell]) strongestCell = i;
  }

  imbalancePct = ((cellV[strongestCell] - cellV[weakestCell]) / packAvg) * 100.0;

  faultOvervoltage = false;
  for (int i = 0; i < 4; i++) {
    if (cellV[i] >= V_CRITICAL_HIGH) faultOvervoltage = true;
  }

  faultWeakCell = (cellV[weakestCell] <= V_CRITICAL_LOW);

  if (faultOvervoltage || faultWeakCell || imbalancePct >= IMBALANCE_CRITICAL) {
    healthState = (faultOvervoltage || faultWeakCell) ? PACK_FAILURE : CRITICAL_IMBALANCE;
  } else if (imbalancePct >= IMBALANCE_MINOR) {
    healthState = MINOR_IMBALANCE;
  } else {
    healthState = HEALTHY;
  }
}

// ---------------- Protection logic (event-driven, non-blocking) ----------------
void runProtectionLogic() {
  bool criticalCondition = (healthState == PACK_FAILURE) || (healthState == CRITICAL_IMBALANCE)
                            || faultOvervoltage || faultWeakCell;

  if (criticalCondition && relayOn) {
    if (millis() - lastRelayChange > RELAY_CHATTER_GUARD) {
      relayOn = false;
      digitalWrite(RELAY_PIN, LOW);
      lastRelayChange = millis();
      logFault("Relay OPENED due to critical condition");
    }
  } else if (!criticalCondition && !relayOn) {
    if (millis() - lastRelayChange > RELAY_CHATTER_GUARD) {
      relayOn = true;
      digitalWrite(RELAY_PIN, HIGH);
      lastRelayChange = millis();
      logFault("Relay CLOSED - condition recovered");
    }
  }

  buzzerActive = criticalCondition || (healthState == MINOR_IMBALANCE);
  if (buzzerActive) {
    unsigned long interval = criticalCondition ? 200 : 800;
    if (millis() - lastBuzzerToggle >= interval) {
      buzzerState = !buzzerState;
      // ✅ ONLY CHANGE: ledcWriteTone instead of digitalWrite (ESP32 buzzer needs PWM)
      if (buzzerState) {
        ledcWriteTone(BUZZER_PIN, criticalCondition ? 2000 : 1000);
      } else {
        ledcWriteTone(BUZZER_PIN, 0);
      }
      lastBuzzerToggle = millis();
    }
  } else {
    ledcWriteTone(BUZZER_PIN, 0);  // ✅ ONLY CHANGE: was digitalWrite(BUZZER_PIN, LOW)
    buzzerState = false;
  }

  digitalWrite(FAULT_LED, criticalCondition ? HIGH : LOW);
}

// ---------------- Runtime mode management ----------------
void updateRuntimeMode() {
  int activeFaults = faultSensor + faultOvervoltage + faultWeakCell + faultRapidFluctuation + faultFrozenADC;

  if (healthState == PACK_FAILURE && activeFaults >= 2) {
    runtimeMode = SHUTDOWN;
  } else if (healthState == PACK_FAILURE || faultOvervoltage || faultWeakCell) {
    runtimeMode = FAILSAFE;
  } else if (activeFaults > 0 || healthState == CRITICAL_IMBALANCE) {
    runtimeMode = DEGRADED;
  } else {
    runtimeMode = NORMAL;
  }

  if (runtimeMode == NORMAL) {
    faultSensor = false;
    faultFrozenADC = false;
  }
}

// ---------------- Status LED indication ----------------
void updateStatusLEDs() {
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RED_LED, LOW);

  switch (healthState) {
    case HEALTHY:
      digitalWrite(GREEN_LED, HIGH);
      break;
    case MINOR_IMBALANCE:
      digitalWrite(YELLOW_LED, HIGH);
      break;
    case CRITICAL_IMBALANCE:
    case PACK_FAILURE:
      digitalWrite(RED_LED, HIGH);
      break;
  }

  if (runtimeMode == FAILSAFE || runtimeMode == SHUTDOWN) {
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(YELLOW_LED, LOW);
    digitalWrite(RED_LED, HIGH);
  } else if (runtimeMode == DEGRADED && healthState == HEALTHY) {
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(YELLOW_LED, LOW);
    
  }
}

// ---------------- LCD HMI ----------------
void updateLCD() 
{
// ================= CRITICAL FAULT OVERRIDE =================
  if (runtimeMode == SHUTDOWN || runtimeMode == FAILSAFE) 
 {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("*** FAULT ***");
  lcd.setCursor(0,1);
  lcd.print("Mode:");
  lcd.print(runtimeMode == SHUTDOWN ? "SHUTDOWN" : "FAILSAFE");
  lcd.setCursor(0,2);
  lcd.print("Weak C");
  lcd.print(weakestCell+1);
  lcd.print(":");
  lcd.print(cellV[weakestCell],2);
  lcd.print("V");
  lcd.setCursor(0,3);
  lcd.print("Relay:OPEN");
  return;
 }
 lcd.clear();
 switch(lcdScreen)
 {  
 // ==================================================
 // SCREEN 1 : CELL VOLTAGES
 // ==================================================
  case 0:
 
   lcd.setCursor(0,0);
   lcd.print("C1:");
   lcd.print(cellV[0],2);
   lcd.print(" C2:");
   lcd.print(cellV[1],2);
   lcd.setCursor(0,1);
   lcd.print("C3:");
   lcd.print(cellV[2],2);
   lcd.print(" C4:");
   lcd.print(cellV[3],2);
   lcd.setCursor(0,2);
   lcd.print("Live Cell Data");
   lcd.setCursor(0,3);
   lcd.print("Screen 1/5");
   break;
 // ==================================================
 // SCREEN 2 : PACK DATA
 // ==================================================
  case 1:
 
   lcd.setCursor(0,0);
   lcd.print("Pack:");
   lcd.print(packVoltage,2);
   lcd.print("V");
   lcd.setCursor(0,1);
   lcd.print("Avg:");
   lcd.print(packAvg,2);
   lcd.print("V"); 
   lcd.setCursor(0,2);
   lcd.print("SOC:");
   lcd.print((int)soc);
   lcd.print("%");
   lcd.setCursor(0,3);
   lcd.print("Screen 2/5");
 
   break;
 // ==================================================
 // SCREEN 3 : ANALYTICS
 // ==================================================
  case 2:
   lcd.setCursor(0,0);
   lcd.print("Weak:C");
   lcd.print(weakestCell+1);
   lcd.print(" ");
   lcd.print(cellV[weakestCell],2);
   lcd.setCursor(0,1);
   lcd.print("Strong:C");
   lcd.print(strongestCell+1);
   lcd.print(" ");
   lcd.print(cellV[strongestCell],2);
   lcd.setCursor(0,2);
   lcd.print("Imb:");
   lcd.print(imbalancePct,1);
   lcd.print("%");
   lcd.setCursor(0,3);
   lcd.print("Screen 3/5");
 
   break;
 // ==================================================
 // SCREEN 4 : PROTECTION STATUS
 // ==================================================
  case 3:

   lcd.setCursor(0,0);
   lcd.print("Health:");
  switch(healthState) {

     case HEALTHY:
       lcd.print("HEALTHY");
       break;

     case MINOR_IMBALANCE:
       lcd.print("WARNING");
       break;
 
     case CRITICAL_IMBALANCE:
       lcd.print("CRITICAL");
       break;
 
     case PACK_FAILURE:
       lcd.print("FAILURE");
       break;
   }
 
   lcd.setCursor(0,1);
   lcd.print("Relay:");
   lcd.print(relayOn ? "ON" : "OFF");
   lcd.setCursor(0,2);
   lcd.print("Buzzer:");
   lcd.print(buzzerActive ? "ON" : "OFF");
   lcd.setCursor(0,3);
   lcd.print("Screen 4/5");
 
   break;
 // ==================================================
 // SCREEN 5 : DIAGNOSTICS
 // ==================================================
  case 4:
 
   lcd.setCursor(0,0);
   lcd.print("Mode:");
 
   switch(runtimeMode) {
 
     case NORMAL:
       lcd.print("NORMAL");
       break;

     case DEGRADED:
       lcd.print("DEGRADE");
       break;
 
    case FAILSAFE:
        lcd.print("FAILSAFE");
       break;
 
     case SHUTDOWN:
       lcd.print("SHUTDOWN");
       break;
   }
 
   lcd.setCursor(0,1);
   lcd.print("S:");
   lcd.print(faultSensor);
   lcd.print(" O:");
   lcd.print(faultOvervoltage);
   lcd.print(" W:");
   lcd.print(faultWeakCell);
   lcd.setCursor(0,2);
   lcd.print("F:");
   lcd.print(faultFrozenADC);
   lcd.print(" R:");
   lcd.print(faultRapidFluctuation);
   lcd.setCursor(0,3);
   lcd.print("Logs:");
   lcd.print(faultLogCount);
   break;

 }
}

// ---------------- Blynk telemetry (event-driven) ----------------
HealthState lastSentHealth = HEALTHY;
RuntimeMode lastSentMode = NORMAL;
bool lastSentRelay = true;

void sendTelemetryIfNeeded() {
  if (millis() - lastBlynkSend < BLYNK_MIN_INTERVAL) return;
  if (!Blynk.connected()) return;

  bool stateChanged = (healthState != lastSentHealth) || (runtimeMode != lastSentMode) || (relayOn != lastSentRelay);
  bool thresholdViolation = faultOvervoltage || faultWeakCell || (imbalancePct >= IMBALANCE_CRITICAL);

  if (stateChanged || thresholdViolation || pendingTelemetrySync) {
    Blynk.virtualWrite(V0, cellV[0]);
    Blynk.virtualWrite(V1, cellV[1]);
    Blynk.virtualWrite(V2, cellV[2]);
    Blynk.virtualWrite(V3, cellV[3]);
    Blynk.virtualWrite(V4, packAvg);
    Blynk.virtualWrite(V5, packVoltage);
    Blynk.virtualWrite(V6, soc);
    Blynk.virtualWrite(V7, imbalancePct);
    Blynk.virtualWrite(V8, (int)healthState);
    Blynk.virtualWrite(V9, (int)runtimeMode);
    Blynk.virtualWrite(V10, relayOn ? 1 : 0);
    Blynk.virtualWrite(V11, WiFi.RSSI());
    Blynk.virtualWrite(V12, weakestCell + 1);
    Blynk.virtualWrite(V13, strongestCell + 1);

    lastSentHealth = healthState;
    lastSentMode = runtimeMode;
    lastSentRelay = relayOn;
    lastBlynkSend = millis();
    pendingTelemetrySync = false;
  }
}

// ---------------- WiFi management ----------------
void manageWiFi() {
  if (millis() - lastWifiCheck < 1000) return;
  lastWifiCheck = millis();

  bool connected = (WiFi.status() == WL_CONNECTED);

  if (connected && !wifiWasConnected) {
    Serial.println("[WiFi] Connected successfully!");
    Serial.print("[WiFi] IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("[WiFi] Signal strength (RSSI): ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    pendingTelemetrySync = true;
    logFault("WiFi connected - syncing telemetry");
  }
  if (!connected && wifiWasConnected) {
    Serial.println("[WiFi] Connection lost!");
    logFault("WiFi connection lost");
  }
  wifiWasConnected = connected;

  if (!connected) {
    Serial.println("[WiFi] Attempting to reconnect...");
    Blynk.connect(500);
  }
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) pinMode(cellPins[i], INPUT);
  pinMode(RELAY_PIN, OUTPUT);
  // ✅ ONLY CHANGE: ledcAttach instead of pinMode for buzzer (ESP32 PWM setup)
  ledcAttach(BUZZER_PIN, 2000, 8);
  pinMode(FAULT_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Battery Intel Sys");
  lcd.setCursor(0,1);
  lcd.print("Booting...");

  Serial.println("[WiFi] Connecting to WiFi...");
  WiFi.begin(ssid, pass);

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("[WiFi] Connected successfully!");
    Serial.print("[WiFi] IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("[WiFi] Signal strength (RSSI): ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
    wifiWasConnected = true;
  } else {
    Serial.println("[WiFi] Initial connection failed, will retry in loop...");
    wifiWasConnected = false;
  }

  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect(3000);
}

// ---------------- Main loop (non-blocking) ----------------
void loop() {
  if (Blynk.connected()) Blynk.run();

  unsigned long now = millis();

  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;
    readSensorsAndAnalyze();
    runProtectionLogic();
    updateRuntimeMode();
    updateStatusLEDs();
    sendTelemetryIfNeeded();
  }

  if (now - lastLCDUpdate >= LCD_INTERVAL) {
    lastLCDUpdate = now;
    if (runtimeMode != FAILSAFE && runtimeMode != SHUTDOWN) {
      lcdScreen = (lcdScreen + 1) % NUM_SCREENS;
    }
    updateLCD();
  }

  manageWiFi();
}
