#include <WiFi.h>
#include <PubSubClient.h>   // Library Manager: "PubSubClient" by Nick O'Leary
#include "mbedtls/md.h"     // built into the ESP32 core (used to verify owner commands)

// ====================== EDIT THESE ======================
const char* WIFI_SSID     = "";
const char* WIFI_PASSWORD = "";

// Secret key of the ONE owner. Use a long random string (16+ characters).
// The same key is typed into the web app to take control.
// Leave it empty ("") and the motor can NOT be controlled remotely (spectator-only).
const char* OWNER_KEY     = "123456";
// ========================================================

// ---------- MQTT (public test broker, no login) ----------
// The web app (MotorStation_App.html) must use the same broker and topics.
const char*    MQTT_HOST        = "broker.emqx.io";
const uint16_t MQTT_PORT        = 1883;                              // plain TCP for the ESP32
const char*    TOPIC_TELEMETRY  = "motorstation/demo-k7q2/telemetry"; // ESP32 -> web app
const char*    TOPIC_STATUS     = "motorstation/demo-k7q2/status";    // "online" / "offline" (last will)
const char*    TOPIC_CMD        = "motorstation/demo-k7q2/cmd";       // web app (owner) -> ESP32
constexpr uint32_t PUBLISH_INTERVAL_MS = 2000;                       // 1 message every 2 s (values are averaged over this time)

// ---------- Owner control ----------
constexpr uint32_t OWNER_LEASE_MS      = 12000;  // owner session expires if no command/heartbeat for this long
constexpr bool     STOP_ON_OWNER_LOSS  = true;   // fail-safe: stop the motor when the owner session is lost/released
constexpr uint32_t DIR_COAST_MS        = 1500;   // pause after a direction change before the motor may run again
constexpr uint8_t  DEFAULT_ON_DUTY     = 30;     // duty used by "ON" if the duty was 0
constexpr uint32_t MIN_REPLY_GAP_MS    = 300;    // limits replies to rejected commands

const int IR_SENSOR_PIN = 39; // Input-only pin (connected after 14.5k/22k + 1k divider)
constexpr uint8_t SENSOR_PIN   = 36; // ADC1 pin (connected after 14.5k/22k + 1k divider)
const int MOTOR_PWM_PIN = 22; // H-Bridge ENA
const int MOTOR_IN1_PIN = 26; // H-Bridge IN1
const int MOTOR_IN2_PIN = 27; // H-Bridge IN2
constexpr float R1 = 14500.0f;
constexpr float R2 = 22000.0f;
constexpr float DIVIDER_RATIO = R2 / (R1 + R2);

constexpr float SENSITIVITY_V_PER_A = 0.100f;
constexpr float NOMINAL_ZERO_V = 2.5f;

// ---------- Sampling ----------
constexpr uint16_t NUM_SAMPLES = 100;
constexpr uint32_t SAMPLE_INTERVAL_US = 200;
constexpr uint32_t PRINT_INTERVAL_MS = 250;
constexpr uint16_t CAL_SAMPLES = 2000;
constexpr float DEADBAND_A = 0.05f;

// ---------- RPM (IR + 4-arm fan) ----------
constexpr uint8_t  PULSES_PER_REV = 4;        // one pulse per fan arm
constexpr uint32_t MIN_PULSE_US   = 2000;     // debounce / noise rejection (2 ms)
constexpr uint32_t RPM_TIMEOUT_US = 500000;   // no pulse for 0.5 s -> RPM = 0 (motor stopped)
constexpr uint32_t RPM_WINDOW_MS  = 30000;    // RPM is only reported after 30 s of pulses, averaged over the last 30 s
constexpr uint32_t RPM_SNAP_MS    = 1000;     // pulse counter snapshot every 1 s
constexpr uint8_t  RPM_SNAP_N     = RPM_WINDOW_MS / RPM_SNAP_MS + 1;   // 31 snapshots span 30 s
constexpr uint32_t RPM_UPDATE_MS  = 200;

// ---------- State ----------
float zeroV = NOMINAL_ZERO_V;
float sensorVoltage = 0.0f;
float currentDC = 0.0f;
float currentRMS = 0.0f;
bool  curOutOfRange = false;     // ADC reading near 0 V or saturated -> sensor wiring problem
constexpr uint32_t REZERO_IDLE_MS = 1500;
uint32_t idleSinceMs = 0;
uint32_t sumMv = 0;
uint64_t sumSqMv = 0;
uint16_t sampleCount = 0;
uint32_t lastSampleUs = 0;
uint32_t lastPrintMs = 0;
uint32_t lastPublishMs = 0;
uint32_t lastRpmMs = 0;

// Current averaged over the publish interval (more precise than one 20 ms window)
float    curAccum  = 0.0f;
uint32_t curAccumN = 0;
long     lastCurMa = 0;

// Motor command as last applied (reported to the web app)
uint8_t motorDutyPct = 0;
uint8_t motorDir = 0;            // 0 = forward, 1 = reverse

// Remote command state (set by the owner)
bool     motorOn   = false;      // motor is OFF at power-up until the owner turns it on
uint8_t  cmdDuty   = 0;          // 0..100 %
uint8_t  cmdDir    = 0;          // 0 = forward, 1 = reverse
uint32_t dirLockUntil = 0;       // motor held off until this time after a direction change

// Owner session
bool     ownerActive = false;
char     ownerSid[16] = "";
uint32_t ownerLastMs = 0;
char     nonce[9] = "00000000";  // changes after every accepted command (replay protection)
uint32_t lastAckId  = 0;
uint8_t  lastAckRes = 0;         // 0 ok, 1 bad key/format, 2 another owner active, 3 stale nonce, 4 bad op/value
uint32_t lastReplyMs = 0;

// RPM state
volatile uint32_t pulseCount = 0;
volatile uint32_t lastPulseUs = 0;
portMUX_TYPE rpmMux = portMUX_INITIALIZER_UNLOCKED;
float rpm = 0.0f;
bool  rpmValid = true;           // false while the 30 s window is still filling
uint32_t rpmWarmLeftS = 0;
uint32_t snapMs[RPM_SNAP_N];
uint32_t snapCnt[RPM_SNAP_N];
uint8_t  snapHead = 0, snapLen = 0;
uint32_t lastSnapMs = 0;

// Network state
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
uint32_t lastWifiTryMs = 0;
uint32_t lastMqttTryMs = 0;
bool wifiWasUp = false;

void IRAM_ATTR onIrPulse() {
  const uint32_t now = micros();
  if (now - lastPulseUs >= MIN_PULSE_US) {
    lastPulseUs = now;
    pulseCount++;
  }
}

// RPM from pulses counted over a sliding 30 s window.
//  - no pulses for 0.5 s        -> motor stopped, RPM = 0 (valid), window restarts
//  - first 30 s after pulses    -> RPM not available yet (rpmValid = false)
//  - afterwards                 -> RPM averaged over the last 30 s, refreshed every second
void updateRpm(uint32_t nowMs) {
  uint32_t count, last;
  portENTER_CRITICAL(&rpmMux);
  count = pulseCount;
  last = lastPulseUs;
  portEXIT_CRITICAL(&rpmMux);

  if (micros() - last > RPM_TIMEOUT_US) {
    rpm = 0.0f;
    rpmValid = true;
    rpmWarmLeftS = 0;
    snapLen = 0;
    snapHead = 0;
    return;
  }

  if (snapLen == 0 || nowMs - lastSnapMs >= RPM_SNAP_MS) {
    lastSnapMs = nowMs;
    uint8_t i;
    if (snapLen < RPM_SNAP_N) {
      i = (snapHead + snapLen) % RPM_SNAP_N;
      snapLen++;
    } else {
      i = snapHead;
      snapHead = (snapHead + 1) % RPM_SNAP_N;
    }
    snapMs[i] = nowMs;
    snapCnt[i] = count;
  }

  const uint8_t newest = (snapHead + snapLen - 1) % RPM_SNAP_N;
  const uint32_t spanMs = snapMs[newest] - snapMs[snapHead];
  if (snapLen == RPM_SNAP_N && spanMs >= RPM_WINDOW_MS) {
    rpm = ((float)(snapCnt[newest] - snapCnt[snapHead]) / PULSES_PER_REV) * 60000.0f / (float)spanMs;
    rpmValid = true;
    rpmWarmLeftS = 0;
  } else {
    rpmValid = false;
    const uint32_t elapsed = nowMs - snapMs[snapHead];
    rpmWarmLeftS = (elapsed >= RPM_WINDOW_MS) ? 0 : (RPM_WINDOW_MS - elapsed + 999) / 1000;
  }
}

float pinMvToSensorV(float pinMv) {
  return (pinMv / 1000.0f) / DIVIDER_RATIO;
}

void calibrateZero() {
  uint64_t sum = 0;
  for (uint16_t i = 0; i < CAL_SAMPLES; i++) {
    sum += analogReadMilliVolts(SENSOR_PIN);
    delayMicroseconds(SAMPLE_INTERVAL_US);
  }
  float v = pinMvToSensorV((float)sum / CAL_SAMPLES);

  if (v > 1.0f && v < 3.0f) {
    zeroV = v;
    Serial.printf("Zero calibrated: %.4f V\n", zeroV);
  } else {
    Serial.printf("Calibration out of range (%.3f V), using nominal %.2f V. "
                  "Check wiring and ensure no load at startup.\n",
                  v, NOMINAL_ZERO_V);
  }
}

void forward(int value){
  analogWrite(MOTOR_PWM_PIN,value);
  digitalWrite(MOTOR_IN1_PIN,1);
  digitalWrite(MOTOR_IN2_PIN,0);
  motorDutyPct = (uint8_t)((value * 100 + 127) / 255);
  motorDir = 0;
}

void back(int value){
  analogWrite(MOTOR_PWM_PIN,value);
  digitalWrite(MOTOR_IN1_PIN,0);
  digitalWrite(MOTOR_IN2_PIN,1);
  motorDutyPct = (uint8_t)((value * 100 + 127) / 255);
  motorDir = 1;
}

void stop(){
  analogWrite(MOTOR_PWM_PIN,0);
  digitalWrite(MOTOR_IN1_PIN,0);
  digitalWrite(MOTOR_IN2_PIN,0);
  motorDutyPct = 0;
}

// ---------- Owner commands ----------
// Applies motorOn / cmdDuty / cmdDir to the H-bridge using the functions above.
void applyMotor() {
  if (dirLockUntil && (int32_t)(millis() - dirLockUntil) < 0) { stop(); return; }
  if (!motorOn || cmdDuty == 0) { stop(); return; }
  const int pwm = (cmdDuty * 255 + 50) / 100;
  if (cmdDir) back(pwm); else forward(pwm);
}

void newNonce() {
  snprintf(nonce, sizeof(nonce), "%08x", (unsigned)esp_random());
}

void hmacHex(const char* key, const char* msg, char* out65) {
  uint8_t h[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx, (const unsigned char*)key, strlen(key));
  mbedtls_md_hmac_update(&ctx, (const unsigned char*)msg, strlen(msg));
  mbedtls_md_hmac_finish(&ctx, h);
  mbedtls_md_free(&ctx);
  for (int i = 0; i < 32; i++) sprintf(out65 + 2 * i, "%02x", h[i]);
  out65[64] = 0;
}

bool constTimeEq(const char* a, const char* b, size_t n) {
  uint8_t d = 0;
  for (size_t i = 0; i < n; i++) d |= (uint8_t)(a[i] ^ b[i]);
  return d == 0;
}

// True when the owner session has been silent for longer than the lease.
// Signed difference on purpose: ownerLastMs can be a few ms NEWER than a nowMs that was
// read earlier in the same loop pass, and an unsigned subtraction would wrap to ~4 billion.
bool ownerLeaseExpired(uint32_t nowMs) {
  return (int32_t)(nowMs - ownerLastMs) > (int32_t)OWNER_LEASE_MS;
}

// Command format (text):  id|sid|op|val|nonce|hmac
//   hmac = HMAC-SHA256(OWNER_KEY, "id|sid|op|val|nonce") as 64 hex chars
// The key itself never travels over MQTT, and a captured command cannot be replayed
// because the nonce changes after every accepted command.
uint8_t processCommand(char* line, uint32_t nowMs, uint32_t* idOut) {
  *idOut = strtoul(line, nullptr, 10);

  char* lastBar = strrchr(line, '|');
  if (!lastBar) return 1;
  const char* sig = lastBar + 1;
  if (strlen(sig) != 64) return 1;
  *lastBar = '\0';                                   // line now holds the signed part

  char expect[65];
  hmacHex(OWNER_KEY, line, expect);
  if (!constTimeEq(expect, sig, 64)) return 1;       // wrong key

  char* save = nullptr;
  char* fId    = strtok_r(line, "|", &save);
  char* fSid   = strtok_r(nullptr, "|", &save);
  char* fOp    = strtok_r(nullptr, "|", &save);
  char* fVal   = strtok_r(nullptr, "|", &save);
  char* fNonce = strtok_r(nullptr, "|", &save);
  if (!fId || !fSid || !fOp || !fVal || !fNonce) return 4;

  if (strcmp(fNonce, nonce) != 0) return 3;          // stale / replayed

  const size_t sl = strlen(fSid);
  if (sl < 1 || sl > 12) return 4;
  for (size_t i = 0; i < sl; i++) if (!isalnum((unsigned char)fSid[i])) return 4;

  // lease: only one owner session at a time
  if (ownerActive && ownerLeaseExpired(nowMs)) {
    ownerActive = false;
    if (STOP_ON_OWNER_LOSS) { motorOn = false; applyMotor(); }
  }
  if (ownerActive && strcmp(ownerSid, fSid) != 0) return 2;

  const long val = atol(fVal);
  bool release = false;

  if (!strcmp(fOp, "hb")) {
    // heartbeat / login: nothing to do
  } else if (!strcmp(fOp, "rel")) {
    release = true;
  } else if (!strcmp(fOp, "on")) {
    motorOn = true;
    if (cmdDuty == 0) cmdDuty = DEFAULT_ON_DUTY;
    applyMotor();
  } else if (!strcmp(fOp, "off")) {
    motorOn = false;
    applyMotor();
  } else if (!strcmp(fOp, "duty")) {
    if (val < 0 || val > 100) return 4;
    cmdDuty = (uint8_t)val;
    applyMotor();
  } else if (!strcmp(fOp, "dir")) {
    if (val != 0 && val != 1) return 4;
    if ((uint8_t)val != cmdDir) {
      const bool wasRunning = motorDutyPct > 0;
      cmdDir = (uint8_t)val;
      if (wasRunning) {                              // never reverse a spinning motor directly
        motorOn = false;
        stop();
        dirLockUntil = nowMs + DIR_COAST_MS;
        if (dirLockUntil == 0) dirLockUntil = 1;
      }
      applyMotor();
    }
  } else {
    return 4;
  }

  if (release) {
    ownerActive = false;
    ownerSid[0] = '\0';
    if (STOP_ON_OWNER_LOSS) { motorOn = false; applyMotor(); }
  } else {
    ownerActive = true;
    strlcpy(ownerSid, fSid, sizeof(ownerSid));
    ownerLastMs = nowMs;
  }
  newNonce();
  return 0;
}

void publishTelemetry(bool periodic);

void onMqttMessage(char* topic, byte* payload, unsigned int len) {
  if (strcmp(topic, TOPIC_CMD) != 0) return;
  if (len == 0 || len >= 200) return;
  char line[200];
  memcpy(line, payload, len);
  line[len] = '\0';

  uint32_t id = 0;
  uint8_t res;
  if (OWNER_KEY[0] == '\0') {                        // no key configured -> remote control disabled
    id = strtoul(line, nullptr, 10);
    res = 1;
  } else {
    res = processCommand(line, millis(), &id);
  }
  lastAckId = id;
  lastAckRes = res;
  Serial.printf("CMD id=%lu result=%u\n", (unsigned long)id, (unsigned)res);

  const uint32_t now = millis();
  if (res == 0 || now - lastReplyMs >= MIN_REPLY_GAP_MS) {
    lastReplyMs = now;
    publishTelemetry(false);                         // immediate reply: new state + new nonce + ack
  }
}

// ---------- Wi-Fi + MQTT ----------
void connectMqtt() {
  String id = "motorstation-esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  Serial.printf("MQTT: connecting to %s:%u ... ", MQTT_HOST, MQTT_PORT);
  // Last will: if the ESP32 drops off, the broker publishes "offline" (retained).
  if (mqtt.connect(id.c_str(), nullptr, nullptr, TOPIC_STATUS, 0, true, "offline")) {
    mqtt.publish(TOPIC_STATUS, "online", true);
    mqtt.subscribe(TOPIC_CMD);
    Serial.println("connected");
  } else {
    Serial.printf("failed (rc=%d), will retry\n", mqtt.state());
  }
}

// Called every loop; never blocks for long.
void networkTick(uint32_t nowMs) {
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiWasUp) {
      wifiWasUp = false;
      Serial.println("WiFi: connection lost");
    }
    if (nowMs - lastWifiTryMs >= 15000) {
      lastWifiTryMs = nowMs;
      Serial.printf("WiFi: connecting to \"%s\" ...\n", WIFI_SSID);
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
    return;
  }

  if (!wifiWasUp) {
    wifiWasUp = true;
    Serial.print("WiFi: connected, IP ");
    Serial.println(WiFi.localIP());
  }

  if (!mqtt.connected()) {
    if (nowMs - lastMqttTryMs >= 5000) {
      lastMqttTryMs = nowMs;
      connectMqtt();
    }
    return;
  }
  mqtt.loop();
}

// JSON fields match what the web app expects. "vib" is left out on purpose:
// there is no vibration sensor on this board, so the app shows NO DATA for it.
// periodic = true  -> closes the current averaging window (every PUBLISH_INTERVAL_MS)
// periodic = false -> immediate reply to a command, reuses the last averaged current
void publishTelemetry(bool periodic) {
  if (!mqtt.connected()) return;

  if (periodic) {
    const float avgA = curAccumN ? (curAccum / (float)curAccumN) : fabsf(currentDC);
    curAccum = 0.0f;
    curAccumN = 0;
    long ma = lroundf(avgA * 1000.0f);
    if (ma > 65535) ma = 65535;
    lastCurMa = ma;
  }

  const uint8_t flags = curOutOfRange ? 1 : 0;   // bit 0 = current ADC out of range
  char rpmStr[12];
  if (rpmValid) snprintf(rpmStr, sizeof(rpmStr), "%ld", lroundf(rpm));
  else          snprintf(rpmStr, sizeof(rpmStr), "null");   // still warming up

  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"rpm\":%s,\"rw\":%lu,\"cur\":%ld,\"duty\":%u,\"mode\":0,\"dir\":%u,\"flags\":%u,\"target\":0,"
           "\"on\":%u,\"cd\":%u,\"own\":%u,\"n\":\"%s\",\"ack\":%lu,\"res\":%u}",
           rpmStr, (unsigned long)rpmWarmLeftS, lastCurMa, (unsigned)motorDutyPct, (unsigned)cmdDir,
           (unsigned)flags, (unsigned)(motorOn ? 1 : 0), (unsigned)cmdDuty, (unsigned)(ownerActive ? 1 : 0),
           nonce, (unsigned long)lastAckId, (unsigned)lastAckRes);
  mqtt.publish(TOPIC_TELEMETRY, buf);
}

void setup(){
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetPinAttenuation(SENSOR_PIN, ADC_11db);
  delay(500);
  calibrateZero();               // runs before Wi-Fi starts and before the motor is driven
  lastSampleUs = micros();
  pinMode(MOTOR_PWM_PIN,OUTPUT);
  pinMode(MOTOR_IN1_PIN,OUTPUT);
  pinMode(MOTOR_IN2_PIN,OUTPUT);
  pinMode(IR_SENSOR_PIN,INPUT);   // GPIO39 has no internal pull-up
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), onIrPulse, RISING);
  stop();
  newNonce();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWifiTryMs = millis();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(512);
  mqtt.setSocketTimeout(3);
  mqtt.setKeepAlive(15);
}

void loop(){
  // (the motor is now driven by the owner's commands, see applyMotor())
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  // 1. Fixed-cadence sampling
  if (nowUs - lastSampleUs >= SAMPLE_INTERVAL_US) {
    lastSampleUs += SAMPLE_INTERVAL_US;
    if (nowUs - lastSampleUs > 10 * SAMPLE_INTERVAL_US) lastSampleUs = nowUs;

    const uint32_t mv = analogReadMilliVolts(SENSOR_PIN);
    sumMv += mv;
    sumSqMv += (uint64_t)mv * mv;

    if (++sampleCount >= NUM_SAMPLES) {
      const float meanMv = (float)sumMv / NUM_SAMPLES;
      float varMv = (float)((double)sumSqMv / NUM_SAMPLES) - meanMv * meanMv;
      if (varMv < 0.0f) varMv = 0.0f;

      sensorVoltage = pinMvToSensorV(meanMv);

      // Slowly track zero while the motor is off
      if (motorDutyPct == 0) {
        if (!idleSinceMs) idleSinceMs = nowMs;
        if (nowMs - idleSinceMs > REZERO_IDLE_MS)
          zeroV += 0.01f * (sensorVoltage - zeroV);
      } else {
        idleSinceMs = 0;
      }

      currentDC = -(sensorVoltage - zeroV) / SENSITIVITY_V_PER_A;
      if (fabsf(currentDC) < 0.05f) currentDC = 0.0f;
      currentRMS = pinMvToSensorV(sqrtf(varMv)) / SENSITIVITY_V_PER_A;

      curOutOfRange = (meanMv < 150.0f || meanMv > 3100.0f);

      curAccum += fabsf(currentDC);   // feeds the averaged current that is published
      curAccumN++;

      sumMv = 0;
      sumSqMv = 0;
      sampleCount = 0;
    }
  }

  // 2. Wi-Fi / MQTT housekeeping
  networkTick(nowMs);

  // 3. Owner session fail-safe and direction-change pause
  if (ownerActive && ownerLeaseExpired(nowMs)) {
    ownerActive = false;
    Serial.println("Owner session expired");
    if (STOP_ON_OWNER_LOSS) { motorOn = false; applyMotor(); }
  }
  if (dirLockUntil && (int32_t)(nowMs - dirLockUntil) >= 0) {
    dirLockUntil = 0;
    applyMotor();
  }

  // 4. RPM (30 s window)
  if (nowMs - lastRpmMs >= RPM_UPDATE_MS) {
    lastRpmMs = nowMs;
    updateRpm(nowMs);
  }

  // 5. Publish to the web app (every PUBLISH_INTERVAL_MS)
  if (nowMs - lastPublishMs >= PUBLISH_INTERVAL_MS) {
    lastPublishMs = nowMs;
    publishTelemetry(true);
  }

  // 6. Serial print
  if (nowMs - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = nowMs;
    Serial.printf("Sensor: %.3f V | DC: %.2f mA | AC RMS: %.2f mA | RPM: %.1f%s | MQTT: %s\n",
                  sensorVoltage, currentDC*1000, currentRMS*1000, rpm,
                  rpmValid ? "" : " (warming up)",
                  mqtt.connected() ? "ok" : "no");
  }
}
