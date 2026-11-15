#include <WiFi.h>
#include <PubSubClient.h>   // Library Manager: "PubSubClient" by Nick O'Leary

// ====================== EDIT THESE ======================
const char* WIFI_SSID     = "TEData_94F1B3";
const char* WIFI_PASSWORD = "h9c23691";
// ========================================================

// ---------- MQTT (public test broker, no login) ----------
// The web app (MotorStation_App.html) must use the same broker and topic.
const char*    MQTT_HOST        = "broker.emqx.io";
const uint16_t MQTT_PORT        = 1883;                              // plain TCP for the ESP32
const char*    TOPIC_TELEMETRY  = "motorstation/demo-k7q2/telemetry"; // ESP32 -> web app
const char*    TOPIC_STATUS     = "motorstation/demo-k7q2/status";    // "online" / "offline" (last will)
constexpr uint32_t PUBLISH_INTERVAL_MS = 100;                        // 10 messages per second

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
constexpr uint32_t RPM_TIMEOUT_US = 500000;   // no pulse for 0.5 s -> RPM = 0

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

// Motor command as last applied (reported to the web app)
uint8_t motorDutyPct = 0;
uint8_t motorDir = 0;            // 0 = forward, 1 = reverse

// RPM state
volatile uint32_t pulseCount = 0;
volatile uint32_t lastPulseUs = 0;
portMUX_TYPE rpmMux = portMUX_INITIALIZER_UNLOCKED;
float rpm = 0.0f;
uint32_t prevCount = 0;
uint32_t prevPulseUs = 0;
bool haveRpmRef = false;

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

// Computes RPM from the time between pulse edges (accurate even at low speed)
void updateRpm() {
  uint32_t count, last;
  portENTER_CRITICAL(&rpmMux);
  count = pulseCount;
  last = lastPulseUs;
  portEXIT_CRITICAL(&rpmMux);

  const uint32_t newPulses = count - prevCount;

  if (newPulses > 0) {
    if (haveRpmRef) {
      const uint32_t dtUs = last - prevPulseUs;
      if (dtUs > 0) {
        rpm = ((float)newPulses / PULSES_PER_REV) * 60.0e6f / (float)dtUs;
      }
    }
    prevCount = count;
    prevPulseUs = last;
    haveRpmRef = true;
  } else if (micros() - last > RPM_TIMEOUT_US) {
    rpm = 0.0f;
    haveRpmRef = false;
    prevCount = count;
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

// ---------- Wi-Fi + MQTT ----------
void connectMqtt() {
  String id = "motorstation-esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  Serial.printf("MQTT: connecting to %s:%u ... ", MQTT_HOST, MQTT_PORT);
  // Last will: if the ESP32 drops off, the broker publishes "offline" (retained).
  if (mqtt.connect(id.c_str(), nullptr, nullptr, TOPIC_STATUS, 0, true, "offline")) {
    mqtt.publish(TOPIC_STATUS, "online", true);
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
void publishTelemetry() {
  if (!mqtt.connected()) return;

  long curMa = lroundf(fabsf(currentDC) * 1000.0f);
  if (curMa > 65535) curMa = 65535;
  const uint8_t flags = curOutOfRange ? 1 : 0;   // bit 0 = current ADC out of range

  char buf[160];
  snprintf(buf, sizeof(buf),
           "{\"rpm\":%ld,\"cur\":%ld,\"duty\":%u,\"mode\":0,\"dir\":%u,\"flags\":%u,\"target\":0}",
           lroundf(rpm), curMa, (unsigned)motorDutyPct, (unsigned)motorDir, (unsigned)flags);
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

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWifiTryMs = millis();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(256);
  mqtt.setSocketTimeout(3);
  mqtt.setKeepAlive(15);
}

void loop(){
  forward(255);
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
if (fabsf(currentDC) < 0.05f) currentDC = 0.0f;      currentRMS = pinMvToSensorV(sqrtf(varMv)) / SENSITIVITY_V_PER_A;

      curOutOfRange = (meanMv < 150.0f || meanMv > 3100.0f);

      sumMv = 0;
      sumSqMv = 0;
      sampleCount = 0;
    }
  }

  // 2. Wi-Fi / MQTT housekeeping
  networkTick(nowMs);

  // 3. RPM update + publish to the web app (10 Hz)
  if (nowMs - lastPublishMs >= PUBLISH_INTERVAL_MS) {
    lastPublishMs = nowMs;
    updateRpm();
    publishTelemetry();
  }

  // 4. Serial print
  if (nowMs - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = nowMs;
    Serial.printf("Sensor: %.3f V | DC: %.2f mA | AC RMS: %.2f mA | RPM: %.1f | MQTT: %s\n",
                  sensorVoltage, currentDC*1000, currentRMS*1000, rpm,
                  mqtt.connected() ? "ok" : "no");
  }
}
