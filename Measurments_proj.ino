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

uint32_t sumMv = 0;
uint64_t sumSqMv = 0;
uint16_t sampleCount = 0;
uint32_t lastSampleUs = 0;
uint32_t lastPrintMs = 0;

// RPM state
volatile uint32_t pulseCount = 0;
volatile uint32_t lastPulseUs = 0;
portMUX_TYPE rpmMux = portMUX_INITIALIZER_UNLOCKED;
float rpm = 0.0f;
uint32_t prevCount = 0;
uint32_t prevPulseUs = 0;
bool haveRpmRef = false;

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
}

void back(int value){
  analogWrite(MOTOR_PWM_PIN,value);
  digitalWrite(MOTOR_IN1_PIN,0);
  digitalWrite(MOTOR_IN2_PIN,1);
}

void stop(){
  analogWrite(MOTOR_PWM_PIN,0);
  digitalWrite(MOTOR_IN1_PIN,0);
  digitalWrite(MOTOR_IN2_PIN,0);
}

void setup(){
  Serial.begin(115200);
  analogReadResolution(12);
  analogSetPinAttenuation(SENSOR_PIN, ADC_11db);
  delay(500);
  calibrateZero();
  lastSampleUs = micros();
  pinMode(MOTOR_PWM_PIN,OUTPUT);
  pinMode(MOTOR_IN1_PIN,OUTPUT);
  pinMode(MOTOR_IN2_PIN,OUTPUT);
  pinMode(IR_SENSOR_PIN,INPUT);   // GPIO39 has no internal pull-up
  attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), onIrPulse, RISING);
  stop();
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
      currentDC = (sensorVoltage - zeroV) / SENSITIVITY_V_PER_A;
      if (fabsf(currentDC) < DEADBAND_A) currentDC = 0.0f;

      currentRMS = pinMvToSensorV(sqrtf(varMv)) / SENSITIVITY_V_PER_A;

      sumMv = 0;
      sumSqMv = 0;
      sampleCount = 0;
    }
  }

  // 2. Periodic RPM update + print
  if (nowMs - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = nowMs;
    updateRpm();
    Serial.printf("Sensor: %.3f V | DC: %.2f mA | AC RMS: %.2f mA | RPM: %.1f\n",
                  sensorVoltage, currentDC*1000, currentRMS*1000, rpm);
  }
}