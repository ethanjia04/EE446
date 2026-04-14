#include <PDM.h>
#include <Arduino_APDS9960.h>
#include <Arduino_BMI270_BMM150.h>
#include <math.h>

// -------------------- Microphone --------------------
short sampleBuffer[256];
volatile int samplesRead = 0;

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}

// -------------------- Thresholds --------------------
const int soundThreshold = 50;      // quiet ~3-10, noisy ~300-1100
const int darkThreshold  = 50;       // bright ~131-140, dark ~59-65
const int nearThreshold  = 100;      // near ~0, far ~237-240
const float motionThreshold = 5;  // based on gyro magnitude estimate

// -------------------- Timing --------------------
unsigned long lastPrint = 0;
const unsigned long printIntervalMs = 200;

// -------------------- Latest sensor values --------------------
int micLevel = 0;
int clearValue = 0;
int proximityValue = 255;
float gx = 0.0, gy = 0.0, gz = 0.0;
float motionMetric = 0.0;

// -------------------- Sensor-ready flags --------------------
bool gotColor = false;
bool gotProximity = false;
bool gotGyro = false;
bool gotMic = false;

// -------------------- Labels --------------------
String finalLabel = "QUIET_BRIGHT_STEADY_FAR";

void setup() {
  Serial.begin(115200);
  delay(1500);

  // Microphone
  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM microphone.");
    while (1);
  }

  // APDS9960
  if (!APDS.begin()) {
    Serial.println("Failed to initialize APDS9960 sensor.");
    while (1);
  }

  // IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU.");
    while (1);
  }

  Serial.println("Task 10 workspace classifier started");
}

void loop() {
  // -------------------- Read microphone --------------------
  if (samplesRead > 0) {
    long sum = 0;
    for (int i = 0; i < samplesRead; i++) {
      sum += abs(sampleBuffer[i]);
    }
    micLevel = sum / samplesRead;
    samplesRead = 0;
    gotMic = true;
  }

  // -------------------- Read ambient light --------------------
  if (APDS.colorAvailable()) {
    int r, g, b, c;
    APDS.readColor(r, g, b, c);
    clearValue = c;
    gotColor = true;
  }

  // -------------------- Read proximity --------------------
  if (APDS.proximityAvailable()) {
    proximityValue = APDS.readProximity();
    gotProximity = true;
  }

  // -------------------- Read gyroscope --------------------
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    motionMetric = fabs(gx) + fabs(gy) + fabs(gz);
    gotGyro = true;
  }

  // Wait until all key sensors have produced at least one valid reading
  if (!(gotMic && gotColor && gotProximity && gotGyro)) {
    return;
  }

  // -------------------- Binary decisions --------------------
  bool sound  = (micLevel > soundThreshold);
  bool dark   = (clearValue < darkThreshold);

  // Smaller proximity value means nearer
  bool near   = (proximityValue < nearThreshold);

  bool moving = (motionMetric > motionThreshold);

  // -------------------- Rule-based classification --------------------
  if (sound && !dark && moving && near) {
    finalLabel = "NOISY_BRIGHT_MOVING_NEAR";
  }
  else if (!sound && dark && !moving && near) {
    finalLabel = "QUIET_DARK_STEADY_NEAR";
  }
  else if (sound && !dark && !moving && !near) {
    finalLabel = "NOISY_BRIGHT_STEADY_FAR";
  }
  else {
    finalLabel = "QUIET_BRIGHT_STEADY_FAR";
  }

  // -------------------- Serial Monitor output --------------------
  // Required 3-line format:
  // raw,mic=<value>,clear=<value>,motion=<value>,prox=<value>
  // flags,sound=<0/1>,dark=<0/1>,moving=<0/1>,near=<0/1>
  // state,<FINAL_LABEL>
  if (millis() - lastPrint >= printIntervalMs) {
    lastPrint = millis();

    Serial.print("raw,mic=");
    Serial.print(micLevel);
    Serial.print(",clear=");
    Serial.print(clearValue);
    Serial.print(",motion=");
    Serial.print(motionMetric, 3);
    Serial.print(",prox=");
    Serial.println(proximityValue);

    Serial.print("flags,sound=");
    Serial.print(sound ? 1 : 0);
    Serial.print(",dark=");
    Serial.print(dark ? 1 : 0);
    Serial.print(",moving=");
    Serial.print(moving ? 1 : 0);
    Serial.print(",near=");
    Serial.println(near ? 1 : 0);

    Serial.print("state,");
    Serial.println(finalLabel);
  }
}