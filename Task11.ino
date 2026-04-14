#include <Arduino_HS300x.h>
#include <Arduino_BMI270_BMM150.h>
#include <Arduino_APDS9960.h>
#include <math.h>

// -------------------- Thresholds --------------------
const float humidityJumpThreshold = 8.0;    // baseline ~29.5, event ~48-55
const float tempRiseThreshold     = 0.5;    // baseline ~30.9, event ~31.5-31.8
const float magShiftThreshold     = 250.0;  // adjust if needed from your measured data
const int   clearChangeThreshold  = 30;     // baseline clear ~131-140, changed ~60-65
const int   rgbChangeThreshold    = 35;     // total RGB difference threshold

// -------------------- Cooldown --------------------
const unsigned long cooldownMs = 2000;
unsigned long lastEventTime = 0;
String latchedLabel = "BASELINE_NORMAL";

// -------------------- Baseline --------------------
bool baselineReady = false;
float baseTemp = 0.0;
float baseHum  = 0.0;
float baseMx = 0.0, baseMy = 0.0, baseMz = 0.0;
int baseR = 0, baseG = 0, baseB = 0, baseC = 0;

// -------------------- Current readings --------------------
float temperature = 0.0;
float humidity = 0.0;
float mx = 0.0, my = 0.0, mz = 0.0;
int r = 0, g = 0, b = 0, clearValue = 0;

// -------------------- Sensor-ready flags --------------------
bool gotMag = false;
bool gotColor = false;

// -------------------- Derived metrics --------------------
float magShift = 0.0;
bool humidJump = false;
bool tempRise = false;
bool magneticDisturbance = false;
bool lightOrColorChange = false;

// -------------------- Timing --------------------
unsigned long lastPrint = 0;
const unsigned long printIntervalMs = 300;

void setup() {
  Serial.begin(115200);
  delay(1500);

  if (!HS300x.begin()) {
    Serial.println("Failed to initialize HS300x sensor.");
    while (1);
  }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU.");
    while (1);
  }

  if (!APDS.begin()) {
    Serial.println("Failed to initialize APDS9960 sensor.");
    while (1);
  }

  Serial.println("Task 11 event detector started");
  Serial.println("Keep the board still for 3 seconds to collect baseline...");
  delay(3000);

  // -------- Collect baseline snapshot --------
  temperature = HS300x.readTemperature();
  humidity = HS300x.readHumidity();

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
    gotMag = true;
  }

  if (APDS.colorAvailable()) {
    APDS.readColor(r, g, b, clearValue);
    gotColor = true;
  }

  // If one of the sensors is not ready yet, keep trying briefly
  unsigned long startWait = millis();
  while ((!gotMag || !gotColor) && (millis() - startWait < 2000)) {
    if (!gotMag && IMU.magneticFieldAvailable()) {
      IMU.readMagneticField(mx, my, mz);
      gotMag = true;
    }
    if (!gotColor && APDS.colorAvailable()) {
      APDS.readColor(r, g, b, clearValue);
      gotColor = true;
    }
    delay(20);
  }

  baseTemp = temperature;
  baseHum = humidity;
  baseMx = mx;
  baseMy = my;
  baseMz = mz;
  baseR = r;
  baseG = g;
  baseB = b;
  baseC = clearValue;

  baselineReady = true;
}

void loop() {
  if (!baselineReady) return;

  // -------------------- Read sensors --------------------
  temperature = HS300x.readTemperature();
  humidity = HS300x.readHumidity();

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
  }

  if (APDS.colorAvailable()) {
    APDS.readColor(r, g, b, clearValue);
  }

  // -------------------- Compute event indicators --------------------
  humidJump = ((humidity - baseHum) > humidityJumpThreshold);
  tempRise  = ((temperature - baseTemp) > tempRiseThreshold);

  float dx = mx - baseMx;
  float dy = my - baseMy;
  float dz = mz - baseMz;
  magShift = fabs(dx) + fabs(dy) + fabs(dz);
  magneticDisturbance = (magShift > magShiftThreshold);

  int clearDiff = abs(clearValue - baseC);
  int rgbDiff = abs(r - baseR) + abs(g - baseG) + abs(b - baseB);
  lightOrColorChange = (clearDiff > clearChangeThreshold) || (rgbDiff > rgbChangeThreshold);

  // -------------------- Rule-based event classification --------------------
  // Priority:
  // 1) breath/warm air
  // 2) magnetic disturbance
  // 3) light/color change
  // 4) baseline normal
  String detectedLabel = "BASELINE_NORMAL";

  if (humidJump || tempRise) {
    detectedLabel = "BREATH_OR_WARM_AIR_EVENT";
  }
  else if (magneticDisturbance) {
    detectedLabel = "MAGNETIC_DISTURBANCE_EVENT";
  }
  else if (lightOrColorChange) {
    detectedLabel = "LIGHT_OR_COLOR_CHANGE_EVENT";
  }
  else {
    detectedLabel = "BASELINE_NORMAL";
  }

  // -------------------- Cooldown / debounce --------------------
  unsigned long now = millis();

  if (detectedLabel != "BASELINE_NORMAL") {
    if ((now - lastEventTime > cooldownMs) || (detectedLabel != latchedLabel)) {
      latchedLabel = detectedLabel;
      lastEventTime = now;
    }
  } else {
    if (now - lastEventTime > cooldownMs) {
      latchedLabel = "BASELINE_NORMAL";
    }
  }

  // -------------------- Required Serial Monitor output --------------------
  // raw,rh=<value>,temp=<value>,mag=<value>,r=<value>,g=<value>,b=<value>,clear=<value>
  // flags,humid_jump=<0/1>,temp_rise=<0/1>,mag_shift=<0/1>,light_or_color_change=<0/1>
  // event,<FINAL_LABEL>
  if (now - lastPrint >= printIntervalMs) {
    lastPrint = now;

    Serial.print("raw,rh=");
    Serial.print(humidity, 2);
    Serial.print(",temp=");
    Serial.print(temperature, 2);
    Serial.print(",mag=");
    Serial.print(magShift, 1);
    Serial.print(",r=");
    Serial.print(r);
    Serial.print(",g=");
    Serial.print(g);
    Serial.print(",b=");
    Serial.print(b);
    Serial.print(",clear=");
    Serial.println(clearValue);

    Serial.print("flags,humid_jump=");
    Serial.print(humidJump ? 1 : 0);
    Serial.print(",temp_rise=");
    Serial.print(tempRise ? 1 : 0);
    Serial.print(",mag_shift=");
    Serial.print(magneticDisturbance ? 1 : 0);
    Serial.print(",light_or_color_change=");
    Serial.println(lightOrColorChange ? 1 : 0);

    Serial.print("event,");
    Serial.println(latchedLabel);
  }
}