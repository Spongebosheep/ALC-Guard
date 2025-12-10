#include <WiFi.h>
#include <HTTPClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <MPU6050.h>
#include <time.h>


// UK time zone (no daylight saving adjustment here for simplicity)
const long gmtOffset_sec     = 0;
const int  daylightOffset_sec = 0;

// Check if current time is within quiet hours (22:00–07:59).
// Used to suppress red warning LED during night.
bool isQuietHours() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    return false;
  }
  int hour = timeinfo.tm_hour;  // 0–23
  return (hour >= 22 || hour < 8);
}

// WiFi configuration
const char* WIFI_SSID     = "Hyperoptic Fibre 68C7 2.4G";
const char* WIFI_PASSWORD = "N4ZhNdh9hu6Cuh";

// Server configuration (Flask backend)
const char* SERVER_HOST = "192.168.1.129";   // Laptop / server IP
const int   SERVER_PORT = 5000;
const char* EVENT_PATH  = "/event";

// Pin definitions
#define FSR_PIN   34    // Bottom FSR (cup weight)
#define TEMP_PIN  25    // DS18B20 data pin

#define LED_R 16       // Common-anode RGB LED
#define LED_G 17
#define LED_B 18

// Alcohol sensor (MQ-3) interface
#define MQ3_PIN 35     // Analog input for AOUT

// Threshold placeholder for future automatic detection (0–4095).
const int MQ3_THRESHOLD = 2500;

// Smoothed analog read from MQ-3
int readMQ3Raw() {
  const int N = 10;
  long sum = 0;
  for (int i = 0; i < N; i++) {
    sum += analogRead(MQ3_PIN);
    delay(5);
  }
  return sum / N;
}

// Determine whether the drink is alcoholic.
bool isAlcoholDrink() {
  int mq = readMQ3Raw();           // kept for future calibration / logging
  Serial.print("MQ3 raw = ");
  Serial.println(mq);
  return (mq > MQ3_THRESHOLD);
}

// FSR reading (weight)
int readFSR() {
  const int N = 20;
  long sum = 0;
  for (int i = 0; i < N; i++) {
    sum += analogRead(FSR_PIN);
    delay(2);
  }
  return sum / N;
}

// Temperature sensor
OneWire oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);

// IMU (MPU6050)
MPU6050 mpu;
float tiltOffset = 0.0;

// Tilt smoothing and angular rate
float tiltFiltered      = 0.0;
float lastTiltFiltered  = 0.0;
float tiltRate          = 0.0;
unsigned long lastTiltUpdateMs = 0;

const float TILT_ALPHA = 0.2f;  // EMA smoothing factor

// Per-event maximum tilt and tilt rate (used to classify pour vs drink)
float eventMaxTilt     = 0.0f;
float eventMaxTiltRate = 0.0f;

float computeTiltAngleDegRaw(int16_t ax, int16_t ay, int16_t az) {
  return atan2(sqrt(pow(ax, 2) + pow(ay, 2)), az) * 180.0 / PI;
}

// RGB LED control (common anode)
void setColor(bool r, bool g, bool b) {
  // Common-anode: LOW = on, HIGH = off
  digitalWrite(LED_R, r ? LOW : HIGH);
  digitalWrite(LED_G, g ? LOW : HIGH);
  digitalWrite(LED_B, b ? LOW : HIGH);
}

void flashColor(bool r, bool g, bool b, int times, int on_ms, int off_ms) {
  for (int i = 0; i < times; i++) {
    setColor(r, g, b);
    delay(on_ms);
    setColor(false, false, false);
    delay(off_ms);
  }
}

// Global drink accumulation (ml)
// Used both by event logic and risk LED.
float totalDrinkMl = 0.0f;

// Daily high-risk threshold (ml of alcoholic beverage).
// Assumes a fixed-strength beer in this prototype.
const float HIGH_ALC_ML_THRESHOLD = 100.0f;

// Update risk LED based on totalDrinkMl.
// Alcohol mode only: red solid when above threshold, disabled at night.
void updateRiskLED() {
  if (totalDrinkMl >= HIGH_ALC_ML_THRESHOLD) {
    if (isQuietHours()) {
      setColor(false, false, false);  // quiet at night
    } else {
      setColor(true, false, false);   // solid red
    }
  } else {
    setColor(false, false, false);     // below threshold: off
  }
}


// FSR to volume calibration (ml)
const int   FSR_EMPTY  = 900;
const int   FSR_HALF   = 1900;
const int   FSR_FULL   = 2550;

const float CUP_CAPACITY_ML = 235.0f;
const float HALF_ML         = CUP_CAPACITY_ML / 2.0f;

// Piecewise-linear mapping: FSR to volume (ml)
float fsrToMl(int fsr) {
  if (fsr <= FSR_EMPTY) return 0.0f;

  // Empty ~ half-full
  if (fsr < FSR_HALF) {
    const float k1 = HALF_ML / (FSR_HALF - FSR_EMPTY);
    return (fsr - FSR_EMPTY) * k1;
  }

  // Half-full ~ full
  if (fsr < FSR_FULL) {
    const float k2 = (CUP_CAPACITY_ML - HALF_ML) / (FSR_FULL - FSR_HALF);
    return HALF_ML + (fsr - FSR_HALF) * k2;
  }

  // Above FSR_FULL: clamp to full capacity
  return CUP_CAPACITY_ML;
}

// Drink detection state
const int LIFT_THRESHOLD   = 400;   // weight < threshold - cup lifted
const int MIN_LEVEL  = 1100;  // minimum FSR level to consider "has liquid"

bool  inDrink = false;
int   drinkStartWeight = 0;
int   drinkEndWeight   = 0;

int   lastStableWeight = 0;
int   lastMaxOnTable   = 0;
int   prevWeight       = 0;

// End-of-event settling
bool          pendingEnd       = false;
unsigned long pendingEndStart  = 0;
const unsigned long END_SETTLE_TIME = 3000UL;

unsigned long lastDrinkTime    = 0;   // reserved for possible future use

// Tilt thresholds
const float DRINK_TILT_START = 20.0f;
const float DRINK_TILT_END   = 10.0f;

// Minimum volume change to classify as a drink (ml)
const float MIN_DRINK_ML = 10.0f;

// Serial JSON event output (for debugging / logging)
void sendEventJSON(const char* type,
                   float startMl,
                   float endMl,
                   float deltaMl,
                   int startWeight,
                   int endWeight,
                   unsigned long t_ms)
{
  Serial.print("{\"event\":\""); Serial.print(type); Serial.print("\"");
  Serial.print(",\"startWeight\":"); Serial.print(startWeight);
  Serial.print(",\"endWeight\":");   Serial.print(endWeight);
  Serial.print(",\"startMl\":");     Serial.print(startMl);
  Serial.print(",\"endMl\":");       Serial.print(endMl);
  Serial.print(",\"deltaMl\":");     Serial.print(deltaMl);
  Serial.print(",\"t_ms\":");        Serial.print(t_ms);
  Serial.println("}");
}

// HTTP POST to Flask server
void sendEventToServer(const char* type,
                       float volumeMl,
                       int startW,
                       int endW,
                       float startM,
                       float endM,
                       float tempC) {

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skip send");
    return;
  }

  String url = String("http://") + SERVER_HOST + ":" + String(SERVER_PORT) + EVENT_PATH;

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  String body = "{";
  body += "\"type\":\"" + String(type) + "\",";
  body += "\"volume\":" + String(volumeMl, 2) + ",";
  body += "\"startWeight\":" + String(startW) + ",";
  body += "\"endWeight\":" + String(endW) + ",";
  body += "\"startMl\":" + String(startM, 2) + ",";
  body += "\"endMl\":" + String(endM, 2) + ",";
  body += "\"temp\":" + String(tempC, 2);
  body += "}";

  Serial.print("POST ");
  Serial.print(url);
  Serial.print(" body: ");
  Serial.println(body);

  int httpCode = http.POST(body);
  Serial.print("HTTP code: ");
  Serial.println(httpCode);

  http.end();
}

// setup
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // Connect WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi connected, IP: ");
  Serial.println(WiFi.localIP());

  // NTP time for quiet hours and timestamps
  configTime(gmtOffset_sec, daylightOffset_sec,
             "pool.ntp.org", "time.nist.gov");
  Serial.println("Syncing time with NTP...");
  delay(2000);

  // Sensors init
  sensors.begin();

  Wire.begin(21, 22); // SDA, SCL
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 OK" : "MPU6050 ERROR");

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  setColor(false, false, false);

  // Tilt calibration: assume cup is upright during startup
  Serial.println("Calibrating tilt... keep cup upright...");
  const int N = 100;
  float sumTilt = 0;

  for (int i = 0; i < N; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float t = computeTiltAngleDegRaw(ax, ay, az);
    sumTilt += t;
    delay(10);
  }

  tiltOffset = sumTilt / N;
  Serial.print("Tilt offset set to: ");
  Serial.println(tiltOffset);
  Serial.println("Calibration done.");

  tiltFiltered      = 0.0f;
  lastTiltFiltered  = 0.0f;
  lastTiltUpdateMs  = millis();

  lastDrinkTime = millis();
}

// loop
void loop() {

  // 1. FSR (weight)
  int weight = readFSR();

  // 2. Temperature
  sensors.requestTemperatures();
  float tempC = sensors.getTempCByIndex(0);

  // 3. IMU and tilt
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float tiltDegRaw = computeTiltAngleDegRaw(ax, ay, az);
  float tiltDeg    = tiltDegRaw - tiltOffset;

  // Tilt smoothing and angular rate
  unsigned long nowMs = millis();

  if (lastTiltUpdateMs == 0) {
    tiltFiltered     = tiltDeg;
    lastTiltFiltered = tiltDeg;
    tiltRate         = 0.0f;
    lastTiltUpdateMs = nowMs;
  } else {
    float dt = (nowMs - lastTiltUpdateMs) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;

    tiltFiltered = TILT_ALPHA * tiltDeg +
                   (1.0f - TILT_ALPHA) * tiltFiltered;
    tiltRate = (tiltFiltered - lastTiltFiltered) / dt;

    lastTiltFiltered = tiltFiltered;
    lastTiltUpdateMs = nowMs;
  }

  // Track per-event max tilt / tiltRate for classification
  if (inDrink || pendingEnd) {
    float absTilt = fabs(tiltFiltered);
    float absRate = fabs(tiltRate);

    if (absTilt > eventMaxTilt) {
      eventMaxTilt = absTilt;
    }
    if (absRate > eventMaxTiltRate) {
      eventMaxTiltRate = absRate;
    }
  }

  // 4. Drink detection state machine 

  // Step 0: cup upright on table - update stable weight
  if (!inDrink &&
      !pendingEnd &&
      weight > LIFT_THRESHOLD &&
      fabs(tiltFiltered) < 8.0f) {
    lastStableWeight = weight;
    if (weight > lastMaxOnTable) {
      lastMaxOnTable = weight;
    }
  }

  // Step 1: detect start of drink (lifted or tilted)
  bool isLifted = (weight < LIFT_THRESHOLD || prevWeight < LIFT_THRESHOLD);
  bool isTilted = (fabs(tiltFiltered) > DRINK_TILT_START);

  if (!inDrink &&
      (isLifted || isTilted) &&
      lastMaxOnTable > MIN_LEVEL) {

    inDrink = true;
    drinkStartWeight = lastMaxOnTable;

    Serial.print("START DRINK, startWeight=");
    Serial.println(drinkStartWeight);

    // Briefly indicate "in use" with green LED
    setColor(false, true, false);
  }

  // Step 2: cup returned to table, wait for settling
  if (inDrink &&
      fabs(tiltFiltered) < DRINK_TILT_END &&
      weight > LIFT_THRESHOLD &&
      !pendingEnd) {

    inDrink = false;
    pendingEnd = true;
    pendingEndStart = millis();

    Serial.println("CUP PLACED, waiting to settle...");
  }

  // Step 3: after settling, classify the event
  if (pendingEnd) {
    if (millis() - pendingEndStart >= END_SETTLE_TIME) {

      pendingEnd = false;

      long sum = 0;
      const int M = 25;
      for (int i = 0; i < M; i++) {
        sum += readFSR();
        delay(20);
      }
      drinkEndWeight = sum / M;

      float mlStart = fsrToMl(drinkStartWeight);
      float mlEnd   = fsrToMl(drinkEndWeight);
      float deltaMl = mlStart - mlEnd;   // >0: liquid removed from cup

      Serial.print("END EVENT, start=");
      Serial.print(drinkStartWeight);
      Serial.print(" (");
      Serial.print(mlStart);
      Serial.print(" ml)  end=");
      Serial.print(drinkEndWeight);
      Serial.print(" (");
      Serial.print(mlEnd);
      Serial.print(" ml)  deltaMl=");
      Serial.println(deltaMl);

      // Check alcohol mode once per event
      bool alcoholic = isAlcoholDrink();
      if (!alcoholic) {
        Serial.println("[INFO] Non-alcohol drink detected, events will NOT be uploaded.");
      }

      // 1) Refill event: volume increased significantly
      if (deltaMl < -10.0f) {
        float refillMl = -deltaMl;

        Serial.print("REFILL EVENT: +");
        Serial.print(refillMl);
        Serial.println(" ml");

        if (alcoholic) {
          sendEventJSON("refill", mlStart, mlEnd, refillMl,
                        drinkStartWeight, drinkEndWeight, millis());

          sendEventToServer("refill", refillMl,
                            drinkStartWeight, drinkEndWeight,
                            mlStart, mlEnd, tempC);
        }
      }

      // 2) Liquid removed: either drink or pour-out
      else if (deltaMl > MIN_DRINK_ML) {
        float absMaxTilt = fabs(eventMaxTilt);
        float absMaxRate = fabs(eventMaxTiltRate);

        // High-speed + large tilt - classify as pour-out
        if (absMaxRate > 25.0f ||
           (absMaxTilt > 30.0f && absMaxRate > 12.0f)) {

          float pourMl = deltaMl;
          Serial.print("POUR_OUT EVENT: ");
          Serial.print(pourMl);
          Serial.println(" ml");

          if (alcoholic) {
            sendEventJSON("pour_out", mlStart, mlEnd, pourMl,
                          drinkStartWeight, drinkEndWeight, millis());

            sendEventToServer("pour_out", pourMl,
                              drinkStartWeight, drinkEndWeight,
                              mlStart, mlEnd, tempC);
          }
        }
        // Otherwise: normal drink
        else {
          float drinkMl = deltaMl;
          if (alcoholic) {
            totalDrinkMl += drinkMl;
          }
          lastDrinkTime = millis();

          Serial.print("DRINK EVENT: ");
          Serial.print(drinkMl);
          Serial.println(" ml");

          // Visual feedback:
          // alcoholic: yellow flash; non-alcoholic: blue flash
          if (alcoholic) {
            flashColor(true, false, false, 3, 120, 120);   // red
          } else {
            flashColor(false, false, true, 3, 120, 120);  // blue
          }

          if (alcoholic) {
            sendEventJSON("drink", mlStart, mlEnd, drinkMl,
                          drinkStartWeight, drinkEndWeight, millis());

            sendEventToServer("drink", drinkMl,
                              drinkStartWeight, drinkEndWeight,
                              mlStart, mlEnd, tempC);
          }
        }
      }

      // 3) Small sip: between noise and full drink
      else if (deltaMl > 1.0f) {
        float sipMl = deltaMl;
        if (alcoholic) {
            totalDrinkMl += sipMl;
        }
        lastDrinkTime = millis();

        Serial.print("SMALL SIP: ");
        Serial.print(sipMl);
        Serial.println(" ml");

        // alcoholic: yellow; non-alcoholic: blue
        if (alcoholic) {
          flashColor(true, true, false, 2, 100, 100);
        } else {
          flashColor(false, false, true, 2, 100, 100);
        }

        if (alcoholic) {
          sendEventJSON("small_sip", mlStart, mlEnd, sipMl,
                        drinkStartWeight, drinkEndWeight, millis());

          sendEventToServer("small_sip", sipMl,
                            drinkStartWeight, drinkEndWeight,
                            mlStart, mlEnd, tempC);
        }
      }

      // 4) Too small: ignore as noise
      else {
        Serial.println("No significant change, ignore.");
      }

      // Reset per-event tilt metrics
      eventMaxTilt     = 0.0f;
      eventMaxTiltRate = 0.0f;

      // Update red risk LED based on totalDrinkMl
      updateRiskLED();

      // Update stable weights
      lastStableWeight = drinkEndWeight;
      lastMaxOnTable   = drinkEndWeight;
    }
  }

  // 6. Frame-level JSON output (for live plotting / debug)
  Serial.print("{\"frame\":1");
  Serial.print(",\"weight\":");       Serial.print(weight);
  Serial.print(",\"temp\":");         Serial.print(tempC);
  Serial.print(",\"tilt_raw\":");     Serial.print(tiltDeg);
  Serial.print(",\"tilt_filt\":");    Serial.print(tiltFiltered);
  Serial.print(",\"tilt_rate\":");    Serial.print(tiltRate);
  Serial.print(",\"totalDrinkMl\":"); Serial.print(totalDrinkMl);
  Serial.print(",\"t_ms\":");         Serial.print(millis());
  Serial.println("}");

  prevWeight = weight;
  delay(150);
}
