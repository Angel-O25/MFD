#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MCP9808.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <math.h>

// ── Model wrappers (NOT the raw model headers — those go in
//    condition_predictor.cpp / severity_predictor.cpp only) ───
#include "condition_predictor.h"
#include "severity_predictor.h"

// =============================================================
// PIN DEFINITIONS
// =============================================================
#define PIN_CT_L1 36
#define PIN_CT_L2 39
#define PIN_CT_L3 34
#define I2C_SDA   21
#define I2C_SCL   22

// =============================================================
// WIFI ACCESS POINT
// =============================================================
#define AP_SSID     "AgosSense"
#define AP_PASSWORD "agos1234"   // min 8 chars; use "" for open
#define AP_CHANNEL  6
#define AP_MAX_CONN 4

// =============================================================
// CALIBRATION
// =============================================================
float CAL_FACTOR_L1   = 1.18f;
float CAL_FACTOR_L2   = 0.90f;
float CAL_FACTOR_L3   = 1.50f;
float adc_voltage_ref = 3.3f;

// =============================================================
// NOISE GATES
// =============================================================
const float CURRENT_NOISE_GATE = 0.15f;
const float VIB_NOISE_GATE     = 0.30f;

// =============================================================
// HEALTH INDEX — normalization ranges (5th/95th pct from training)
// =============================================================
const float NORM_UNBAL_LO  =  2.0f,  NORM_UNBAL_HI  = 65.0f;
const float NORM_VIB_LO    =  0.2f,  NORM_VIB_HI    = 15.0f;
const float NORM_TEMP_LO   = 28.0f,  NORM_TEMP_HI   = 45.0f;
const float HI_WARN_THRESH =  0.60f;
const float HI_FAULT_THRESH=  0.40f;
const float EWMA_ALPHA     =  0.30f;
const float SAMPLE_INTERVAL=  1.5f;

// =============================================================
// THERMAL OVERLOAD THRESHOLDS (IEC 60947-4-1)
// =============================================================
const float TO_WARN_TEMP   =  50.0f;
const float TO_FAULT_TEMP  =  80.0f;
const float TO_CRIT_TEMP   = 100.0f;
const float TO_WARN_SLOPE  =   0.05f;
const float TO_FAULT_SLOPE =   0.10f;

// =============================================================
// TTF WINDOW + LCD WATCHDOG
// =============================================================
#define TTF_WINDOW      60
#define LCD_WATCHDOG_MS 15000UL

// =============================================================
// HARDWARE OBJECTS
// =============================================================
TFT_eSPI         tft = TFT_eSPI();
Adafruit_MPU6050 mpu;
Adafruit_MCP9808 mcp;
AsyncWebServer   server(80);

// =============================================================
// DATA BUFFERS (1 kHz, 1 second window)
// =============================================================
#define SAMPLE_RATE_HZ 1000
volatile uint16_t buffer_L1[SAMPLE_RATE_HZ];
volatile uint16_t buffer_L2[SAMPLE_RATE_HZ];
volatile uint16_t buffer_L3[SAMPLE_RATE_HZ];
volatile float    buffer_vibX[SAMPLE_RATE_HZ];
volatile float    buffer_vibY[SAMPLE_RATE_HZ];
volatile float    buffer_vibZ[SAMPLE_RATE_HZ];
volatile int      sample_index = 0;
volatile bool     buffer_ready = false;

// =============================================================
// EXTRACTED FEATURES
// =============================================================
volatile float feature_rms_L1 = 0.0f, feature_rms_L2 = 0.0f, feature_rms_L3 = 0.0f;
volatile float feature_current_unbalance = 0.0f;
volatile float feature_rms_vibX = 0.0f, feature_kurt_vibX = 0.0f, feature_crest_vibX = 0.0f;
volatile float feature_rms_vibY = 0.0f, feature_kurt_vibY = 0.0f, feature_crest_vibY = 0.0f;
volatile float feature_rms_vibZ = 0.0f, feature_kurt_vibZ = 0.0f, feature_crest_vibZ = 0.0f;
volatile float current_temp_c    = 0.0f;
volatile float ema_temp          = 25.0f;
volatile float feature_temp_slope= 0.0f;
volatile float prev_temp_c       = 0.0f;

// =============================================================
// INFERENCE OUTPUTS
// =============================================================
volatile int   fault_condition = 1;    // 0=EF  1=HEALTHY  2=MF
volatile int   fault_severity  = 2;    // 0=MILD 1=MOD 2=NONE 3=SEVERE
volatile float health_index    = 1.0f;
volatile float ewma_hi         = 1.0f;
volatile float ttf_seconds     = -1.0f;
volatile bool  is_anomaly      = false;
volatile int   thermal_status  = 0;    // 0=NORMAL 1=WARN 2=FAULT 3=CRIT

// =============================================================
// TTF STATE
// =============================================================
float hi_history[TTF_WINDOW] = {};
int   hi_history_idx  = 0;
bool  hi_history_full = false;

// =============================================================
// GRAPH BUFFERS
// =============================================================
#define GRAPH_WIDTH 230
volatile float hist_L1[GRAPH_WIDTH], hist_L2[GRAPH_WIDTH], hist_L3[GRAPH_WIDTH];
volatile float hist_vX[GRAPH_WIDTH], hist_vY[GRAPH_WIDTH], hist_vZ[GRAPH_WIDTH];
volatile float hist_temp[GRAPH_WIDTH], hist_slope[GRAPH_WIDTH];
volatile bool  update_graph_display = false;

// =============================================================
// STATE MACHINES
// =============================================================
enum PhaseType  { DETECTING, OFFLINE, SINGLE_PHASE, THREE_PHASE };
enum TabState   { HOME_TAB, GRAPH_TAB, NUMBER_TAB };
enum GraphMode  { GRAPH_ELEC, GRAPH_MECH, GRAPH_THERM };
enum CalState   { CAL_IDLE, CAL_RUNNING };

volatile PhaseType currentPhase    = DETECTING;
TabState           currentTab      = HOME_TAB;
bool               forceUIUpdate   = true;
GraphMode          currentGraphMode= GRAPH_ELEC;
volatile CalState  calState        = CAL_IDLE;
volatile unsigned long lastDrawMs  = 0;

volatile float cal_raw_rms_v_L1 = 0.0f;
volatile float cal_raw_rms_v_L2 = 0.0f;
volatile float cal_raw_rms_v_L3 = 0.0f;

// =============================================================
// FREERTOS
// =============================================================
TaskHandle_t      TaskSensorRead;
TaskHandle_t      TaskProcessData;
TaskHandle_t      TaskUI;
SemaphoreHandle_t i2cMutex;

// =============================================================
// PROTOTYPES
// =============================================================
void  sensorReadTask(void*);
void  processDataTask(void*);
void  uiTask(void*);
void  handleSerial();
void  printCalMenu();
void  reinitTFT();
void  drawTabs();
void  drawStaticContent();
void  drawDynamicContent();
void  drawGraphContent();
float calculateRMS(volatile uint16_t*, int, float, float*);
float calculateUnbalance(float, float, float);
void  calculateVibFeatures(volatile float*, int, volatile float&, volatile float&, volatile float&);
void  detectPhaseType();
int   mapFloatToY(float, float, float, int, int);

// =============================================================
// MATH HELPERS
// =============================================================
float normClamp(float v, float lo, float hi) {
    return constrain((v - lo) / (hi - lo + 1e-9f), 0.0f, 1.0f);
}

float computeHealthIndex(float unbal, float vib_mag, float temp) {
    return constrain(
        1.0f - ( 0.40f * normClamp(unbal,   NORM_UNBAL_LO, NORM_UNBAL_HI)
               + 0.35f * normClamp(vib_mag, NORM_VIB_LO,   NORM_VIB_HI)
               + 0.25f * normClamp(temp,    NORM_TEMP_LO,   NORM_TEMP_HI)),
        0.0f, 1.0f);
}

void fitLinearTrend(float* y, int n, float& slope, float& intercept) {
    float sx=0, sy=0, sxy=0, sxx=0;
    for (int i = 0; i < n; i++) { sx+=i; sy+=y[i]; sxy+=i*y[i]; sxx+=i*i; }
    float d = n*sxx - sx*sx;
    if (fabsf(d) < 1e-9f) { slope=0; intercept=sy/n; return; }
    slope     = (n*sxy - sx*sy) / d;
    intercept = (sy - slope*sx) / n;
}

// =============================================================
// JSON BUILDER (for /data endpoint)
// =============================================================
String buildJSON() {
    StaticJsonDocument<512> doc;
    doc["condition"] = fault_condition==0 ? "ELEC FAULT"
                     : fault_condition==2 ? "MECH FAULT"
                     :                      "HEALTHY";
    doc["severity"]  = fault_severity==0 ? "MILD"
                     : fault_severity==1 ? "MODERATE"
                     : fault_severity==3 ? "SEVERE"
                     :                     "NONE";
    doc["anomaly"]   = is_anomaly;
    doc["hi"]        = roundf(health_index * 1000.0f) / 1000.0f;
    doc["ewma_hi"]   = roundf(ewma_hi      * 1000.0f) / 1000.0f;
    doc["ttf_s"]     = ttf_seconds > 0 ? ttf_seconds : -1.0f;
    doc["L1"]        = roundf(feature_rms_L1 * 1000.0f) / 1000.0f;
    doc["L2"]        = roundf(feature_rms_L2 * 1000.0f) / 1000.0f;
    doc["L3"]        = roundf(feature_rms_L3 * 1000.0f) / 1000.0f;
    doc["unbal"]     = roundf(feature_current_unbalance * 10.0f) / 10.0f;
    doc["vib_x"]     = roundf(feature_rms_vibX * 1000.0f) / 1000.0f;
    doc["vib_y"]     = roundf(feature_rms_vibY * 1000.0f) / 1000.0f;
    doc["vib_z"]     = roundf(feature_rms_vibZ * 1000.0f) / 1000.0f;
    doc["kurt_max"]  = roundf(max(feature_kurt_vibX, max(feature_kurt_vibY, feature_kurt_vibZ)) * 10.0f) / 10.0f;
    doc["temp"]      = roundf(ema_temp * 100.0f) / 100.0f;
    doc["slope"]     = feature_temp_slope;
    String out; serializeJson(doc, out);
    return out;
}

// =============================================================
// SETUP
// =============================================================
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n\n=== AgosSense Boot ===");

    i2cMutex = xSemaphoreCreateMutex();
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);
    Wire.setTimeOut(20);
    analogReadResolution(12);

    if (!mpu.begin()) Serial.println("[WARN] MPU6050 not found");
    delay(200);
    if (!mcp.begin()) Serial.println("[WARN] MCP9808 not found");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);

    // TFT
    tft.init();
    tft.setRotation(2);
    uint16_t calData[5] = {275, 3620, 264, 3532, 1};
    tft.setTouch(calData);
    tft.fillScreen(TFT_BLACK);
    lastDrawMs = millis();

    for (int i = 0; i < GRAPH_WIDTH; i++) {
        hist_L1[i]=0; hist_L2[i]=0; hist_L3[i]=0;
        hist_vX[i]=0; hist_vY[i]=0; hist_vZ[i]=0;
        hist_temp[i]=0; hist_slope[i]=0;
    }

    // WiFi AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD, AP_CHANNEL, 0, AP_MAX_CONN);
    delay(200);
    Serial.printf("[WiFi] AP: SSID=%s  IP=%s\n",
        AP_SSID, WiFi.softAPIP().toString().c_str());

    // LittleFS
    if (!LittleFS.begin(true)) {
        Serial.println("[FS] LittleFS mount failed");
    } else {
        Serial.println("[FS] LittleFS OK");
    }

    // HTTP routes
    server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(LittleFS, "/index.html", "text/html");
    });
    server.on("/data", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "application/json", buildJSON());
    });
    server.onNotFound([](AsyncWebServerRequest* req) {
        req->send(404, "text/plain", "Not found");
    });
    server.begin();
    Serial.println("[HTTP] Server running on port 80");

    xTaskCreatePinnedToCore(sensorReadTask,  "Sensor",  4096,  NULL, 3, &TaskSensorRead,  1);
    xTaskCreatePinnedToCore(processDataTask, "Process", 8192,  NULL, 2, &TaskProcessData, 1);
    xTaskCreatePinnedToCore(uiTask,          "UI",      8192,  NULL, 1, &TaskUI,          0);
}

void loop() { vTaskDelete(NULL); }

// =============================================================
void reinitTFT() {
    tft.init(); tft.setRotation(2);
    uint16_t calData[5] = {275, 3620, 264, 3532, 1};
    tft.setTouch(calData);
    tft.fillScreen(TFT_BLACK);
    forceUIUpdate = true;
    lastDrawMs    = millis();
    Serial.println("[UI] LCD watchdog — reinitialised");
}

// =============================================================
void printCalMenu() {
    Serial.println("\n========================================");
    Serial.println("  AGOSSENSE CALIBRATION MENU");
    Serial.println("========================================");
    Serial.println("  h        - Show this menu");
    Serial.println("  s        - Show current settings");
    Serial.println("  r        - RAW mode (pre-cal voltages)");
    Serial.println("  x        - Exit RAW mode");
    Serial.println("  1,<val>  - Set CAL_FACTOR_L1");
    Serial.println("  2,<val>  - Set CAL_FACTOR_L2");
    Serial.println("  3,<val>  - Set CAL_FACTOR_L3");
    Serial.println("  CAL_FACTOR = clamp_amps / raw_rms_v");
    Serial.println("========================================\n");
}

void handleSerial() {
    if (!Serial.available()) return;
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input == "h") { printCalMenu(); }
    else if (input == "s") {
        Serial.printf("L1=%.4f  L2=%.4f  L3=%.4f  GATE=%.3fA\n",
            CAL_FACTOR_L1, CAL_FACTOR_L2, CAL_FACTOR_L3, CURRENT_NOISE_GATE);
    }
    else if (input == "r") { calState = CAL_RUNNING; Serial.println("[CAL] RAW mode ON"); }
    else if (input == "x") { calState = CAL_IDLE;    Serial.println("[CAL] RAW mode OFF"); }
    else if (input.startsWith("1,") || input.startsWith("2,") || input.startsWith("3,")) {
        char  ch  = input.charAt(0);
        float val = input.substring(2).toFloat();
        if (val <= 0) { Serial.println("[CAL] Invalid — must be > 0"); return; }
        if      (ch == '1') CAL_FACTOR_L1 = val;
        else if (ch == '2') CAL_FACTOR_L2 = val;
        else                CAL_FACTOR_L3 = val;
        Serial.printf("[CAL] L%c = %.4f\n", ch, val);
    }
}

// =============================================================
float calculateRMS(volatile uint16_t* buf, int len, float cal, float* raw_out) {
    double sum = 0;
    for (int i = 0; i < len; i++) sum += buf[i];
    double mean = sum / len;
    double ssq  = 0;
    for (int i = 0; i < len; i++) { double v = buf[i]-mean; ssq += v*v; }
    float rms_v = (sqrtf(ssq/len) / 4095.0f) * adc_voltage_ref;
    if (raw_out) *raw_out = rms_v;
    float amps = rms_v * cal;
    return (amps < CURRENT_NOISE_GATE) ? 0.0f : amps;
}

float calculateUnbalance(float l1, float l2, float l3) {
    if (l1==0 && l2==0 && l3==0) return 0.0f;
    float avg = (l1+l2+l3) / 3.0f;
    return (max(fabsf(l1-avg), max(fabsf(l2-avg), fabsf(l3-avg))) / avg) * 100.0f;
}

void calculateVibFeatures(volatile float* buf, int len,
                           volatile float& rms, volatile float& kurtosis, volatile float& crest) {
    float sum = 0;
    for (int i = 0; i < len; i++) sum += buf[i];
    float mean = sum / len;
    float ssq=0, sq4=0, peak=0;
    for (int i = 0; i < len; i++) {
        float d = buf[i]-mean, a = fabsf(d), sq = d*d;
        ssq += sq; sq4 += sq*sq;
        if (a > peak) peak = a;
    }
    float var = ssq / len;
    rms = sqrtf(var);
    if (rms < VIB_NOISE_GATE) { rms=0; kurtosis=0; crest=0; return; }
    kurtosis = (var > 0.0001f) ? (sq4/len) / (var*var) : 3.0f;
    crest    = (rms > 0.0001f) ? peak / rms : 1.0f;
}

void detectPhaseType() {
    bool a = feature_rms_L1 > CURRENT_NOISE_GATE;
    bool b = feature_rms_L2 > CURRENT_NOISE_GATE;
    bool c = feature_rms_L3 > CURRENT_NOISE_GATE;
    if (a && b && c)        currentPhase = THREE_PHASE;
    else if (a && !b && !c) currentPhase = SINGLE_PHASE;
    else if (!a && !b && !c)currentPhase = OFFLINE;
}

int mapFloatToY(float v, float lo, float hi, int yb, int yt) {
    if (v <= lo) return yb;
    if (v >= hi) return yt;
    return yb - (int)(((v-lo)/(hi-lo)) * (yb-yt));
}

// =============================================================
// TASK: 1 kHz SENSOR POLLING  (Core 1)
// =============================================================
void sensorReadTask(void* pv) {
    TickType_t wake = xTaskGetTickCount();
    bool mpu_tick   = false;
    for (;;) {
        if (!buffer_ready) {
            buffer_L1[sample_index] = ((uint32_t)analogRead(PIN_CT_L1)+analogRead(PIN_CT_L1)+analogRead(PIN_CT_L1)+analogRead(PIN_CT_L1)) >> 2;
            buffer_L2[sample_index] = ((uint32_t)analogRead(PIN_CT_L2)+analogRead(PIN_CT_L2)+analogRead(PIN_CT_L2)+analogRead(PIN_CT_L2)) >> 2;
            buffer_L3[sample_index] = ((uint32_t)analogRead(PIN_CT_L3)+analogRead(PIN_CT_L3)+analogRead(PIN_CT_L3)+analogRead(PIN_CT_L3)) >> 2;
            if (mpu_tick) {
                if (xSemaphoreTake(i2cMutex, 0) == pdTRUE) {
                    sensors_event_t a, g, t;
                    mpu.getEvent(&a, &g, &t);
                    buffer_vibX[sample_index] = a.acceleration.x;
                    buffer_vibY[sample_index] = a.acceleration.y;
                    buffer_vibZ[sample_index] = a.acceleration.z;
                    xSemaphoreGive(i2cMutex);
                } else if (sample_index > 0) {
                    buffer_vibX[sample_index] = buffer_vibX[sample_index-1];
                    buffer_vibY[sample_index] = buffer_vibY[sample_index-1];
                    buffer_vibZ[sample_index] = buffer_vibZ[sample_index-1];
                }
            } else if (sample_index > 0) {
                buffer_vibX[sample_index] = buffer_vibX[sample_index-1];
                buffer_vibY[sample_index] = buffer_vibY[sample_index-1];
                buffer_vibZ[sample_index] = buffer_vibZ[sample_index-1];
            }
            mpu_tick = !mpu_tick;
            if (++sample_index >= SAMPLE_RATE_HZ) { sample_index = 0; buffer_ready = true; }
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
    }
}

// =============================================================
// TASK: PROCESSING + INFERENCE  (Core 1, ~1 Hz)
// =============================================================
void processDataTask(void* pv) {
    for (;;) {
        handleSerial();
        if (!buffer_ready) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

        // Temperature
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            current_temp_c = mcp.readTempC();
            xSemaphoreGive(i2cMutex);
        }
        ema_temp = 0.1f * current_temp_c + 0.9f * ema_temp;
        feature_temp_slope = (prev_temp_c != 0.0f) ? ema_temp - prev_temp_c : 0.0f;
        prev_temp_c = ema_temp;

        // Current
        float rv1=0, rv2=0, rv3=0;
        feature_rms_L1 = calculateRMS(buffer_L1, SAMPLE_RATE_HZ, CAL_FACTOR_L1, &rv1);
        feature_rms_L2 = calculateRMS(buffer_L2, SAMPLE_RATE_HZ, CAL_FACTOR_L2, &rv2);
        feature_rms_L3 = calculateRMS(buffer_L3, SAMPLE_RATE_HZ, CAL_FACTOR_L3, &rv3);
        feature_current_unbalance = calculateUnbalance(feature_rms_L1, feature_rms_L2, feature_rms_L3);
        cal_raw_rms_v_L1=rv1; cal_raw_rms_v_L2=rv2; cal_raw_rms_v_L3=rv3;

        // Vibration
        calculateVibFeatures(buffer_vibX, SAMPLE_RATE_HZ, feature_rms_vibX, feature_kurt_vibX, feature_crest_vibX);
        calculateVibFeatures(buffer_vibY, SAMPLE_RATE_HZ, feature_rms_vibY, feature_kurt_vibY, feature_crest_vibY);
        calculateVibFeatures(buffer_vibZ, SAMPLE_RATE_HZ, feature_rms_vibZ, feature_kurt_vibZ, feature_crest_vibZ);
        detectPhaseType();

        // Feature vector — order must match training pipeline:
        // L1 L2 L3 Unbal VibX_RMS VibY_RMS VibZ_RMS
        // VibX_K VibY_K VibZ_K  VibX_C VibY_C VibZ_C  Temp Slope
        float feat[15] = {
            feature_rms_L1, feature_rms_L2, feature_rms_L3,
            feature_current_unbalance,
            feature_rms_vibX, feature_rms_vibY, feature_rms_vibZ,
            feature_kurt_vibX, feature_kurt_vibY, feature_kurt_vibZ,
            feature_crest_vibX, feature_crest_vibY, feature_crest_vibZ,
            ema_temp, feature_temp_slope
        };

        // ML Inference (separate translation units — no class clash)
        fault_condition = predictCondition(feat);  // 0=EF 1=HEALTHY 2=MF
        fault_severity  = predictSeverity(feat);   // 0=MILD 1=MOD 2=NONE 3=SEVERE

        // Health Index
        float vib_mag = sqrtf(feature_rms_vibX*feature_rms_vibX +
                              feature_rms_vibY*feature_rms_vibY +
                              feature_rms_vibZ*feature_rms_vibZ);
        health_index = computeHealthIndex(feature_current_unbalance, vib_mag, ema_temp);
        ewma_hi      = EWMA_ALPHA * health_index + (1.0f - EWMA_ALPHA) * ewma_hi;
        is_anomaly   = (ewma_hi < HI_WARN_THRESH);

        // Thermal Overload (IEC 60947-4-1)
        if      (ema_temp >= TO_CRIT_TEMP  || feature_temp_slope >= 0.15f)        thermal_status = 3;
        else if (ema_temp >= TO_FAULT_TEMP || feature_temp_slope >= TO_FAULT_SLOPE)thermal_status = 2;
        else if (ema_temp >= TO_WARN_TEMP  || feature_temp_slope >= TO_WARN_SLOPE) thermal_status = 1;
        else                                                                        thermal_status = 0;

        // TTF
        hi_history[hi_history_idx] = ewma_hi;
        hi_history_idx = (hi_history_idx + 1) % TTF_WINDOW;
        if (hi_history_idx == 0) hi_history_full = true;
        if (hi_history_full) {
            float ordered[TTF_WINDOW];
            for (int i = 0; i < TTF_WINDOW; i++)
                ordered[i] = hi_history[(hi_history_idx + i) % TTF_WINDOW];
            float sl, ic;
            fitLinearTrend(ordered, TTF_WINDOW, sl, ic);
            ttf_seconds = (sl < -1e-5f)
                ? max(0.0f, (HI_FAULT_THRESH - ordered[TTF_WINDOW-1]) / sl) * SAMPLE_INTERVAL
                : -1.0f;
        }

        // Shift graph buffers
        for (int i = 0; i < GRAPH_WIDTH-1; i++) {
            hist_L1[i]=hist_L1[i+1]; hist_L2[i]=hist_L2[i+1]; hist_L3[i]=hist_L3[i+1];
            hist_vX[i]=hist_vX[i+1]; hist_vY[i]=hist_vY[i+1]; hist_vZ[i]=hist_vZ[i+1];
            hist_temp[i]=hist_temp[i+1]; hist_slope[i]=hist_slope[i+1];
        }
        hist_L1[GRAPH_WIDTH-1]   = feature_rms_L1;
        hist_L2[GRAPH_WIDTH-1]   = feature_rms_L2;
        hist_L3[GRAPH_WIDTH-1]   = feature_rms_L3;
        hist_vX[GRAPH_WIDTH-1]   = feature_rms_vibX;
        hist_vY[GRAPH_WIDTH-1]   = feature_rms_vibY;
        hist_vZ[GRAPH_WIDTH-1]   = feature_rms_vibZ;
        hist_temp[GRAPH_WIDTH-1] = ema_temp;
        hist_slope[GRAPH_WIDTH-1]= feature_temp_slope;
        update_graph_display = true;

        // Serial output
        if (calState == CAL_RUNNING) {
            Serial.printf("[RAW] ms=%lu  L1_v=%.6f  L2_v=%.6f  L3_v=%.6f  -> %.3f / %.3f / %.3f A\n",
                millis(), rv1, rv2, rv3,
                (float)feature_rms_L1, (float)feature_rms_L2, (float)feature_rms_L3);
        } else {
            Serial.printf("%lu,%.3f,%.3f,%.3f,%.1f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f\n",
                millis(),
                (float)feature_rms_L1,(float)feature_rms_L2,(float)feature_rms_L3,
                (float)feature_current_unbalance,
                (float)feature_rms_vibX,(float)feature_rms_vibY,(float)feature_rms_vibZ,
                (float)feature_kurt_vibX,(float)feature_kurt_vibY,(float)feature_kurt_vibZ,
                (float)feature_crest_vibX,(float)feature_crest_vibY,(float)feature_crest_vibZ,
                (float)ema_temp,(float)feature_temp_slope);
        }

        buffer_ready = false;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =============================================================
// TASK: TFT UI  (Core 0)
// =============================================================
void uiTask(void* pv) {
    forceUIUpdate = true;
    lastDrawMs    = millis();
    for (;;) {
        if (millis() - lastDrawMs > LCD_WATCHDOG_MS) reinitTFT();

        uint16_t rx, ry;
        bool pressed = tft.getTouch(&rx, &ry);
        if (pressed) {
            uint16_t x = map(ry, 320, 0, 0, 240);
            uint16_t y = map(rx, 0, 240, 0, 320);
            if (y > 270) {
                if      (x <  80 && currentTab != HOME_TAB)   { currentTab = HOME_TAB;   forceUIUpdate = true; }
                else if (x < 160 && currentTab != GRAPH_TAB)  { currentTab = GRAPH_TAB;  forceUIUpdate = true; }
                else if (x >=160 && currentTab != NUMBER_TAB) { currentTab = NUMBER_TAB; forceUIUpdate = true; }
                vTaskDelay(pdMS_TO_TICKS(200));
            } else if (currentTab == GRAPH_TAB && y < 30) {
                if      (x <  80) currentGraphMode = GRAPH_ELEC;
                else if (x < 160) currentGraphMode = GRAPH_MECH;
                else              currentGraphMode = GRAPH_THERM;
                forceUIUpdate = true;
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }

        if (forceUIUpdate) {
            tft.fillScreen(TFT_BLACK);
            drawTabs();
            drawStaticContent();
            forceUIUpdate = false;
        }
        if (currentTab == GRAPH_TAB && update_graph_display) {
            drawGraphContent();
            update_graph_display = false;
        }
        drawDynamicContent();
        lastDrawMs = millis();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// =============================================================
// UI COLOUR / LABEL HELPERS
// =============================================================
uint16_t conditionColor(int c) {
    switch (c) { case 0: return TFT_RED; case 1: return TFT_GREEN; case 2: return TFT_ORANGE; }
    return TFT_WHITE;
}
const char* conditionLabel(int c) {
    switch (c) { case 0: return "ELEC FAULT"; case 1: return "HEALTHY"; case 2: return "MECH FAULT"; }
    return "UNKNOWN";
}
const char* severityLabel(int s) {
    switch (s) { case 0: return "MILD"; case 1: return "MODERATE"; case 2: return "NONE"; case 3: return "SEVERE"; }
    return "?";
}
uint16_t hiColor(float h) { return h > 0.60f ? TFT_GREEN : h > 0.40f ? TFT_ORANGE : TFT_RED; }
const char* thermalLabel(int ts) {
    switch (ts) { case 0: return "NORMAL"; case 1: return "WARN"; case 2: return "FAULT"; case 3: return "CRIT!"; }
    return "?";
}
uint16_t thermalColor(int ts) {
    switch (ts) { case 0: return TFT_GREEN; case 1: return TFT_YELLOW; case 2: return TFT_ORANGE; case 3: return TFT_RED; }
    return TFT_WHITE;
}

// =============================================================
void drawTabs() {
    tft.fillRect(0,   280, 80, 40, currentTab==HOME_TAB   ? TFT_BLUE : TFT_DARKGREY);
    tft.fillRect(80,  280, 80, 40, currentTab==GRAPH_TAB  ? TFT_BLUE : TFT_DARKGREY);
    tft.fillRect(160, 280, 80, 40, currentTab==NUMBER_TAB ? TFT_BLUE : TFT_DARKGREY);
    tft.setTextColor(TFT_WHITE); tft.setTextSize(1);
    tft.drawCentreString("HOME",  40,  290, 2);
    tft.drawCentreString("GRAPH", 120, 290, 2);
    tft.drawCentreString("NUMS",  200, 290, 2);
}

void drawStaticContent() {
    tft.setTextColor(TFT_LIGHTGREY); tft.setTextSize(1);

    if (currentTab == HOME_TAB) {
        tft.setCursor(4,   4); tft.print("Phase:");
        tft.setCursor(4,  40); tft.print("Fault:");
        tft.setCursor(4,  76); tft.print("Severity:");
        tft.setCursor(4, 110); tft.print("Thermal:");
        tft.setCursor(4, 140); tft.print("Health:");
        tft.drawRect(4, 155, 232, 14, TFT_WHITE);
        tft.setCursor(4, 178); tft.print("TTF:");
        tft.setCursor(4, 214); tft.print("L1/L2/L3 (A):");
        tft.setCursor(4, 250); tft.print("Temp:");
    }
    else if (currentTab == NUMBER_TAB) {
        tft.setTextSize(2);
        tft.setCursor(10,  15); tft.print("L1 RMS(A):");
        tft.setCursor(10,  50); tft.print("L2 RMS(A):");
        tft.setCursor(10,  85); tft.print("L3 RMS(A):");
        tft.setCursor(10, 120); tft.print("Unbal(%):");
        tft.setCursor(10, 155); tft.print("Ovr Vib :");
        tft.setCursor(10, 190); tft.print("Max Kurt:");
        tft.setCursor(10, 225); tft.print("Temp(C) :");
        tft.setCursor(10, 260); tft.print("Slope   :");
    }
    else if (currentTab == GRAPH_TAB) {
        tft.fillRect(0,   0, 80, 30, currentGraphMode==GRAPH_ELEC  ? TFT_MAROON : TFT_BLACK);
        tft.fillRect(80,  0, 80, 30, currentGraphMode==GRAPH_MECH  ? TFT_MAROON : TFT_BLACK);
        tft.fillRect(160, 0, 80, 30, currentGraphMode==GRAPH_THERM ? TFT_MAROON : TFT_BLACK);
        tft.drawRect(0,   0, 80, 30, TFT_WHITE);
        tft.drawRect(80,  0, 80, 30, TFT_WHITE);
        tft.drawRect(160, 0, 80, 30, TFT_WHITE);
        tft.drawCentreString("ELEC",  40,   8, 2);
        tft.drawCentreString("MECH",  120,  8, 2);
        tft.drawCentreString("THERM", 200,  8, 2);
        tft.drawRect(4, 34, GRAPH_WIDTH+2, 242, TFT_WHITE);
        update_graph_display = true;
    }
}

void drawDynamicContent() {
    if (currentTab == HOME_TAB) {
        tft.setTextSize(1);

        // Phase
        tft.setTextColor(TFT_CYAN, TFT_BLACK);
        tft.setCursor(60, 4);
        if      (currentPhase == DETECTING)    tft.print("Detecting...  ");
        else if (currentPhase == OFFLINE)      tft.print("Offline       ");
        else if (currentPhase == SINGLE_PHASE) tft.print("1-Phase       ");
        else                                   tft.print("3-Phase       ");

        // Fault condition
        uint16_t cc = conditionColor(fault_condition);
        tft.fillRect(60, 36, 178, 20, TFT_BLACK);
        tft.setTextColor(cc, TFT_BLACK);
        tft.setCursor(60, 40);
        tft.print(conditionLabel(fault_condition));
        if (calState == CAL_RUNNING) {
            tft.fillRect(60, 36, 178, 20, TFT_RED);
            tft.setTextColor(TFT_YELLOW, TFT_RED);
            tft.setCursor(62, 40); tft.print("[ CAL MODE ]");
        }

        // Severity
        tft.fillRect(80, 72, 158, 18, TFT_BLACK);
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setCursor(80, 76);
        tft.print(severityLabel(fault_severity));

        // Thermal
        tft.fillRect(80, 106, 158, 18, TFT_BLACK);
        tft.setTextColor(thermalColor(thermal_status), TFT_BLACK);
        tft.setCursor(80, 110);
        tft.print(thermalLabel(thermal_status));

        // Health index value
        float hi = (float)ewma_hi;
        uint16_t hc = hiColor(hi);
        tft.fillRect(60, 136, 176, 18, TFT_BLACK);
        tft.setTextColor(hc, TFT_BLACK);
        tft.setCursor(60, 140);
        char hibuf[18];
        snprintf(hibuf, sizeof(hibuf), "%.2f (%.0f%%)", hi, hi*100.0f);
        tft.print(hibuf);

        // Health bar
        tft.fillRect(5, 156, 230, 12, TFT_DARKGREY);
        int bw = constrain((int)(hi * 230), 0, 230);
        tft.fillRect(5, 156, bw, 12, hc);
        tft.drawLine(5+(int)(0.40f*230), 156, 5+(int)(0.40f*230), 167, TFT_RED);
        tft.drawLine(5+(int)(0.60f*230), 156, 5+(int)(0.60f*230), 167, TFT_ORANGE);

        // TTF
        tft.fillRect(36, 174, 202, 20, TFT_BLACK);
        tft.setTextColor(TFT_MAGENTA, TFT_BLACK);
        tft.setCursor(36, 178);
        float ttf = (float)ttf_seconds;
        if (ttf > 0) {
            char buf[30];
            snprintf(buf, sizeof(buf), "%.0fs (%.1fmin)", ttf, ttf/60.0f);
            tft.print(buf);
        } else {
            tft.print("Stable / Not degrading");
        }

        // Current summary
        tft.fillRect(4, 224, 234, 22, TFT_BLACK);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(4, 228);
        char cbuf[36];
        snprintf(cbuf, sizeof(cbuf), "%.2f / %.2f / %.2fA",
            (float)feature_rms_L1, (float)feature_rms_L2, (float)feature_rms_L3);
        tft.print(cbuf);

        // Temperature
        tft.fillRect(44, 250, 194, 18, TFT_BLACK);
        tft.setTextColor(TFT_ORANGE, TFT_BLACK);
        tft.setCursor(44, 254);
        char tbuf[26];
        snprintf(tbuf, sizeof(tbuf), "%.1fC  sl:%.4f", (float)ema_temp, (float)feature_temp_slope);
        tft.print(tbuf);
    }
    else if (currentTab == NUMBER_TAB) {
        tft.setTextSize(2); tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        float vib = sqrtf(feature_rms_vibX*feature_rms_vibX + feature_rms_vibY*feature_rms_vibY + feature_rms_vibZ*feature_rms_vibZ);
        float kurt = max(feature_kurt_vibX, max(feature_kurt_vibY, feature_kurt_vibZ));
        tft.setCursor(140,  15); tft.print(feature_rms_L1, 2);           tft.print("  ");
        tft.setCursor(140,  50); tft.print(feature_rms_L2, 2);           tft.print("  ");
        tft.setCursor(140,  85); tft.print(feature_rms_L3, 2);           tft.print("  ");
        tft.setCursor(140, 120); tft.print(feature_current_unbalance, 1);tft.print("  ");
        tft.setCursor(140, 155); tft.print(vib, 2);                      tft.print("  ");
        tft.setCursor(140, 190); tft.print(kurt, 1);                     tft.print("  ");
        tft.setCursor(140, 225); tft.print(ema_temp, 1);                 tft.print("  ");
        tft.setCursor(140, 260); tft.print(feature_temp_slope, 2);       tft.print("  ");
    }
}

void drawGraphContent() {
    const int xs=5, yt=35, yb=275;
    tft.fillRect(xs, yt, GRAPH_WIDTH, yb-yt, TFT_BLACK);
    tft.setTextSize(1);

    if (currentGraphMode == GRAPH_ELEC) {
        int mid = yt + (yb-yt)/2;
        tft.drawLine(xs, mid, xs+GRAPH_WIDTH, mid, TFT_DARKGREY);
        tft.setTextColor(TFT_LIGHTGREY);
        tft.setCursor(xs+2, yt+4);   tft.print("+15A");
        tft.setCursor(xs+2, mid-10); tft.print("0A");
        tft.setCursor(xs+2, yb-12);  tft.print("-15A");
        long s1=0, s2=0, s3=0;
        for (int i = 0; i < GRAPH_WIDTH; i++) { s1+=buffer_L1[i]; s2+=buffer_L2[i]; s3+=buffer_L3[i]; }
        float o1=s1/GRAPH_WIDTH, o2=s2/GRAPH_WIDTH, o3=s3/GRAPH_WIDTH;
        for (int i = 0; i < GRAPH_WIDTH-1; i++) {
            auto A=[&](float r,float o,float f){ return ((r-o)/4095.0f)*adc_voltage_ref*f; };
            tft.drawLine(xs+i,   mapFloatToY(A(buffer_L1[i],  o1,CAL_FACTOR_L1),-15,15,yb,yt),
                         xs+i+1, mapFloatToY(A(buffer_L1[i+1],o1,CAL_FACTOR_L1),-15,15,yb,yt), TFT_RED);
            tft.drawLine(xs+i,   mapFloatToY(A(buffer_L2[i],  o2,CAL_FACTOR_L2),-15,15,yb,yt),
                         xs+i+1, mapFloatToY(A(buffer_L2[i+1],o2,CAL_FACTOR_L2),-15,15,yb,yt), TFT_GREEN);
            tft.drawLine(xs+i,   mapFloatToY(A(buffer_L3[i],  o3,CAL_FACTOR_L3),-15,15,yb,yt),
                         xs+i+1, mapFloatToY(A(buffer_L3[i+1],o3,CAL_FACTOR_L3),-15,15,yb,yt), TFT_BLUE);
        }
    }
    else if (currentGraphMode == GRAPH_MECH) {
        int mid = mapFloatToY(10, 0, 20, yb, yt);
        tft.drawLine(xs, mid, xs+GRAPH_WIDTH, mid, TFT_DARKGREY);
        tft.setTextColor(TFT_LIGHTGREY);
        tft.setCursor(xs+2, yt+4);  tft.print("20ms2");
        tft.setCursor(xs+2, mid-10);tft.print("10");
        tft.setTextColor(TFT_CYAN);    tft.setCursor(xs+50,  yt+4); tft.print("vX");
        tft.setTextColor(TFT_MAGENTA); tft.setCursor(xs+80,  yt+4); tft.print("vY");
        tft.setTextColor(TFT_YELLOW);  tft.setCursor(xs+110, yt+4); tft.print("vZ");
        for (int i = 0; i < GRAPH_WIDTH-1; i++) {
            tft.drawLine(xs+i, mapFloatToY(hist_vX[i],0,20,yb,yt), xs+i+1, mapFloatToY(hist_vX[i+1],0,20,yb,yt), TFT_CYAN);
            tft.drawLine(xs+i, mapFloatToY(hist_vY[i],0,20,yb,yt), xs+i+1, mapFloatToY(hist_vY[i+1],0,20,yb,yt), TFT_MAGENTA);
            tft.drawLine(xs+i, mapFloatToY(hist_vZ[i],0,20,yb,yt), xs+i+1, mapFloatToY(hist_vZ[i+1],0,20,yb,yt), TFT_YELLOW);
        }
    }
    else if (currentGraphMode == GRAPH_THERM) {
        int mid = mapFloatToY(60, 20, 100, yb, yt);
        tft.drawLine(xs, mid, xs+GRAPH_WIDTH, mid, TFT_DARKGREY);
        tft.setTextColor(TFT_LIGHTGREY);
        tft.setCursor(xs+2, yt+4);  tft.print("100C");
        tft.setCursor(xs+2, mid-10);tft.print("60C");
        tft.setTextColor(TFT_ORANGE); tft.setCursor(xs+50, yt+4); tft.print("Temp");
        tft.setTextColor(TFT_WHITE);  tft.setCursor(xs+90, yt+4); tft.print("Slope");
        for (int i = 0; i < GRAPH_WIDTH-1; i++) {
            tft.drawLine(xs+i, mapFloatToY(hist_temp[i], 20,100,yb,yt),    xs+i+1, mapFloatToY(hist_temp[i+1], 20,100,yb,yt),    TFT_ORANGE);
            tft.drawLine(xs+i, mapFloatToY(hist_slope[i],-0.1f,0.2f,yb,yt),xs+i+1, mapFloatToY(hist_slope[i+1],-0.1f,0.2f,yb,yt),TFT_WHITE);
        }
    }
}