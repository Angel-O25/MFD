#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MCP9808.h>
#include <math.h>

// --- PIN DEFINITIONS ---
#define PIN_CT_L1 36
#define PIN_CT_L2 39
#define PIN_CT_L3 34
#define I2C_SDA 21
#define I2C_SCL 22

//===============================================================//
//  FOR CALIBRATION                                              //
//===============================================================//
// --- INDEPENDENT CALIBRATION SCALARS ---                       //
float CAL_FACTOR_L1 = 1.45; // Adjust during calibration         //
float CAL_FACTOR_L2 = 0.12;                                      //
float CAL_FACTOR_L3 = 5.88;                                      //
float adc_voltage_ref = 3.3;                                     //
//===============================================================//
// --- NOISE GATES ---                                           //
const float CURRENT_NOISE_GATE = 0.2; // Amps                    //
const float VIB_NOISE_GATE = 0.4;     // m/s^2                   //
//===============================================================//

// --- HARDWARE OBJECTS ---
TFT_eSPI tft = TFT_eSPI(); 
Adafruit_MPU6050 mpu;
Adafruit_MCP9808 mcp;

// --- DATA BUFFERS ---
#define SAMPLE_RATE_HZ 1000
volatile uint16_t buffer_L1[SAMPLE_RATE_HZ];
volatile uint16_t buffer_L2[SAMPLE_RATE_HZ];
volatile uint16_t buffer_L3[SAMPLE_RATE_HZ];
volatile float buffer_vibX[SAMPLE_RATE_HZ];
volatile float buffer_vibY[SAMPLE_RATE_HZ]; 
volatile float buffer_vibZ[SAMPLE_RATE_HZ]; 

volatile int sample_index = 0;
volatile bool buffer_ready = false;

// --- EXTRACTED FEATURES FOR ML & UI ---
volatile float feature_rms_L1 = 0.0;
volatile float feature_rms_L2 = 0.0;
volatile float feature_rms_L3 = 0.0;
volatile float feature_current_unbalance = 0.0;

volatile float feature_rms_vibX = 0.0, feature_kurt_vibX = 0.0, feature_crest_vibX = 0.0; 
volatile float feature_rms_vibY = 0.0, feature_kurt_vibY = 0.0, feature_crest_vibY = 0.0; 
volatile float feature_rms_vibZ = 0.0, feature_kurt_vibZ = 0.0, feature_crest_vibZ = 0.0; 

volatile float current_temp_c = 0.0;
volatile float ema_temp = 25.0; // Smoothed Temperature
volatile float feature_temp_slope = 0.0;
volatile float prev_temp_c = 0.0; 

// --- GRAPHING HISTORICAL BUFFERS ---
#define GRAPH_WIDTH 230
volatile float hist_L1[GRAPH_WIDTH], hist_L2[GRAPH_WIDTH], hist_L3[GRAPH_WIDTH];
volatile float hist_vX[GRAPH_WIDTH], hist_vY[GRAPH_WIDTH], hist_vZ[GRAPH_WIDTH];
volatile float hist_temp[GRAPH_WIDTH], hist_slope[GRAPH_WIDTH];
volatile bool update_graph_display = false;

// --- STATE MACHINES ---
enum PhaseType { DETECTING, OFFLINE, SINGLE_PHASE, THREE_PHASE };
volatile PhaseType currentPhase = DETECTING;

enum TabState { HOME_TAB, GRAPH_TAB, NUMBER_TAB };
TabState currentTab = HOME_TAB;
bool forceUIUpdate = true; 

enum GraphMode { GRAPH_ELEC, GRAPH_MECH, GRAPH_THERM };
GraphMode currentGraphMode = GRAPH_ELEC;

// --- FREERTOS TASK HANDLES & MUTEX ---
TaskHandle_t TaskSensorRead;
TaskHandle_t TaskProcessData;
TaskHandle_t TaskUI;
SemaphoreHandle_t i2cMutex; 

// --- FUNCTION PROTOTYPES ---
void sensorReadTask(void *pvParameters);
void processDataTask(void *pvParameters);
void uiTask(void *pvParameters);
void drawTabs();
void drawStaticContent();
void drawDynamicContent();
void drawGraphContent();
float calculateRMS(volatile uint16_t* buffer, int length, float calFactor); 
float calculateUnbalance(float l1, float l2, float l3);
void calculateVibFeatures(volatile float* buffer, int length, volatile float &rms, volatile float &kurtosis, volatile float &crest);
void detectPhaseType();
int mapFloatToY(float value, float min_val, float max_val, int y_bottom, int y_top);

// ==========================================
// SETUP
// ==========================================
void setup() {
    Serial.begin(115200);
    
    i2cMutex = xSemaphoreCreateMutex(); 

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); 
    Wire.setTimeOut(20); 

    analogReadResolution(12);

    if (!mpu.begin()) Serial.println("MPU6050 Error");
    delay(200); 
    if (!mcp.begin()) Serial.println("MCP9808 Error");
    
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ); 

    tft.init();
    tft.setRotation(2); 
    
    uint16_t calData[5] = { 275, 3620, 264, 3532, 1 };
    tft.setTouch(calData);
    tft.fillScreen(TFT_BLACK);

    for(int i=0; i<GRAPH_WIDTH; i++) {
        hist_L1[i] = 0; hist_L2[i] = 0; hist_L3[i] = 0;
        hist_vX[i] = 0; hist_vY[i] = 0; hist_vZ[i] = 0;
        hist_temp[i] = 0; hist_slope[i] = 0;
    }

    xTaskCreatePinnedToCore(sensorReadTask, "Sensor", 4096, NULL, 3, &TaskSensorRead, 1);
    xTaskCreatePinnedToCore(processDataTask, "Process", 8192, NULL, 2, &TaskProcessData, 1);
    xTaskCreatePinnedToCore(uiTask, "UI", 8192, NULL, 1, &TaskUI, 0);
}

void loop() { vTaskDelete(NULL); }

// ==========================================
// MATH & LOGIC
// ==========================================
float calculateRMS(volatile uint16_t* buffer, int length, float calFactor) {
    double sum_raw = 0;
    for (int i = 0; i < length; i++) sum_raw += buffer[i];
    double dynamic_dc_offset = sum_raw / length;

    double sum_sq = 0;
    for (int i = 0; i < length; i++) {
        double centered_val = buffer[i] - dynamic_dc_offset;
        sum_sq += (centered_val * centered_val);
    }
    
    float rms_adc = sqrt(sum_sq / length);
    float amps = ((rms_adc / 4095.0) * adc_voltage_ref) * calFactor; 
    
    if (amps < CURRENT_NOISE_GATE) return 0.0;
    return amps;
}

float calculateUnbalance(float l1, float l2, float l3) {
    if (l1 == 0 && l2 == 0 && l3 == 0) return 0.0;
    float avg = (l1 + l2 + l3) / 3.0;
    float max_diff = max(abs(l1 - avg), max(abs(l2 - avg), abs(l3 - avg)));
    return (max_diff / avg) * 100.0;
}

void calculateVibFeatures(volatile float* buffer, int length, volatile float &rms, volatile float &kurtosis, volatile float &crest) {
    float sum = 0;
    for(int i = 0; i < length; i++) sum += buffer[i];
    float mean = sum / length;

    float sum_sq_diff = 0, sum_quad_diff = 0, max_peak = 0;

    for(int i = 0; i < length; i++) {
        float diff = buffer[i] - mean;
        float abs_diff = abs(diff);
        float sq_diff = diff * diff;
        
        sum_sq_diff += sq_diff;
        sum_quad_diff += (sq_diff * sq_diff);
        if(abs_diff > max_peak) max_peak = abs_diff;
    }

    float variance = sum_sq_diff / length;
    rms = sqrt(variance);

    if (rms < VIB_NOISE_GATE) {
        rms = 0.0; kurtosis = 0.0; crest = 0.0;
        return;
    }

    kurtosis = (variance > 0.0001) ? ((sum_quad_diff / length) / (variance * variance)) : 3.0;
    crest = (rms > 0.0001) ? (max_peak / rms) : 1.0;
}

void detectPhaseType() {
    if (feature_rms_L1 > CURRENT_NOISE_GATE && feature_rms_L2 > CURRENT_NOISE_GATE && feature_rms_L3 > CURRENT_NOISE_GATE) {
        currentPhase = THREE_PHASE;
    } else if (feature_rms_L1 > CURRENT_NOISE_GATE && feature_rms_L2 <= CURRENT_NOISE_GATE && feature_rms_L3 <= CURRENT_NOISE_GATE) {
        currentPhase = SINGLE_PHASE;
    } else if (feature_rms_L1 <= CURRENT_NOISE_GATE && feature_rms_L2 <= CURRENT_NOISE_GATE && feature_rms_L3 <= CURRENT_NOISE_GATE) {
        currentPhase = OFFLINE;
    }
}

int mapFloatToY(float value, float min_val, float max_val, int y_bottom, int y_top) {
    if (value <= min_val) return y_bottom;
    if (value >= max_val) return y_top;
    float ratio = (value - min_val) / (max_val - min_val);
    return y_bottom - (int)(ratio * (y_bottom - y_top));
}

// ==========================================
// TASK: 1 kHz Sensor Polling (Core 1)
// ==========================================
void sensorReadTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    bool mpu_tick = false;

    for (;;) {
        if (!buffer_ready) {
            buffer_L1[sample_index] = analogRead(PIN_CT_L1);
            buffer_L2[sample_index] = analogRead(PIN_CT_L2);
            buffer_L3[sample_index] = analogRead(PIN_CT_L3);
            
            if (mpu_tick) {
                if (xSemaphoreTake(i2cMutex, 0) == pdTRUE) {
                    sensors_event_t a, g, temp;
                    mpu.getEvent(&a, &g, &temp);
                    buffer_vibX[sample_index] = a.acceleration.x;
                    buffer_vibY[sample_index] = a.acceleration.y; 
                    buffer_vibZ[sample_index] = a.acceleration.z; 
                    xSemaphoreGive(i2cMutex); 
                } else if (sample_index > 0) {
                    buffer_vibX[sample_index] = buffer_vibX[sample_index - 1]; 
                    buffer_vibY[sample_index] = buffer_vibY[sample_index - 1]; 
                    buffer_vibZ[sample_index] = buffer_vibZ[sample_index - 1]; 
                }
            } else if (sample_index > 0) {
                 buffer_vibX[sample_index] = buffer_vibX[sample_index - 1];
                 buffer_vibY[sample_index] = buffer_vibY[sample_index - 1]; 
                 buffer_vibZ[sample_index] = buffer_vibZ[sample_index - 1]; 
            }
            
            mpu_tick = !mpu_tick;
            sample_index++;
            
            if (sample_index >= SAMPLE_RATE_HZ) {
                sample_index = 0;
                buffer_ready = true; 
            }
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1)); 
    }
}

// ==========================================
// TASK: 1-Second Processing & Logging (Core 1)
// ==========================================
void processDataTask(void *pvParameters) {
    for (;;) {
        if (buffer_ready) {
            if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
                current_temp_c = mcp.readTempC();
                xSemaphoreGive(i2cMutex);
            }

            // Temperature EMA Smoothing & Slope
            ema_temp = (0.1 * current_temp_c) + (0.9 * ema_temp);
            if (prev_temp_c != 0.0) feature_temp_slope = ema_temp - prev_temp_c;
            prev_temp_c = ema_temp;

            // Extract Electrical Features
            feature_rms_L1 = calculateRMS(buffer_L1, SAMPLE_RATE_HZ, CAL_FACTOR_L1);
            feature_rms_L2 = calculateRMS(buffer_L2, SAMPLE_RATE_HZ, CAL_FACTOR_L2);
            feature_rms_L3 = calculateRMS(buffer_L3, SAMPLE_RATE_HZ, CAL_FACTOR_L3);
            feature_current_unbalance = calculateUnbalance(feature_rms_L1, feature_rms_L2, feature_rms_L3);
            
            // Extract Mechanical Features
            calculateVibFeatures(buffer_vibX, SAMPLE_RATE_HZ, feature_rms_vibX, feature_kurt_vibX, feature_crest_vibX);
            calculateVibFeatures(buffer_vibY, SAMPLE_RATE_HZ, feature_rms_vibY, feature_kurt_vibY, feature_crest_vibY);
            calculateVibFeatures(buffer_vibZ, SAMPLE_RATE_HZ, feature_rms_vibZ, feature_kurt_vibZ, feature_crest_vibZ);
            
            detectPhaseType();

            // Shift Historical Arrays
            for(int i = 0; i < GRAPH_WIDTH - 1; i++) {
                hist_L1[i] = hist_L1[i+1]; hist_L2[i] = hist_L2[i+1]; hist_L3[i] = hist_L3[i+1];
                hist_vX[i] = hist_vX[i+1]; hist_vY[i] = hist_vY[i+1]; hist_vZ[i] = hist_vZ[i+1];
                hist_temp[i] = hist_temp[i+1]; hist_slope[i] = hist_slope[i+1];
            }
            hist_L1[GRAPH_WIDTH - 1] = feature_rms_L1;
            hist_L2[GRAPH_WIDTH - 1] = feature_rms_L2;
            hist_L3[GRAPH_WIDTH - 1] = feature_rms_L3;
            hist_vX[GRAPH_WIDTH - 1] = feature_rms_vibX;
            hist_vY[GRAPH_WIDTH - 1] = feature_rms_vibY;
            hist_vZ[GRAPH_WIDTH - 1] = feature_rms_vibZ;
            hist_temp[GRAPH_WIDTH - 1] = ema_temp;
            hist_slope[GRAPH_WIDTH - 1] = feature_temp_slope;

            update_graph_display = true;

            // NEW: 15-Point CSV Logging string
            Serial.print(millis()); Serial.print(",");
            Serial.print(feature_rms_L1); Serial.print(",");
            Serial.print(feature_rms_L2); Serial.print(",");
            Serial.print(feature_rms_L3); Serial.print(",");
            Serial.print(feature_current_unbalance); Serial.print(",");
            
            Serial.print(feature_rms_vibX); Serial.print(",");
            Serial.print(feature_rms_vibY); Serial.print(",");
            Serial.print(feature_rms_vibZ); Serial.print(",");
            
            Serial.print(feature_kurt_vibX); Serial.print(",");
            Serial.print(feature_kurt_vibY); Serial.print(",");
            Serial.print(feature_kurt_vibZ); Serial.print(",");
            
            Serial.print(feature_crest_vibX); Serial.print(",");
            Serial.print(feature_crest_vibY); Serial.print(",");
            Serial.print(feature_crest_vibZ); Serial.print(",");
            
            Serial.print(ema_temp); Serial.print(",");
            Serial.println(feature_temp_slope);

            buffer_ready = false; 
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

// ==========================================
// TASK: TFT UI & Touch Handling (Core 0)
// ==========================================
void uiTask(void *pvParameters) {
    forceUIUpdate = true; 
    for (;;) {
        uint16_t raw_x, raw_y;
        bool pressed = tft.getTouch(&raw_x, &raw_y);

        if (pressed) {
            uint16_t x = map(raw_y, 320, 0, 0, 240); 
            uint16_t y = map(raw_x, 0, 240, 0, 320);

            if (y > 270) { 
                if (x < 80 && currentTab != HOME_TAB) { currentTab = HOME_TAB; forceUIUpdate = true; }
                else if (x >= 80 && x < 160 && currentTab != GRAPH_TAB) { currentTab = GRAPH_TAB; forceUIUpdate = true; }
                else if (x >= 160 && currentTab != NUMBER_TAB) { currentTab = NUMBER_TAB; forceUIUpdate = true; }
                vTaskDelay(pdMS_TO_TICKS(200)); 
            }
            else if (currentTab == GRAPH_TAB && y < 30) {
                if (x < 80) currentGraphMode = GRAPH_ELEC;
                else if (x >= 80 && x < 160) currentGraphMode = GRAPH_MECH;
                else if (x >= 160) currentGraphMode = GRAPH_THERM;
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
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

// ==========================================
// UI DRAWING FUNCTIONS 
// ==========================================
void drawTabs() {
    tft.fillRect(0, 280, 80, 40, currentTab == HOME_TAB ? TFT_BLUE : TFT_DARKGREY);
    tft.fillRect(80, 280, 80, 40, currentTab == GRAPH_TAB ? TFT_BLUE : TFT_DARKGREY);
    tft.fillRect(160, 280, 80, 40, currentTab == NUMBER_TAB ? TFT_BLUE : TFT_DARKGREY);
    
    tft.setTextColor(TFT_WHITE); 
    tft.setTextSize(1); 
    tft.drawCentreString("HOME", 40, 290, 2);
    tft.drawCentreString("GRAPH", 120, 290, 2);
    tft.drawCentreString("NUMS", 200, 290, 2);
}

void drawStaticContent() {
    tft.setTextColor(TFT_WHITE); 
    tft.setTextSize(2);

    if (currentTab == HOME_TAB) {
        tft.fillCircle(15, 22, 5, TFT_WHITE);
        tft.setCursor(30, 15); tft.print("Motor Type:");
        
        tft.fillTriangle(15, 80, 10, 90, 20, 90, TFT_ORANGE); 
        tft.setTextColor(TFT_ORANGE); tft.setCursor(30, 80); tft.print("Status:");
        
        tft.fillCircle(11, 149, 4, TFT_RED); tft.fillCircle(19, 149, 4, TFT_RED); 
        tft.fillTriangle(7, 150, 23, 150, 15, 159, TFT_RED);
        tft.setTextColor(TFT_RED); tft.setCursor(30, 145); tft.print("Health Idx:");
        
        tft.drawRect(10, 165, 220, 20, TFT_WHITE); 
        
        tft.setTextColor(TFT_MAGENTA); 
        tft.drawCircle(15, 217, 7, TFT_MAGENTA); 
        tft.drawLine(15, 217, 15, 212, TFT_MAGENTA); tft.drawLine(15, 217, 19, 217, TFT_MAGENTA);
        tft.setCursor(30, 210); tft.print("Time/Fault:");

    } 
    else if (currentTab == NUMBER_TAB) {
        tft.setCursor(10, 15);  tft.print("L1 RMS(A):");
        tft.setCursor(10, 50);  tft.print("L2 RMS(A):");
        tft.setCursor(10, 85);  tft.print("L3 RMS(A):");
        
        // Consolidated UI Features
        tft.setCursor(10, 120); tft.print("Unbal(%):");
        tft.setCursor(10, 155); tft.print("Ovr Vib :");
        tft.setCursor(10, 190); tft.print("Max Kurt:");
        
        tft.setCursor(10, 225); tft.print("Temp(C) :");
        tft.setCursor(10, 260); tft.print("Slope   :"); 
    }
    else if (currentTab == GRAPH_TAB) {
        tft.fillRect(0, 0, 80, 30, currentGraphMode == GRAPH_ELEC ? TFT_MAROON : TFT_BLACK);
        tft.fillRect(80, 0, 80, 30, currentGraphMode == GRAPH_MECH ? TFT_MAROON : TFT_BLACK);
        tft.fillRect(160, 0, 80, 30, currentGraphMode == GRAPH_THERM ? TFT_MAROON : TFT_BLACK);
        
        tft.drawRect(0, 0, 80, 30, TFT_WHITE);
        tft.drawRect(80, 0, 80, 30, TFT_WHITE);
        tft.drawRect(160, 0, 80, 30, TFT_WHITE);

        tft.setTextSize(1);
        tft.drawCentreString("ELEC", 40, 8, 2);
        tft.drawCentreString("MECH", 120, 8, 2);
        tft.drawCentreString("THERM", 200, 8, 2);
        
        tft.drawRect(4, 34, GRAPH_WIDTH + 2, 242, TFT_WHITE); 
        update_graph_display = true; 
    }
}

void drawDynamicContent() {
    tft.setTextSize(2);
    
    if (currentTab == HOME_TAB) {
        tft.setCursor(30, 35); 
        tft.setTextColor(TFT_CYAN, TFT_BLACK); 
        if (currentPhase == DETECTING) tft.print("Detecting... "); 
        else if (currentPhase == OFFLINE) tft.print("Offline      ");
        else if (currentPhase == SINGLE_PHASE) tft.print("1-Phase      ");
        else if (currentPhase == THREE_PHASE) tft.print("3-Phase      ");

        tft.setCursor(30, 100); 
        tft.setTextColor(TFT_ORANGE, TFT_BLACK); tft.print("DATA GATHERING"); 

        tft.fillRect(11, 166, 218, 18, TFT_BLACK); 
        tft.setTextSize(1); tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.drawCentreString("N/A (Collecting Data)", 120, 168, 2);
        
        tft.setTextSize(2); tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(30, 230); tft.print(">30 Days "); 
    } 
    else if (currentTab == NUMBER_TAB) {
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        
        // 1. Electrical
        tft.setCursor(140, 15);  tft.print(feature_rms_L1, 2);   tft.print("  ");
        tft.setCursor(140, 50);  tft.print(feature_rms_L2, 2);   tft.print("  ");
        tft.setCursor(140, 85);  tft.print(feature_rms_L3, 2);   tft.print("  ");
        tft.setCursor(140, 120); tft.print(feature_current_unbalance, 1); tft.print("  ");
        
        // 2. Consolidated Mechanical (Calculated just for the UI)
        float ui_overall_vib = sqrt((feature_rms_vibX * feature_rms_vibX) + 
                                    (feature_rms_vibY * feature_rms_vibY) + 
                                    (feature_rms_vibZ * feature_rms_vibZ));
        
        float ui_max_kurt = max(feature_kurt_vibX, max(feature_kurt_vibY, feature_kurt_vibZ));
        
        tft.setCursor(140, 155); tft.print(ui_overall_vib, 2); tft.print("  "); 
        tft.setCursor(140, 190); tft.print(ui_max_kurt, 1); tft.print("  "); 
        
        // 3. Thermal
        tft.setCursor(140, 225); tft.print(ema_temp, 1);   tft.print("  ");
        tft.setCursor(140, 260); tft.print(feature_temp_slope, 2); tft.print("  "); 
    }
}

void drawGraphContent() {
    int x_start = 5, y_top = 35, y_bottom = 275;
    tft.fillRect(x_start, y_top, GRAPH_WIDTH, y_bottom - y_top, TFT_BLACK);
    tft.setTextSize(1);

    if (currentGraphMode == GRAPH_ELEC) {
        int mid_y = y_top + (y_bottom - y_top) / 2;
        tft.drawLine(x_start, mid_y, x_start + GRAPH_WIDTH, mid_y, TFT_DARKGREY);
        
        tft.setTextColor(TFT_LIGHTGREY);
        tft.setCursor(x_start + 2, y_top + 4); tft.print("+15A"); 
        tft.setCursor(x_start + 2, mid_y - 10); tft.print("0A");
        tft.setCursor(x_start + 2, y_bottom - 12); tft.print("-15A");

        long s1 = 0, s2 = 0, s3 = 0;
        for (int i = 0; i < GRAPH_WIDTH; i++) {
            s1 += buffer_L1[i]; s2 += buffer_L2[i]; s3 += buffer_L3[i];
        }
        float off1 = (float)s1 / GRAPH_WIDTH; float off2 = (float)s2 / GRAPH_WIDTH; float off3 = (float)s3 / GRAPH_WIDTH;

        for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
            auto toAmps = [&](float raw, float offset, float factor) {
                float v_at_pin = ((raw - offset) / 4095.0) * adc_voltage_ref;
                return v_at_pin * factor;
            };

            float a1_s = toAmps(buffer_L1[i], off1, CAL_FACTOR_L1); float a1_e = toAmps(buffer_L1[i+1], off1, CAL_FACTOR_L1);
            float a2_s = toAmps(buffer_L2[i], off2, CAL_FACTOR_L2); float a2_e = toAmps(buffer_L2[i+1], off2, CAL_FACTOR_L2);
            float a3_s = toAmps(buffer_L3[i], off3, CAL_FACTOR_L3); float a3_e = toAmps(buffer_L3[i+1], off3, CAL_FACTOR_L3);

            tft.drawLine(x_start + i, mapFloatToY(a1_s, -15, 15, y_bottom, y_top), x_start + i + 1, mapFloatToY(a1_e, -15, 15, y_bottom, y_top), TFT_RED);
            tft.drawLine(x_start + i, mapFloatToY(a2_s, -15, 15, y_bottom, y_top), x_start + i + 1, mapFloatToY(a2_e, -15, 15, y_bottom, y_top), TFT_GREEN);
            tft.drawLine(x_start + i, mapFloatToY(a3_s, -15, 15, y_bottom, y_top), x_start + i + 1, mapFloatToY(a3_e, -15, 15, y_bottom, y_top), TFT_BLUE);
        }
    }
    else if (currentGraphMode == GRAPH_MECH) {
        int mid_y = mapFloatToY(10, 0, 20, y_bottom, y_top);
        tft.drawLine(x_start, mid_y, x_start + GRAPH_WIDTH, mid_y, TFT_DARKGREY);
        
        tft.setTextColor(TFT_LIGHTGREY);
        tft.setCursor(x_start + 2, y_top + 4); tft.print("20ms2");
        tft.setCursor(x_start + 2, mid_y - 10); tft.print("10");

        tft.setTextColor(TFT_CYAN);    tft.setCursor(x_start + 50, y_top + 4); tft.print("vX");
        tft.setTextColor(TFT_MAGENTA); tft.setCursor(x_start + 80, y_top + 4); tft.print("vY");
        tft.setTextColor(TFT_YELLOW);  tft.setCursor(x_start + 110, y_top + 4); tft.print("vZ");

        for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
            tft.drawLine(x_start + i, mapFloatToY(hist_vX[i], 0, 20, y_bottom, y_top), x_start + i + 1, mapFloatToY(hist_vX[i+1], 0, 20, y_bottom, y_top), TFT_CYAN);
            tft.drawLine(x_start + i, mapFloatToY(hist_vY[i], 0, 20, y_bottom, y_top), x_start + i + 1, mapFloatToY(hist_vY[i+1], 0, 20, y_bottom, y_top), TFT_MAGENTA);
            tft.drawLine(x_start + i, mapFloatToY(hist_vZ[i], 0, 20, y_bottom, y_top), x_start + i + 1, mapFloatToY(hist_vZ[i+1], 0, 20, y_bottom, y_top), TFT_YELLOW);
        }
    } 
    else if (currentGraphMode == GRAPH_THERM) {
        int mid_y = mapFloatToY(60, 20, 100, y_bottom, y_top);
        tft.drawLine(x_start, mid_y, x_start + GRAPH_WIDTH, mid_y, TFT_DARKGREY);
        
        tft.setTextColor(TFT_LIGHTGREY);
        tft.setCursor(x_start + 2, y_top + 4); tft.print("100C");
        tft.setCursor(x_start + 2, mid_y - 10); tft.print("60C");

        tft.setTextColor(TFT_ORANGE); tft.setCursor(x_start + 50, y_top + 4); tft.print("Temp");
        tft.setTextColor(TFT_WHITE);  tft.setCursor(x_start + 90, y_top + 4); tft.print("Slope");

        for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
            tft.drawLine(x_start + i, mapFloatToY(hist_temp[i], 20, 100, y_bottom, y_top), x_start + i + 1, mapFloatToY(hist_temp[i+1], 20, 100, y_bottom, y_top), TFT_ORANGE);
            tft.drawLine(x_start + i, mapFloatToY(hist_slope[i], -2, 5, y_bottom, y_top), x_start + i + 1, mapFloatToY(hist_slope[i+1], -2, 5, y_bottom, y_top), TFT_WHITE);
        }
    }
}