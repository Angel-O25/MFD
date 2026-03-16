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

// --- GLOBAL CALIBRATION VARIABLES ---
float ct_turns_ratio = 2000.0;    
float burden_resistor_ohms = 100.0; 
float adc_voltage_ref = 3.3;
const float CURRENT_NOISE_FLOOR = 0.7;   

// --- INDEPENDENT CALIBRATION SCALARS ---
// These factors account for MCT-10 variance and wire resistance
float CAL_FACTOR_L1 = 20.45; // Example: Amps per Volt measured at pin
float CAL_FACTOR_L2 = 20.12; 
float CAL_FACTOR_L3 = 20.88;

float adc_voltage_ref = 3.3; // Measured voltage at ESP32 3V3 pin

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

// --- EXTRACTED FEATURES ---
volatile float feature_rms_L1 = 0.0;
volatile float feature_rms_L2 = 0.0;
volatile float feature_rms_L3 = 0.0;
volatile float feature_rms_vibX = 0.0; 
volatile float feature_rms_vibY = 0.0; 
volatile float feature_rms_vibZ = 0.0; 
volatile float feature_rms_vibMag = 0.0;  
volatile float feature_kurt_vibMag = 0.0; 
volatile float current_temp_c = 0.0;

// NEW: Temperature Rate of Change Variables
volatile float feature_temp_slope = 0.0;
volatile float prev_temp_c = 0.0; 

// --- GRAPHING HISTORICAL BUFFERS ---
#define GRAPH_WIDTH 230
volatile float hist_L1[GRAPH_WIDTH], hist_L2[GRAPH_WIDTH], hist_L3[GRAPH_WIDTH];
volatile float hist_vX[GRAPH_WIDTH], hist_vY[GRAPH_WIDTH], hist_vZ[GRAPH_WIDTH];
volatile float hist_temp[GRAPH_WIDTH], hist_slope[GRAPH_WIDTH];
volatile bool update_graph_display = false; // Triggers UI redraw

// --- STATE MACHINES ---
enum PhaseType { DETECTING, OFFLINE, SINGLE_PHASE, THREE_PHASE };
volatile PhaseType currentPhase = DETECTING;

enum TabState { HOME_TAB, GRAPH_TAB, NUMBER_TAB };
TabState currentTab = HOME_TAB;
bool forceUIUpdate = true; 

// NEW: Graph Sub-Menu State
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
float calculateRMS(volatile uint16_t* buffer, int length);
float calculateRMS_Float(volatile float* buffer, int length);
float calculateKurtosis_Float(volatile float* buffer, int length);
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

    // Initialize history arrays to zero
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
    // Convert ADC ticks to Volts, then apply the unique Calibration Factor
    float rms_voltage = (rms_adc / 4095.0) * adc_voltage_ref; 
    return rms_voltage * calFactor; 
}

float calculateRMS_Float(volatile float* buffer, int length) {
    double sum_raw = 0;
    for (int i = 0; i < length; i++) sum_raw += buffer[i];
    double mean = sum_raw / length; 

    double sum_sq = 0;
    for (int i = 0; i < length; i++) {
        double centered_val = buffer[i] - mean;
        sum_sq += (centered_val * centered_val);
    }
    return sqrt(sum_sq / length);
}

float calculateKurtosis_Float(volatile float* buffer, int length) {
    double sum_raw = 0;
    for (int i = 0; i < length; i++) sum_raw += buffer[i];
    double mean = sum_raw / length;

    double sum_sq = 0, sum_quad = 0; 
    for (int i = 0; i < length; i++) {
        double centered_val = buffer[i] - mean;
        double squared = centered_val * centered_val;
        sum_sq += squared;
        sum_quad += (squared * squared);
    }
    
    double variance = sum_sq / length;
    if (variance < 0.5) return 3.00; // Noise Gate
    return (sum_quad / length) / (variance * variance);
}

void detectPhaseType() {
    if (feature_rms_L1 > CURRENT_NOISE_FLOOR && feature_rms_L2 > CURRENT_NOISE_FLOOR && feature_rms_L3 > CURRENT_NOISE_FLOOR) {
        currentPhase = THREE_PHASE;
    } else if (feature_rms_L1 > CURRENT_NOISE_FLOOR && feature_rms_L2 <= CURRENT_NOISE_FLOOR && feature_rms_L3 <= CURRENT_NOISE_FLOOR) {
        currentPhase = SINGLE_PHASE;
    } else if (feature_rms_L1 <= CURRENT_NOISE_FLOOR && feature_rms_L2 <= CURRENT_NOISE_FLOOR && feature_rms_L3 <= CURRENT_NOISE_FLOOR) {
        currentPhase = OFFLINE;
    }
}

// Map a float value to the Graph's Y-pixel coordinates
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

            // 1. Calculate Temperature Slope (Degrees per second)
            if (prev_temp_c != 0.0) {
                feature_temp_slope = current_temp_c - prev_temp_c;
            }
            prev_temp_c = current_temp_c;

            // 2. Extract Features
            feature_rms_L1 = calculateRMS(buffer_L1, SAMPLE_RATE_HZ);
            feature_rms_L2 = calculateRMS(buffer_L2, SAMPLE_RATE_HZ);
            feature_rms_L3 = calculateRMS(buffer_L3, SAMPLE_RATE_HZ);
            
            feature_rms_vibX = calculateRMS_Float(buffer_vibX, SAMPLE_RATE_HZ); 
            feature_rms_vibY = calculateRMS_Float(buffer_vibY, SAMPLE_RATE_HZ); 
            feature_rms_vibZ = calculateRMS_Float(buffer_vibZ, SAMPLE_RATE_HZ); 
            
            detectPhaseType();

            // 3. Shift Historical Arrays for Graphing
            for(int i = 0; i < GRAPH_WIDTH - 1; i++) {
                hist_L1[i] = hist_L1[i+1];
                hist_L2[i] = hist_L2[i+1];
                hist_L3[i] = hist_L3[i+1];
                hist_vX[i] = hist_vX[i+1];
                hist_vY[i] = hist_vY[i+1];
                hist_vZ[i] = hist_vZ[i+1];
                hist_temp[i] = hist_temp[i+1];
                hist_slope[i] = hist_slope[i+1];
            }
            // Append newest data to the end of the arrays
            hist_L1[GRAPH_WIDTH - 1] = feature_rms_L1;
            hist_L2[GRAPH_WIDTH - 1] = feature_rms_L2;
            hist_L3[GRAPH_WIDTH - 1] = feature_rms_L3;
            hist_vX[GRAPH_WIDTH - 1] = feature_rms_vibX;
            hist_vY[GRAPH_WIDTH - 1] = feature_rms_vibY;
            hist_vZ[GRAPH_WIDTH - 1] = feature_rms_vibZ;
            hist_temp[GRAPH_WIDTH - 1] = current_temp_c;
            hist_slope[GRAPH_WIDTH - 1] = feature_temp_slope;

            update_graph_display = true; // Signal UI to redraw the graph lines

            // 4. CSV Logging (Updated to include Temp Slope)
            Serial.print(millis()); Serial.print(", ");
            Serial.print(feature_rms_L1); Serial.print(", ");
            Serial.print(feature_rms_L2); Serial.print(", ");
            Serial.print(feature_rms_L3); Serial.print(", ");
            Serial.print(feature_rms_vibX); Serial.print(", ");
            Serial.print(feature_rms_vibY); Serial.print(", ");
            Serial.print(feature_rms_vibZ); Serial.print(", ");
            Serial.print(current_temp_c); Serial.print(", ");
            Serial.println(feature_temp_slope); // NEW

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

            // Bottom Navigation Tabs
            if (y > 270) { 
                if (x < 80 && currentTab != HOME_TAB) { currentTab = HOME_TAB; forceUIUpdate = true; }
                else if (x >= 80 && x < 160 && currentTab != GRAPH_TAB) { currentTab = GRAPH_TAB; forceUIUpdate = true; }
                else if (x >= 160 && currentTab != NUMBER_TAB) { currentTab = NUMBER_TAB; forceUIUpdate = true; }
                vTaskDelay(pdMS_TO_TICKS(200)); 
            }
            // NEW: Top Sub-Navigation ONLY when in GRAPH_TAB
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

        // Only redraw graph lines exactly when new data arrives (every 1 sec)
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
        // 1. Motor Type (White Circle)
        tft.fillCircle(15, 22, 5, TFT_WHITE);
        tft.setTextColor(TFT_WHITE);
        tft.setCursor(30, 15); tft.print("Motor Type:");
        
        // 2. Status (Orange Triangle pointing up)
        tft.fillTriangle(15, 80, 10, 90, 20, 90, TFT_ORANGE); 
        tft.setTextColor(TFT_ORANGE);
        tft.setCursor(30, 80); tft.print("Status:");
        
        // 3. Health Index (Red Heart - Centered on X=15 to match other icons)
        tft.fillCircle(11, 149, 4, TFT_RED); 
        tft.fillCircle(19, 149, 4, TFT_RED); 
        tft.fillTriangle(7, 150, 23, 150, 15, 159, TFT_RED);
        tft.setTextColor(TFT_RED); 
        tft.setCursor(30, 145); tft.print("Health Idx:");
        
        // Draw the empty Bar Graph Frame
        tft.drawRect(10, 165, 220, 20, TFT_WHITE); 
        
        // 4. Time to Fault (Magenta Clock)
        tft.setTextColor(TFT_MAGENTA); 
        tft.drawCircle(15, 217, 7, TFT_MAGENTA); 
        tft.drawLine(15, 217, 15, 212, TFT_MAGENTA); // Hour hand
        tft.drawLine(15, 217, 19, 217, TFT_MAGENTA); // Minute hand
        tft.setCursor(30, 210); tft.print("Time/Fault:");

    } 
    else if (currentTab == NUMBER_TAB) {
        tft.setCursor(10, 15);  tft.print("L1 RMS(A):");
        tft.setCursor(10, 50);  tft.print("L2 RMS(A):");
        tft.setCursor(10, 85);  tft.print("L3 RMS(A):");
        tft.setCursor(10, 120); tft.print("VibX RMS :");
        tft.setCursor(10, 155); tft.print("VibY RMS :");
        tft.setCursor(10, 190); tft.print("VibZ RMS :");
        tft.setCursor(10, 225); tft.print("Temp(C)  :");
        tft.setCursor(10, 260); tft.print("Slope    :"); 
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
        // Motor Phase Auto-Detect
        tft.setCursor(30, 35); // Aligned directly under the title
        tft.setTextColor(TFT_CYAN, TFT_BLACK); 
        if (currentPhase == DETECTING) tft.print("Detecting... "); 
        else if (currentPhase == OFFLINE) tft.print("Offline      ");
        else if (currentPhase == SINGLE_PHASE) tft.print("1-Phase      ");
        else if (currentPhase == THREE_PHASE) tft.print("3-Phase      ");

        // Status
        tft.setCursor(30, 100); // Aligned directly under the title
        tft.setTextColor(TFT_ORANGE, TFT_BLACK); 
        tft.print("DATA GATHERING"); 

        // Bar Graph Fill 
        tft.fillRect(11, 166, 218, 18, TFT_BLACK); 
        tft.setTextSize(1);
        tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
        tft.drawCentreString("N/A (Collecting Data)", 120, 168, 2);
        tft.setTextSize(2);

        // Time to Fault
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(30, 230); tft.print(">30 Days "); // Aligned directly under the title
    } 
    else if (currentTab == NUMBER_TAB) {
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setCursor(140, 15);  tft.print(feature_rms_L1, 2);   tft.print("  ");
        tft.setCursor(140, 50);  tft.print(feature_rms_L2, 2);   tft.print("  ");
        tft.setCursor(140, 85);  tft.print(feature_rms_L3, 2);   tft.print("  ");
        tft.setCursor(140, 120); tft.print(feature_rms_vibX, 2); tft.print("  ");
        tft.setCursor(140, 155); tft.print(feature_rms_vibY, 2); tft.print("  "); 
        tft.setCursor(140, 190); tft.print(feature_rms_vibZ, 2); tft.print("  "); 
        tft.setCursor(140, 225); tft.print(current_temp_c, 1);   tft.print("  ");
        tft.setCursor(140, 260); tft.print(feature_temp_slope, 2); tft.print("  "); 
    }
}


void drawGraphContent() {
    // Graph plotting boundaries (Maximizing the space)
    int x_start = 5;
    int y_top = 35;
    int y_bottom = 275;

    // Clear previous lines inside the graph box
    tft.fillRect(x_start, y_top, GRAPH_WIDTH, y_bottom - y_top, TFT_BLACK);

    tft.setTextSize(1);

    if (currentGraphMode == GRAPH_ELEC) {
        // --- ELECTRICAL CALIBRATED SINE WAVE GRAPH ---
        
        int mid_y = y_top + (y_bottom - y_top) / 2;
        tft.drawLine(x_start, mid_y, x_start + GRAPH_WIDTH, mid_y, TFT_DARKGREY);
        
        tft.setTextColor(TFT_LIGHTGREY);
        tft.setCursor(x_start + 2, y_top + 4); tft.print("+15A"); 
        tft.setCursor(x_start + 2, mid_y - 10); tft.print("0A");
        tft.setCursor(x_start + 2, y_bottom - 12); tft.print("-15A");
        
        tft.setTextColor(TFT_RED);   tft.setCursor(x_start + 70, y_top + 4); tft.print("L1");
        tft.setTextColor(TFT_GREEN); tft.setCursor(x_start + 100, y_top + 4); tft.print("L2");
        tft.setTextColor(TFT_BLUE);  tft.setCursor(x_start + 130, y_top + 4); tft.print("L3");

        // 1. Calculate DC Offsets for this frame (Hardware bias removal)
        long s1 = 0, s2 = 0, s3 = 0;
        for (int i = 0; i < GRAPH_WIDTH; i++) {
            s1 += buffer_L1[i]; s2 += buffer_L2[i]; s3 += buffer_L3[i];
        }
        float off1 = (float)s1 / GRAPH_WIDTH;
        float off2 = (float)s2 / GRAPH_WIDTH;
        float off3 = (float)s3 / GRAPH_WIDTH;

        // 2. Plot lines using Calibrated Amp values
        for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
            // Function to convert raw ADC tick to real Amps
            auto getAmps = [&](float raw, float offset) {
                float voltage = ((raw - offset) / 4095.0) * adc_voltage_ref;
                return (voltage / burden_resistor_ohms) * ct_turns_ratio;
            };

            float amps1_start = getAmps(buffer_L1[i], off1);
            float amps1_end   = getAmps(buffer_L1[i+1], off1);
            
            float amps2_start = getAmps(buffer_L2[i], off2);
            float amps2_end   = getAmps(buffer_L2[i+1], off2);
            
            float amps3_start = getAmps(buffer_L3[i], off3);
            float amps3_end   = getAmps(buffer_L3[i+1], off3);

            // Map Amps (-15 to +15) to Y-pixels
            tft.drawLine(x_start + i, mapFloatToY(amps1_start, -15, 15, y_bottom, y_top), 
                         x_start + i + 1, mapFloatToY(amps1_end, -15, 15, y_bottom, y_top), TFT_RED);
            
            tft.drawLine(x_start + i, mapFloatToY(amps2_start, -15, 15, y_bottom, y_top), 
                         x_start + i + 1, mapFloatToY(amps2_end, -15, 15, y_bottom, y_top), TFT_GREEN);
            
            tft.drawLine(x_start + i, mapFloatToY(amps3_start, -15, 15, y_bottom, y_top), 
                         x_start + i + 1, mapFloatToY(amps3_end, -15, 15, y_bottom, y_top), TFT_BLUE);
        }
    }
    else if (currentGraphMode == GRAPH_MECH) {
        // --- MECHANICAL GRAPH (0 to 20 m/s2 scale) ---
        int mid_y = mapFloatToY(10, 0, 20, y_bottom, y_top);
        tft.drawLine(x_start, mid_y, x_start + GRAPH_WIDTH, mid_y, TFT_DARKGREY);
        
        tft.setTextColor(TFT_LIGHTGREY);
        tft.setCursor(x_start + 2, y_top + 4); tft.print("20ms2");
        tft.setCursor(x_start + 2, mid_y - 10); tft.print("10");

        tft.setTextColor(TFT_CYAN);    tft.setCursor(x_start + 50, y_top + 4); tft.print("vX");
        tft.setTextColor(TFT_MAGENTA); tft.setCursor(x_start + 80, y_top + 4); tft.print("vY");
        tft.setTextColor(TFT_YELLOW);  tft.setCursor(x_start + 110, y_top + 4); tft.print("vZ");

        for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
            tft.drawLine(x_start + i, mapFloatToY(hist_vX[i], 0, 20, y_bottom, y_top),
                         x_start + i + 1, mapFloatToY(hist_vX[i+1], 0, 20, y_bottom, y_top), TFT_CYAN);
            tft.drawLine(x_start + i, mapFloatToY(hist_vY[i], 0, 20, y_bottom, y_top),
                         x_start + i + 1, mapFloatToY(hist_vY[i+1], 0, 20, y_bottom, y_top), TFT_MAGENTA);
            tft.drawLine(x_start + i, mapFloatToY(hist_vZ[i], 0, 20, y_bottom, y_top),
                         x_start + i + 1, mapFloatToY(hist_vZ[i+1], 0, 20, y_bottom, y_top), TFT_YELLOW);
        }
    } 
    else if (currentGraphMode == GRAPH_THERM) {
        // --- THERMAL GRAPH (Temp: 20C to 100C) ---
        int mid_y = mapFloatToY(60, 20, 100, y_bottom, y_top);
        tft.drawLine(x_start, mid_y, x_start + GRAPH_WIDTH, mid_y, TFT_DARKGREY);
        
        tft.setTextColor(TFT_LIGHTGREY);
        tft.setCursor(x_start + 2, y_top + 4); tft.print("100C");
        tft.setCursor(x_start + 2, mid_y - 10); tft.print("60C");

        tft.setTextColor(TFT_ORANGE); tft.setCursor(x_start + 50, y_top + 4); tft.print("Temp");
        tft.setTextColor(TFT_WHITE);  tft.setCursor(x_start + 90, y_top + 4); tft.print("Slope");

        for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
            tft.drawLine(x_start + i, mapFloatToY(hist_temp[i], 20, 100, y_bottom, y_top),
                         x_start + i + 1, mapFloatToY(hist_temp[i+1], 20, 100, y_bottom, y_top), TFT_ORANGE);
            // Notice: Slope scale is mapped -2 to 5 so it overlays nicely
            tft.drawLine(x_start + i, mapFloatToY(hist_slope[i], -2, 5, y_bottom, y_top),
                         x_start + i + 1, mapFloatToY(hist_slope[i+1], -2, 5, y_bottom, y_top), TFT_WHITE);
        }
    }
}