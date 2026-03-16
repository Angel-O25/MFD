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
volatile int sample_index = 0;
volatile bool buffer_ready = false;

// --- EXTRACTED FEATURES ---
volatile float feature_rms_L1 = 0.0;
volatile float feature_rms_L2 = 0.0;
volatile float feature_rms_L3 = 0.0;
volatile float feature_rms_vibX = 0.0; // New Vibration Feature
volatile float current_temp_c = 0.0;

// --- PHASE DETECTION STATE ---
enum PhaseType { DETECTING, OFFLINE, SINGLE_PHASE, THREE_PHASE };
volatile PhaseType currentPhase = DETECTING;
const float CURRENT_NOISE_FLOOR = 0.15; 

// --- UI STATE MACHINE ---
enum TabState { HOME_TAB, GRAPH_TAB, NUMBER_TAB };
TabState currentTab = HOME_TAB;
bool forceUIUpdate = true; 

// --- FREERTOS TASK HANDLES ---
TaskHandle_t TaskSensorRead;
TaskHandle_t TaskProcessData;
TaskHandle_t TaskUI;

// --- FUNCTION PROTOTYPES ---
void sensorReadTask(void *pvParameters);
void processDataTask(void *pvParameters);
void uiTask(void *pvParameters);
void drawTabs();
void drawStaticContent();
void drawDynamicContent();
float calculateRMS(volatile uint16_t* buffer, int length);
float calculateRMS_Float(volatile float* buffer, int length);
void detectPhaseType();

// ==========================================
// SETUP
// ==========================================
void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); 

    analogReadResolution(12);

    // MPU6050 Robust Initialization
    bool mpu_found = mpu.begin(0x68, &Wire); // Try default address
    if (!mpu_found) {
        mpu_found = mpu.begin(0x69, &Wire);  // Try alternate address
    }
    if (!mpu_found) {
        Serial.println("MPU6050 Error: Not found at 0x68 or 0x69");
    } else {
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setFilterBandwidth(MPU6050_BAND_260_HZ); 
    }

    if (!mcp.begin()) Serial.println("MCP9808 Error");
    
    tft.init();
    tft.setRotation(1); 
    
    uint16_t calData[5] = { 275, 3620, 264, 3532, 1 };
    tft.setTouch(calData);
    tft.fillScreen(TFT_BLACK);

    xTaskCreatePinnedToCore(sensorReadTask, "Sensor", 4096, NULL, 3, &TaskSensorRead, 1);
    xTaskCreatePinnedToCore(processDataTask, "Process", 8192, NULL, 2, &TaskProcessData, 1);
    xTaskCreatePinnedToCore(uiTask, "UI", 8192, NULL, 1, &TaskUI, 0);
}

void loop() { vTaskDelete(NULL); }

// ==========================================
// MATH & LOGIC
// ==========================================
float calculateRMS(volatile uint16_t* buffer, int length) {
    double sum_raw = 0;
    for (int i = 0; i < length; i++) sum_raw += buffer[i];
    double dynamic_dc_offset = sum_raw / length;

    double sum_sq = 0;
    for (int i = 0; i < length; i++) {
        double centered_val = buffer[i] - dynamic_dc_offset;
        sum_sq += (centered_val * centered_val);
    }
    
    float rms_adc = sqrt(sum_sq / length);
    float rms_voltage = (rms_adc / 4095.0) * adc_voltage_ref;
    float primary_current = (rms_voltage / burden_resistor_ohms) * ct_turns_ratio;
    
    return primary_current;
}

// New function to handle the float array from the MPU6050
float calculateRMS_Float(volatile float* buffer, int length) {
    double sum_raw = 0;
    for (int i = 0; i < length; i++) sum_raw += buffer[i];
    double mean = sum_raw / length; // Center the vibration around 0

    double sum_sq = 0;
    for (int i = 0; i < length; i++) {
        double centered_val = buffer[i] - mean;
        sum_sq += (centered_val * centered_val);
    }
    
    return sqrt(sum_sq / length);
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

// ==========================================
// TASK: 1 kHz Sensor Polling (Core 1)
// ==========================================
void sensorReadTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        if (!buffer_ready) {
            buffer_L1[sample_index] = analogRead(PIN_CT_L1);
            buffer_L2[sample_index] = analogRead(PIN_CT_L2);
            buffer_L3[sample_index] = analogRead(PIN_CT_L3);
            
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);
            buffer_vibX[sample_index] = a.acceleration.x;
            
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
            mcp.wake();
            current_temp_c = mcp.readTempC();
            mcp.shutdown(); 

            // Extract Features
            feature_rms_L1 = calculateRMS(buffer_L1, SAMPLE_RATE_HZ);
            feature_rms_L2 = calculateRMS(buffer_L2, SAMPLE_RATE_HZ);
            feature_rms_L3 = calculateRMS(buffer_L3, SAMPLE_RATE_HZ);
            feature_rms_vibX = calculateRMS_Float(buffer_vibX, SAMPLE_RATE_HZ);
            
            detectPhaseType();

            // CSV Logging
            Serial.print(millis());
            Serial.print(", ");
            Serial.print(feature_rms_L1);
            Serial.print(", ");
            Serial.print(feature_rms_L2);
            Serial.print(", ");
            Serial.print(feature_rms_L3);
            Serial.print(", ");
            Serial.print(feature_rms_vibX);
            Serial.print(", ");
            Serial.println(current_temp_c);

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
        uint16_t x, y;
        bool pressed = tft.getTouch(&x, &y);

        // Touch logic for BOTTOM Navigation Bar (Y > 190)
        // If your touch is truly inverted top-to-bottom, pressing the physical bottom 
        // will trigger a small Y value. If the tabs don't switch, change `y > 190` back to `y < 50`.
        if (pressed && y > 190) { 
            if (x < 106 && currentTab != HOME_TAB) { currentTab = HOME_TAB; forceUIUpdate = true; }
            else if (x >= 106 && x < 213 && currentTab != GRAPH_TAB) { currentTab = GRAPH_TAB; forceUIUpdate = true; }
            else if (x >= 213 && currentTab != NUMBER_TAB) { currentTab = NUMBER_TAB; forceUIUpdate = true; }
            vTaskDelay(pdMS_TO_TICKS(200)); 
        }

        if (forceUIUpdate) {
            tft.fillScreen(TFT_BLACK);
            drawTabs();
            drawStaticContent(); 
            forceUIUpdate = false;
        }

        drawDynamicContent();
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

// ==========================================
// UI DRAWING FUNCTIONS
// ==========================================
void drawTabs() {
    // Tabs moved to the bottom 40 pixels (Y = 200 to 240)
    tft.fillRect(0, 200, 106, 40, currentTab == HOME_TAB ? TFT_BLUE : TFT_DARKGREY);
    tft.fillRect(107, 200, 106, 40, currentTab == GRAPH_TAB ? TFT_BLUE : TFT_DARKGREY);
    tft.fillRect(214, 200, 106, 40, currentTab == NUMBER_TAB ? TFT_BLUE : TFT_DARKGREY);
    
    tft.setTextColor(TFT_WHITE); 
    tft.drawCentreString("HOME", 53, 212, 2);
    tft.drawCentreString("GRAPH", 160, 212, 2);
    tft.drawCentreString("NUMS", 267, 212, 2);
}

void drawStaticContent() {
    tft.setTextColor(TFT_WHITE); 
    tft.setTextSize(2);

    // Text moved up since the tabs are now at the bottom
    if (currentTab == HOME_TAB) {
        tft.setCursor(10, 20); tft.print("Motor Type: ");
        tft.setCursor(10, 60); tft.print("Status: ");
        tft.setCursor(10, 100); tft.print("Health Index:");
        tft.setCursor(10, 140); tft.print("Time to Fault:");
    } 
    else if (currentTab == NUMBER_TAB) {
        tft.setCursor(10, 10); tft.print("L1 RMS (A):");
        tft.setCursor(10, 45); tft.print("L2 RMS (A):");
        tft.setCursor(10, 80); tft.print("L3 RMS (A):");
        tft.setCursor(10, 115); tft.print("VibX(m/s2):");
        tft.setCursor(10, 150); tft.print("Temp (C)  :");
    }
    else if (currentTab == GRAPH_TAB) {
        tft.drawRect(10, 10, 300, 175, TFT_WHITE); 
        tft.setCursor(20, 20);
        tft.setTextSize(1);
        tft.print("Real-time plotting goes here...");
    }
}

void drawDynamicContent() {
    tft.setTextSize(2);
    
    if (currentTab == HOME_TAB) {
        tft.setCursor(140, 20);
        tft.setTextColor(TFT_CYAN, TFT_BLACK); 
        if (currentPhase == DETECTING) tft.print("Detecting... "); 
        else if (currentPhase == OFFLINE) tft.print("Offline      ");
        else if (currentPhase == SINGLE_PHASE) tft.print("1-Phase      ");
        else if (currentPhase == THREE_PHASE) tft.print("3-Phase      ");

        tft.setCursor(110, 60);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.print("HEALTHY      "); 

        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.setCursor(170, 100); tft.print("0.98  ");
        tft.setCursor(180, 140); tft.print(">30 Days");
    } 
    else if (currentTab == NUMBER_TAB) {
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.setCursor(160, 10);  tft.print(feature_rms_L1, 2); tft.print("   ");
        tft.setCursor(160, 45);  tft.print(feature_rms_L2, 2); tft.print("   ");
        tft.setCursor(160, 80);  tft.print(feature_rms_L3, 2); tft.print("   ");
        tft.setCursor(160, 115); tft.print(feature_rms_vibX, 2); tft.print("   ");
        tft.setCursor(160, 150); tft.print(current_temp_c, 1); tft.print("   ");
    }
}