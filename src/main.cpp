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
float dc_bias_offset_v = 1.65;    

// --- HARDWARE OBJECTS ---
TFT_eSPI tft = TFT_eSPI(); 
Adafruit_MPU6050 mpu;
Adafruit_MCP9808 mcp;

// --- DATA BUFFERS ---
#define SAMPLE_RATE_HZ 1000
volatile uint16_t buffer_L1[SAMPLE_RATE_HZ];
volatile float buffer_vibX[SAMPLE_RATE_HZ];
volatile int sample_index = 0;
volatile bool buffer_ready = false;

// --- EXTRACTED FEATURES (Global for UI access) ---
volatile float feature_rms_L1 = 0.0;
volatile float feature_pk_L1 = 0.0;
volatile float feature_rms_vibX = 0.0;
volatile float current_temp_c = 0.0;

// --- UI STATE MACHINE ---
enum TabState { HOME_TAB, GRAPH_TAB, NUMBER_TAB };
TabState currentTab = HOME_TAB;
bool forceUIUpdate = true; // Flag to redraw static UI elements

// --- FREERTOS TASK HANDLES ---
TaskHandle_t TaskSensorRead;
TaskHandle_t TaskProcessData;
TaskHandle_t TaskUI;

// --- FUNCTION PROTOTYPES ---
void sensorReadTask(void *pvParameters);
void processDataTask(void *pvParameters);
void uiTask(void *pvParameters);
void drawTabs();
void drawHomeContent();
void drawGraphContent();
void drawNumberContent();
float calculateRMS(volatile uint16_t* buffer, int length);
float calculatePeakToPeak(volatile uint16_t* buffer, int length);

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); 

    analogReadResolution(12);

    if (!mpu.begin()) Serial.println("MPU6050 Error");
    if (!mcp.begin()) Serial.println("MCP9808 Error");
    
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ); 

    tft.init();
    tft.setRotation(1); 
    
    // Touch calibration data (You will need to run a calibration sketch to get these exact values)
    uint16_t calData[5] = { 275, 3620, 264, 3532, 1 };
    tft.setTouch(calData);

    tft.fillScreen(TFT_BLACK);

    xTaskCreatePinnedToCore(sensorReadTask, "Sensor", 4096, NULL, 3, &TaskSensorRead, 1);
    xTaskCreatePinnedToCore(processDataTask, "Process", 8192, NULL, 2, &TaskProcessData, 1);
    xTaskCreatePinnedToCore(uiTask, "UI", 8192, NULL, 1, &TaskUI, 0);
}

void loop() { vTaskDelete(NULL); }

// ==========================================
// FEATURE EXTRACTION MATH
// ==========================================
float calculateRMS(volatile uint16_t* buffer, int length) {
    double sum_sq = 0;
    // Note: This calculates raw ADC RMS. You must subtract the DC bias (approx 2048) first.
    for (int i = 0; i < length; i++) {
        int centered_val = buffer[i] - 2048; // Remove 1.65V DC offset
        sum_sq += (centered_val * centered_val);
    }
    float rms_adc = sqrt(sum_sq / length);
    
    // Convert ADC RMS to true Amperage (Needs tuning with your exact hardware)
    float rms_voltage = (rms_adc / 4095.0) * adc_voltage_ref;
    float primary_current = (rms_voltage / burden_resistor_ohms) * ct_turns_ratio;
    return primary_current;
}

float calculatePeakToPeak(volatile uint16_t* buffer, int length) {
    uint16_t min_val = 4095;
    uint16_t max_val = 0;
    for (int i = 0; i < length; i++) {
        if (buffer[i] > max_val) max_val = buffer[i];
        if (buffer[i] < min_val) min_val = buffer[i];
    }
    return (max_val - min_val);
}

// ==========================================
// TASK: 1 kHz Sensor Polling (Core 1)
// ==========================================
void sensorReadTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
        if (!buffer_ready) {
            buffer_L1[sample_index] = analogRead(PIN_CT_L1);
            
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
            feature_pk_L1 = calculatePeakToPeak(buffer_L1, SAMPLE_RATE_HZ);
            
            // Note: Add Vib RMS math here

            // CSV Logging
            Serial.print(millis());
            Serial.print(", ");
            Serial.print(feature_rms_L1);
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
    for (;;) {
        uint16_t x, y;
        bool pressed = tft.getTouch(&x, &y);

        // Simple Touch Zone Logic for Top Navigation Bar
        if (pressed && y < 40) { // If touch is in the top 40 pixels
            if (x < 106 && currentTab != HOME_TAB) { currentTab = HOME_TAB; forceUIUpdate = true; }
            else if (x > 106 && x < 213 && currentTab != GRAPH_TAB) { currentTab = GRAPH_TAB; forceUIUpdate = true; }
            else if (x > 213 && currentTab != NUMBER_TAB) { currentTab = NUMBER_TAB; forceUIUpdate = true; }
        }

        if (forceUIUpdate) {
            tft.fillScreen(TFT_BLACK);
            drawTabs();
            forceUIUpdate = false;
        }

        // Draw dynamic content based on active tab
        if (currentTab == HOME_TAB) drawHomeContent();
        else if (currentTab == GRAPH_TAB) drawGraphContent();
        else if (currentTab == NUMBER_TAB) drawNumberContent();

        vTaskDelay(pdMS_TO_TICKS(100)); // UI updates at 10Hz
    }
}

// ==========================================
// UI DRAWING FUNCTIONS
// ==========================================
void drawTabs() {
    tft.fillRect(0, 0, 106, 40, currentTab == HOME_TAB ? TFT_BLUE : TFT_DARKGREY);
    tft.fillRect(107, 0, 106, 40, currentTab == GRAPH_TAB ? TFT_BLUE : TFT_DARKGREY);
    tft.fillRect(214, 0, 106, 40, currentTab == NUMBER_TAB ? TFT_BLUE : TFT_DARKGREY);
    
    tft.setTextColor(TFT_WHITE);
    tft.drawCentreString("HOME", 53, 10, 2);
    tft.drawCentreString("GRAPH", 160, 10, 2);
    tft.drawCentreString("NUMS", 267, 10, 2);
}

void drawHomeContent() {
    // Overwrite old text with a black rectangle before drawing new text
    tft.fillRect(10, 60, 300, 100, TFT_BLACK); 
    tft.setCursor(10, 60);
    tft.setTextColor(TFT_GREEN);
    tft.setTextSize(2);
    tft.println("MOTOR STATUS: HEALTHY"); // Placeholder for ML output
    
    tft.setTextColor(TFT_WHITE);
    tft.print("Health Index: ");
    tft.println("0.98"); 
    
    tft.print("Est. Time to Fault: ");
    tft.println(">30 Days");
}

void drawGraphContent() {
    // Placeholder for waveform rendering logic
    tft.fillRect(10, 60, 300, 160, TFT_BLACK);
    tft.drawRect(10, 60, 300, 160, TFT_WHITE);
    tft.setCursor(20, 70);
    tft.setTextSize(1);
    tft.print("Real-time plotting goes here...");
    // Note: To draw the graph efficiently, map() the buffer values to Y-coordinates
    // and draw lines between the points.
}

void drawNumberContent() {
    tft.fillRect(10, 60, 300, 160, TFT_BLACK);
    tft.setCursor(10, 60);
    tft.setTextSize(2);
    
    tft.print("RMS L1 (A): "); tft.println(feature_rms_L1);
    tft.print("Pk-Pk L1: "); tft.println(feature_pk_L1);
    tft.print("Temp (C): "); tft.println(current_temp_c);
}