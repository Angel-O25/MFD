#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MCP9808.h>

// --- PIN DEFINITIONS ---
#define PIN_CT_L1 36
#define PIN_CT_L2 39
#define PIN_CT_L3 34
#define I2C_SDA 21
#define I2C_SCL 22

// --- GLOBAL CALIBRATION VARIABLES (MCT-10) ---
// Adjust these based on your specific burden resistor and voltage divider
float ct_turns_ratio = 2000.0;    // 10A / 5mA
float burden_resistor_ohms = 100.0;
float adc_voltage_ref = 3.3;      // ESP32 ADC reference voltage
float dc_bias_offset_v = 1.65;    // Voltage divider offset

// CT Calibration multipliers (can be tweaked per phase if sensors vary)
float cal_L1 = 1.0;
float cal_L2 = 1.0;
float cal_L3 = 1.0;

// --- HARDWARE OBJECTS ---
TFT_eSPI tft = TFT_eSPI(); 
Adafruit_MPU6050 mpu;
Adafruit_MCP9808 mcp;

// --- DATA BUFFERS (1-Second Window) ---
#define SAMPLE_RATE_HZ 1000
volatile uint16_t buffer_L1[SAMPLE_RATE_HZ];
volatile uint16_t buffer_L2[SAMPLE_RATE_HZ];
volatile uint16_t buffer_L3[SAMPLE_RATE_HZ];
volatile float buffer_vibX[SAMPLE_RATE_HZ];
volatile float buffer_vibY[SAMPLE_RATE_HZ];
volatile float buffer_vibZ[SAMPLE_RATE_HZ];

volatile int sample_index = 0;
volatile bool buffer_ready = false;
volatile float current_temp_c = 0.0;

// --- FREERTOS TASK HANDLES ---
TaskHandle_t TaskSensorRead;
TaskHandle_t TaskProcessData;
TaskHandle_t TaskUI;

// --- FUNCTION PROTOTYPES ---
void sensorReadTask(void *pvParameters);
void processDataTask(void *pvParameters);
void uiTask(void *pvParameters);

void setup() {
    Serial.begin(115200);
    
    // Initialize I2C at 400kHz for fast MPU polling
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000); 

    // Initialize ADC
    analogReadResolution(12);

    // Initialize Sensors
    if (!mpu.begin()) Serial.println("Failed to find MPU6050");
    if (!mcp.begin()) Serial.println("Failed to find MCP9808");
    
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ); // Maximize response for 1kHz sampling

    // Initialize TFT
    tft.init();
    tft.setRotation(1); // Landscape
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString("System Initializing...", 10, 10, 4);

    // --- FREERTOS TASK CREATION ---
    // Core 1: Hard real-time sensor reading (Priority 3 - Highest)
    xTaskCreatePinnedToCore(sensorReadTask, "SensorRead", 4096, NULL, 3, &TaskSensorRead, 1);
    
    // Core 1: 1-second data processing & logging (Priority 2)
    xTaskCreatePinnedToCore(processDataTask, "ProcessData", 8192, NULL, 2, &TaskProcessData, 1);
    
    // Core 0: Touchscreen and UI updates (Priority 1)
    xTaskCreatePinnedToCore(uiTask, "UI", 8192, NULL, 1, &TaskUI, 0);
}

void loop() {
    // Empty. FreeRTOS handles the execution in the tasks above.
    vTaskDelete(NULL); 
}

// ==========================================
// TASK: 1 kHz Sensor Polling (Core 1)
// ==========================================
void sensorReadTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1); // 1 millisecond period (1kHz)

    for (;;) {
        if (!buffer_ready) {
            // Read ADC
            buffer_L1[sample_index] = analogRead(PIN_CT_L1);
            buffer_L2[sample_index] = analogRead(PIN_CT_L2);
            buffer_L3[sample_index] = analogRead(PIN_CT_L3);

            // Read MPU6050
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);
            buffer_vibX[sample_index] = a.acceleration.x;
            buffer_vibY[sample_index] = a.acceleration.y;
            buffer_vibZ[sample_index] = a.acceleration.z;

            sample_index++;

            // Check if 1-second window is full
            if (sample_index >= SAMPLE_RATE_HZ) {
                sample_index = 0;
                buffer_ready = true; // Signal the Process task
            }
        }
        // Block task until exactly 1ms has passed since the last unblock
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

// ==========================================
// TASK: 1-Second Processing & Logging (Core 1)
// ==========================================
void processDataTask(void *pvParameters) {
    for (;;) {
        if (buffer_ready) {
            // 1. Read slow-changing temperature
            mcp.wake();
            current_temp_c = mcp.readTempC();
            mcp.shutdown(); // Conserve power/reduce self-heating

            // 2. Placeholder for Feature Extraction (RMS, THD, etc.)
            // float rms_L1 = calculateRMS((uint16_t*)buffer_L1, cal_L1);
            
            // 3. Serial Logging (CSV Format)
            // timestamp_ms, I1_RMS, I2_RMS, I3_RMS, VibX_RMS, VibY_RMS, VibZ_RMS, Temp_C
            Serial.print(millis());
            Serial.print(", ");
            Serial.print("RMS_L1_PLACEHOLDER"); // Replace with actual calculated RMS
            Serial.print(", ");
            Serial.print("RMS_L2_PLACEHOLDER");
            Serial.print(", ");
            Serial.print("RMS_L3_PLACEHOLDER");
            Serial.print(", ");
            Serial.print("VIB_X_PLACEHOLDER");
            Serial.print(", ");
            Serial.print("VIB_Y_PLACEHOLDER");
            Serial.print(", ");
            Serial.print("VIB_Z_PLACEHOLDER");
            Serial.print(", ");
            Serial.println(current_temp_c);

            // Release buffer for the next second of data collection
            buffer_ready = false; 
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Yield to watchdogs
    }
}

// ==========================================
// TASK: TFT UI & Touch Handling (Core 0)
// ==========================================
void uiTask(void *pvParameters) {
    for (;;) {
        // UI updates will go here.
        // Example: Update a simple status text every 500ms
        tft.fillRect(10, 40, 300, 30, TFT_BLACK); // Clear previous text
        tft.setCursor(10, 40);
        tft.print("Temp: ");
        tft.print(current_temp_c);
        tft.print(" C");

        // Polling touch events
        uint16_t x, y;
        if (tft.getTouch(&x, &y)) {
            // Handle tab switching or data labeling here
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Update UI at 10Hz to save CPU
    }
}