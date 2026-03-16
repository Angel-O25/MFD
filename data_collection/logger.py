"""
=============================================================================
MOTOR FAULT DATA COLLECTION LOGGER - USER MANUAL
=============================================================================
Hello! Thank you for helping collect data for this motor fault detection thesis.
Please read these instructions carefully before starting.

PREREQUISITES:
1. You must have Python installed on your computer.
2. You must install the PySerial library. Open your terminal/command prompt and type:
   pip install pyserial

BEFORE YOU START:
1. Plug the ESP32 via USB into your computer.
2. ***CRITICAL RULE***: Make sure PlatformIO, Arduino IDE, or any other Serial Monitor 
   is CLOSED. If another program is watching the COM port, this script will crash!

HOW TO COLLECT DATA:
1. Run this script by typing: python logger.py
2. Look at the list of available COM ports and type yours in.
3. The script will ask you for three things based on the experiment matrix:
   - Condition (Healthy, MF, EF, TO, Combined)
   - Severity (None, Mild, Moderate, Severe)
   - Load % (0, 50, 75, 100)
4. Turn on the motor setup.
5. Press Enter on your keyboard to start recording. You will see data streaming.
6. Wait exactly 60 seconds (or 10 minutes for Thermal Overload tests).
7. Press CTRL + C on your keyboard to stop recording and safely save the CSV file.
=============================================================================
"""

import serial
import serial.tools.list_ports
import csv
import time
from datetime import datetime
import sys
import os

# --- CONFIGURATION ---
BAUD_RATE = 115200

def print_manual():
    print("\n" + "="*50)
    print(" 🛠️  MOTOR FAULT DATA LOGGER - QUICK START GUIDE")
    print("="*50)
    print("1. Ensure PlatformIO / Arduino Serial Monitor is CLOSED.")
    print("2. Select your COM Port.")
    print("3. Enter the test parameters when prompted.")
    print("4. Turn on the motor.")
    print("5. Let the script run for 60 seconds.")
    print("6. Press CTRL+C to stop and save the data.\n")
    print("Valid Conditions : Healthy, MF, EF, TO, Combined")
    print("Valid Severities : None, Mild, Moderate, Severe, Mild+Moderate")
    print("Valid Loads (%)  : 0, 50, 75, 100")
    print("="*50 + "\n")

def get_available_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("⚠️  No COM ports found! Is the ESP32 plugged in?")
        return []
    
    print("--- Available COM Ports ---")
    for port in ports:
        print(f" - {port.device}: {port.description}")
    print("---------------------------")
    return [port.device for port in ports]

def main():
    print_manual()
    
    # 1. Automatically scan and ask for COM Port
    get_available_ports()
    com_port = input("\nEnter the COM Port from the list above (e.g., COM3): ").strip()
    
    # 2. Ask for the experimental parameters
    print("\n--- Experiment Parameters ---")
    condition = input("Enter Fault Condition (Healthy, MF, EF, TO, Combined): ").strip()
    severity = input("Enter Severity (None, Mild, Moderate, Severe): ").strip()
    load = input("Enter Motor Load % (0, 50, 75, 100): ").strip()

    # Confirm before starting
    input(f"\n[?] Ready to record: {condition} | {severity} | {load}% Load on {com_port}. Press ENTER to start...")

    # 3. Generate an automatic, timestamped filename
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    filename = f"dataset_{condition}_{severity}_{load}Load_{timestamp}.csv"
    filename = filename.replace('+', 'plus') # Fix filename characters if they typed 'Mild+Moderate'

    # Find the user's Desktop, regardless of whether they are on Windows or Mac
    desktop_dir = os.path.join(os.path.expanduser("~"), "Desktop")
    save_dir = os.path.join(desktop_dir, "Motor_Fault_Data")
    
    # Create the folder if it doesn't exist yet
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
        
    # Lock the file path to that Desktop folder
    filepath = os.path.join(save_dir, filename)
    
    # 4. Define the CSV Headers
    headers = [
        "Time_ms", 
        "L1_RMS_A", "L2_RMS_A", "L3_RMS_A", "Current_Unbalance_Pct",
        "VibX_RMS", "VibY_RMS", "VibZ_RMS", 
        "VibX_Kurtosis", "VibY_Kurtosis", "VibZ_Kurtosis",
        "VibX_Crest", "VibY_Crest", "VibZ_Crest",
        "Temp_C", "Temp_Slope", 
        "Label_Condition", "Label_Severity", "Label_Load"
    ]

    try:
        # 1. THE HARDWARE FIX: Connect to ESP32 WITHOUT resetting it
        ser = serial.Serial()
        ser.port = com_port
        ser.baudrate = BAUD_RATE
        ser.timeout = 2
        
        # Tell the USB bridge NOT to toggle the ESP32's reset/boot pins
        ser.dtr = False
        ser.rts = False
        ser.open()

        print(f"\n[+] Connected to {com_port} (Hardware reset bypassed!)")
        
        # FIX: Changed filename to filepath so it prints the full location
        print(f"[+] Recording data to: {filepath}")
        print("[!] >> PRESS CTRL+C TO STOP RECORDING AND SAVE <<\n")
        
        # FIX: Changed filename to filepath so it actually saves to the Desktop folder
        with open(filepath, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(headers) 
            
            # 5. Listen to the Serial Port continuously
            while True:
                if ser.in_waiting > 0:
                    # 2. THE SOFTWARE SHIELD: Catch any weird parsing errors instantly
                    try:
                        # Use ascii instead of utf-8, it's stricter and ignores garbage better
                        raw_bytes = ser.readline()
                        line = raw_bytes.decode('ascii', errors='ignore').strip()
                        
                        if line:
                            sensor_data = [val.strip() for val in line.split(',')]
                            
                            if len(sensor_data) == 16:
                                sensor_data.extend([condition, severity, load])
                                writer.writerow(sensor_data)
                                print(f"Logged: {sensor_data}")
                            else:
                                print(f"Skipping malformed data length: {line}")
                    
                    except Exception as loop_err:
                        # If a line is completely cursed, ignore it and keep the script alive
                        print(f"Ignored a cursed byte sequence: {loop_err}")

    except serial.SerialException as e:
        print(f"\n[-] ERROR: Could not open {com_port}.")
        print("[-] FIX: Is PlatformIO's Serial Monitor still open? Close it and try again!")
        input("\nPress Enter to exit...") 
    except KeyboardInterrupt:
        print("\n\n[+] Data collection stopped by user.")
        
        # FIX: Changed filename to filepath here too
        print(f"[+] File saved successfully: {filepath}")
        print("[+] Great job! You can now turn off the motor.\n")
    except Exception as e:
        print(f"\n[-] AN UNEXPECTED FATAL ERROR OCCURRED: {e}")
        input("\nPress Enter to exit...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()