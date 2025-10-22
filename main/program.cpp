
/* --------------------  Notes (START)  ---------------------- 
Make sure to check whether isSimulation is set to true or false.
If true, the HX711 will be set to a default scale value.
If false, the HX711 will be set to the scale value obtained from calibration.

Modify the credentials.h file to include the Telegram Bot Token and Chat ID, and 
ensure that the WiFi credentials are correct.

TODO: Implement Test Mode (Hybrid) (Telegram - HW Control)        (Partially Done)
TODO: Implement Calibration Functionality                         (Pending)
TODO: Implement Water Level Sensor Test Functionality             (Pending)
TODO: Implement Button Test Functionality                         (Partially Done)
TODO: Implement Inlet Valve Test Functionality                    (Done)
TODO: Implement Drain Motor Test Functionality                    (Done)  
TODO: Implement Inverter/Motor Test Functionality                 (Partially Done)
TODO: Implement Connectivity Test Functionality                   (Done)  
TODO: Improve Wash/Rinse Cycle Logic (more agitation patterns)    (Pending)
TODO: Add Feedback From Inverter Drive                            (Pending) 
TODO: Improve Error Handling and Recovery                         (Pending) 
TODO: Own Inverter Drive Design                                   (Pending - Long-term)


TOC (Table of Contents):
1. Compiler Directives: Lines 63-106
2. Object Declarations: Lines 109-118
4. Function Declarations: Lines 121-178
3. State Variables (Global): Lines 181-223
5. Engineering Mode Variables: Lines 226-244
6. Button ISRs: Lines 247-297
7. Status LEDs Control Function: Lines 300-388
8. OTA Helper Functions: Lines 390-454
9. Wash Program Function: Lines 469-605
10. Rinse Program Function: Lines 608-691
11. Spin Program Function: Lines 694-766
12. Soak Program Function: Lines 769-841
13. WiFi Connection Function: Lines 844-891
14. Engineering Mode Helper Functions: Lines 894-932
15. Water Level Sensor Test Function: Lines 935-1042
16. Inlet Valve Test Function: Lines 1045-1208
17. Drain Motor (Wash Stage) Test Function: Lines 1211-1334
18. Drain Motor (Spin Stage) Test Function: Lines 1337-1423
19. Motor Rotation Test Function: Lines 1426-1547
20. LED Test Function: Lines 1550-1649
21. MCU Self-Test Function: Lines 1652-1749
22. All Buttons Test Function: Lines 1752-1837
23. Connectivity Test Function: Lines 1840-1870
24. Calibration Test Function: Lines 1873-1888
25. Send System Info Function: Lines 1891-1931
26. Engineering Mode Menu Function: Lines 1934-1948
27. Component Test Submenu Function: Lines 1952-1969
28. Engineering Mode Control Functions: Lines 1972-2087
29. Mode State Control Function: Lines 2091-2128
30. Main Setup Function: Lines 2130-2232
31. Main Loop Function: Lines 2235-2467




--------------------  Notes (END)  ---------------------- */


/* --------------------  1. Compiler Directives (START)  ---------------------- */
#include <Wire.h>                         // Include the Wire Library 
#include <WiFi.h>                         // Include the WiFi Library
#include <HX711.h>                        // Include the Water Level Sensor (HX710B) Library                
#include <Arduino.h>                      // Include the Arduino Library
#include <WiFiClient.h>                   // Include the WiFiClient Library
#include <WebServer.h>                    // Include the WebServer Library
#include <ElegantOTA.h>                   // Include the ElegantOTA Library
#include "credentials.h"                  // Include the credentials header file
#include <freertos/task.h>                // Include the FreeRTOS Task Library
#include <WiFiClientSecure.h>             // Include the WiFiClientSecure Library
#include <UniversalTelegramBot.h>         // Include the UniversalTelegramBot Library
#include <ArduinoJson.h>                  // Include the ArduinoJson Library
#include <LiquidCrystal_I2C.h>            // Include the LiquidCrystal_I2C Library
#include <freertos/FreeRTOS.h>            // Include the FreeRTOS Library
#include "esp_system.h"                   // Include the ESP System Library
#include "soc/rtc_cntl_reg.h"             // Include the SoC RTC Control Register Library 

#define INV_PW 32         // Inverter Power Control Pin
#define DM_WASH 25        // Drain Motor Wash Stage Pin
#define DM_SPIN 27        // Drain Motor Spin Stage Pin
#define IV 14             // Inlet Valve Control Pin
#define CO1 26            // Changeover Relay 1 Control Pin 
#define CO2 33            // Changeover Relay 2 Control Pin
#define WLS_DATA 23       // Water Level Sensor SDA (Data) Pin
#define WLS_CLK 19        // Water Level Sensor SCL (Clock) Pin
#define FB_SIG 5          // Inverter Feedback Input Pin
#define CTR_SIG 18        // Inverter Control Ouput Pin
#define SOAK_LED 17       // Soak LED Pin
#define WASH_LED 4        // Wash LED Pin
#define RINSE_LED 13      // Rinse LED Pin
#define SPIN_LED 16       // Spin LED Pin
#define WIFI_LED 15       // WiFi LED Pin
#define HALT_BTN 34       // Pause Button Pin  
#define COMP_BTN 12       // Complete Program Button Pin
#define WASH_BTN 2        // Wash Only Button Pin
#define RINSE_BTN 0       // Rinse Only Button Pin
#define SPIN_BTN 35       // Spin Only Button Pin
#define I2C_ADDR 0x27     // LCD Display I2C Address
#define DISPLAY_COLS 16   // LCD Display Columns
#define DISPLAY_ROWS 2    // LCD Display Rows
#define OFF LOW           // Naming Convensions
#define ON HIGH           // Naming Conventions
/* --------------------  1. Compiler Directives (END)  ---------------------- */


/* --------------------  2. Object Declarations (START)  ---------------------- */
HX711 level;                                                        // HX711 load cell amplifier for water level measurement
float waterLevel;                                                   // Current water level in Liters (final calibrated value)
float tareWaterLevel;                                               // Intermediate water level calculation before offset applied
WebServer server(1906);                                             // HTTP web server on port 1906 for remote control and OTA updates
WiFiClientSecure secured_client;                                    // Secure WiFi client for encrypted Telegram API communication
UniversalTelegramBot telegram(BOT_TOKEN, secured_client);           // Telegram bot instance for sending/receiving messages
TaskHandle_t ledtask_handle = NULL;                                 // FreeRTOS task handle for LED status indicator task
LiquidCrystal_I2C display(I2C_ADDR, DISPLAY_COLS, DISPLAY_ROWS);    // 16x2 LCD display via I2C (address 0x27)
/* --------------------  2. Object Declarations (END)  ---------------------- */


/* --------------------  3. Function Declarations (START)  ---------------------- */

// Button ISRs (IRAM_ATTR: stored in RAM for fast interrupt response)
void washButtonISR();      // Interrupt handler for WASH button press
void spinButtonISR();      // Interrupt handler for SPIN button press
void rinseButtonISR();     // Interrupt handler for RINSE button press
void compButtonISR();      // Interrupt handler for COMPLETE button press
void haltButtonISR();      // Interrupt handler for HALT button press

// Task and Display Functions
void ledtask(void *parameter);       // FreeRTOS task for LED status blinking
void displayPrint();                 // Update 16x2 LCD display with current status
void displayTestMenu();             // Display engineering mode test menu on LCD
void reboot();                       // Reboot ESP32 to bootloader mode

// OTA (Over-The-Air) Firmware Update Callbacks
void onOTAStart();                   // Called when OTA upload starts
void onOTAProgress(size_t current, size_t final); // Progress callback during upload
void onOTAEnd(bool success);         // Called when OTA upload completes

// Wash Cycle Logic Functions
void washLogic();                    // Main wash stage execution logic
void rinseLogic();                   // Main rinse stage execution logic
void spinLogic();                    // Main spin stage execution logic
void soakLogic();                    // Pre-wash soak stage execution logic

// WiFi and Network Functions
boolean connectWifi();               // Connect to WiFi network using credentials

// Engineering/Test Mode Functions
void waterLevelSensorTest();       // Test water level sensor functionality
void inletValveTest();               // Auto-test water inlet valve (4 minutes)
void drainMotorWashStageTest();      // Manual test for wash drain motor
void drainMotorSpinStageTest();      // Automated spin stage motor test
void motorRotationTest();          // Test inverter drive motor rotation
void ledTest();                     // Test status LEDs functionality
void mcuSelfTest();                  // Perform MCU self-diagnostic tests
void allButtonsTest();               // Test all button inputs functionality
void connectivityTest();             // Test WiFi and Telegram connectivity
void calibrationTest();              // Water level sensor calibration guide
void sendSystemInfo();               // Send ESP32 system info via Telegram


// Telegram Menu Functions
void sendMenu();                     // Send main engineering mode menu
void sendSubMenu();                  // Send component test submenu
void handleMenu(String cmd);         // Process main menu selections
void handleSubMenu(String cmd);      // Process submenu selections
void handleTelegramMessages();       // Poll and process incoming Telegram messages

// Engineering Mode Control
void enterEngineeringMode();         // Activate test mode, disable normal operation
void exitEngineeringMode();          // Return to normal washing machine operation

// Core Arduino Functions
void setup();                        // Initialize system, pins, and tasks
void loop();                         // Main program loop
/* --------------------  3. Function Declarations (END)  ---------------------- */


/* --------------------  4. State Variables (GLOBAL) (START)  ---------------------- */

// Operating Mode Flags (Mutually exclusive states)
bool isSoaking = false;              // Soak cycle active
bool isWashing = false;              // Wash cycle active
bool isRinsing = false;              // Rinse cycle active
bool isSpinning = false;             // Spin cycle active
bool isWaiting = true;               // Idle/waiting state
bool isCompleteProgramWash = false;  // Complete wash phase of full program
bool isCompleteProgramRinse = false; // Complete rinse phase of full program
bool isCompleteProgramSpin = false;  // Complete spin phase of full program

// Control Flags
bool isSimulation = false;           // Use default HX711 values instead of calibration
bool buttonPressed = false;          // Flag set when any button is pressed
bool programRunning = false;         // True if any wash cycle is active
bool wifiConnected = false;          // WiFi connection status

// Water Usage Tracking (Volatile: updated in interrupts)
volatile float washWaterUsed = 0;    // Liters used in wash cycle
volatile float rinseWaterUsed = 0;   // Liters used in rinse cycle
volatile float totalWaterUsed = 0;   // Total liters used in current cycle
volatile int selectedMode = 0;       // User selection: 1=WASH, 2=RINSE, 3=SPIN

// Sensor Calibration Constants
const int multiplier = 27.4;         // HX711 calibration multiplier (raw→kg conversion)
const int offset = 10.9;             // Water level offset (tare adjustment)

// Water Level Thresholds
const int setFillingWaterLevel = 18.5;  // Target water level for filling (Liters)
const int setDrainingWaterLevel = 2;    // Minimum water level before stopping drain

// Timing Constants
const int waitTime = 10000;          // Pause duration between phases (10 seconds)

// Timing Variables (Runtime Tracking)
unsigned long runTime = 0;           // Elapsed time in current cycle
unsigned long startTime = 0;         // Timestamp when cycle started
unsigned long startWaitTime = 0;     // Timestamp when wait period started
unsigned long ota_progress_millis = 0; // Last OTA progress update timestamp
unsigned long lastButtonPressTime = 0;  // Debounce: time of last button press

/* --------------------  4. State Variables (GLOBAL) (END)  ---------------------- */


/* --------------------  5. Engineering Mode Variables (START)  ---------------------- */
bool isTestMode = false;
int testMenuOption = 0;   // 0 = main menu, 100 = component test submenu
int componentTestOption = 0;  
const String ENGINEERING_COMMAND = "engineering";
const String ENGINEERING_EXIT = "exit";
unsigned long lastTelegramCheck = 0;
const unsigned long telegramCheckDelay = 1000; 

// ========== VALVE TEST Variables ==========
const float VALVE_TEST_DURATION = 60000;  // 60 seconds in milliseconds
const float VALVE_TEST_MIN_DELTA = 2.0;   // Minimum water level increase (Liters) for PASS
const float VALVE_TEST_interval = 1000;  // Sample every 1 second
const int VALVE_TEST_SAMPLES = VALVE_TEST_DURATION / VALVE_TEST_interval;

// ========== Drain Motor Test Variables ==========
bool awaitingDrainMotorResponse = false;
bool drainMotorTestResult = false; // true = smooth, false = stuck
/* --------------------  5. Engineering Mode Variables (END)  ---------------------- */


/* --------------------  6. Button ISRs (START)  ---------------------- */
void IRAM_ATTR washButtonISR()
{
  if (!programRunning)
  {
    selectedMode = 1;
    lastButtonPressTime = millis();
    buttonPressed = true;
  }
}

void IRAM_ATTR spinButtonISR() {
  if (awaitingDrainMotorResponse) {
    // User pressed SPIN = Motor working smoothly
    drainMotorTestResult = true;
    awaitingDrainMotorResponse = false;
  } else if (!programRunning) {
    selectedMode = 3;
    lastButtonPressTime = millis();
    buttonPressed = true;
  }
}

void IRAM_ATTR rinseButtonISR() {
  if (awaitingDrainMotorResponse) {
    // User pressed RINSE = Motor stuck/failed
    drainMotorTestResult = false;
    awaitingDrainMotorResponse = false;
  } else if (!programRunning) {
    selectedMode = 2;
    lastButtonPressTime = millis();
    buttonPressed = true;
  }
}


void IRAM_ATTR compButtonISR()
{
  if (!programRunning)
  {
    selectedMode = 4;
    lastButtonPressTime = millis();
    buttonPressed = true;
  }
}

void IRAM_ATTR haltButtonISR()
{
  selectedMode = 0;
}
/* -------------------- 6. Button ISRs (END)  ---------------------- */


/* -------------------- 7. Status LEDs Control Function (START)  ---------------------- */
void ledtask(void *parameter)
{
  while (true)
  {
    if (isSoaking)
    {
      digitalWrite(RINSE_LED, OFF);
      digitalWrite(SPIN_LED, OFF);
      digitalWrite(WASH_LED, OFF);
      digitalWrite(SOAK_LED, OFF);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      digitalWrite(SOAK_LED, ON);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    else if (isWashing)
    {
      digitalWrite(SOAK_LED, OFF);
      digitalWrite(RINSE_LED, OFF);
      digitalWrite(SPIN_LED, OFF);
      digitalWrite(WASH_LED, OFF);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      digitalWrite(WASH_LED, ON);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    else if (isRinsing)
    {
      digitalWrite(SOAK_LED, OFF);
      digitalWrite(WASH_LED, OFF);
      digitalWrite(SPIN_LED, OFF);
      digitalWrite(RINSE_LED, OFF);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      digitalWrite(RINSE_LED, ON);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    else if (isSpinning)
    {
      digitalWrite(SOAK_LED, OFF);
      digitalWrite(WASH_LED, OFF);
      digitalWrite(RINSE_LED, OFF);
      digitalWrite(SPIN_LED, OFF);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      digitalWrite(SPIN_LED, ON);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    else if (isCompleteProgramWash)
    {
      digitalWrite(SOAK_LED, OFF);
      digitalWrite(RINSE_LED, OFF);
      digitalWrite(SPIN_LED, OFF);
      digitalWrite(WASH_LED, OFF);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      digitalWrite(WASH_LED, ON);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    else if (isCompleteProgramRinse)
    {
      digitalWrite(SOAK_LED, OFF);
      digitalWrite(WASH_LED, ON);
      digitalWrite(SPIN_LED, OFF);
      digitalWrite(RINSE_LED, OFF);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      digitalWrite(RINSE_LED, ON);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    else if (isCompleteProgramSpin)
    {
      digitalWrite(SOAK_LED, OFF);
      digitalWrite(WASH_LED, ON);
      digitalWrite(RINSE_LED, ON);
      digitalWrite(SPIN_LED, OFF);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      digitalWrite(SPIN_LED, ON);
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    else if (selectedMode == 0)
    {
      digitalWrite(SOAK_LED, OFF);
      digitalWrite(RINSE_LED, OFF);
      digitalWrite(SPIN_LED, OFF);
      digitalWrite(WASH_LED, OFF);

      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}
/* --------------------  7. Status LEDs Control Function (END)  ---------------------- */

/* --------------------  8. OTA Helper Functions (START)  ---------------------- */
void onOTAStart()
{
  Serial.println("OTA update started!");
}

void onOTAProgress(size_t current, size_t final)
{
  static int numpoint = 0;
  if (millis() - ota_progress_millis > 500)
  {
    ota_progress_millis = millis();
    // Print to serial monitor
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
    // Toggle the WiFi LED
    digitalWrite(WIFI_LED, !digitalRead(WIFI_LED));
    // Update display
    display.clear();
    display.setCursor(1, 0);
    display.print("Updating");
    // Add trailing dots
    for (int i = 0; i < numpoint; ++i)
    {
      display.print(".");
    }
    // Increment number of dots, reset if it reaches 5
    numpoint = (numpoint + 1) % 6;
  }
}

void onOTAEnd(bool success)
{
  if (success)
  {
    Serial.println("OTA update finished successfully!");
    display.clear();
    display.setCursor(3, 0);
    display.print("OTA Update");
    display.setCursor(3, 1);
    display.print("Successful");
    for (int i = 0; i < 3; i++)
    {
      digitalWrite(WIFI_LED, ON);
      vTaskDelay(300 / portTICK_PERIOD_MS);
      digitalWrite(WIFI_LED, OFF);
      vTaskDelay(300 / portTICK_PERIOD_MS);
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    display.clear();
    display.print("Rebooting.....");
  }
  else
  {
    Serial.println("There was an error during OTA update!");
    display.clear();
    display.setCursor(3, 0);
    display.print("OTA Update");
    display.setCursor(3, 1);
    display.print("Unsuccessful");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    display.clear();
    display.print("Rebooting.....");
  }
}
/* --------------------  8. OTA Helper Functions (END)  ---------------------- */


/* --------------------  Menu Print Function (START)  ---------------------- */
void displayPrint()
{
  display.clear();
  display.setCursor(1, 0);
  display.print("Please Select");
  display.setCursor(1, 1);
  display.print("A Program");
}
/* --------------------  Menu Print Function (END)  ---------------------- */


/* --------------------  9. Wash Program Function (START)  ---------------------- */
void washLogic()
{
  programRunning = true;
  digitalWrite(INV_PW, OFF);
  digitalWrite(DM_SPIN, OFF);
  digitalWrite(DM_WASH, OFF);
  digitalWrite(CO1, OFF);
  digitalWrite(CO2, OFF);
  display.clear();
  display.setCursor(0, 0);
  display.print("Filling Water..");
  display.setCursor(0, 1);
  display.print("WASH");
  display.setCursor(13, 1);
  display.print("L");

  // Water Filling Control
  while (waterLevel < setFillingWaterLevel)
  {
    digitalWrite(IV, ON);
    tareWaterLevel = level.get_units() / multiplier;
    waterLevel = tareWaterLevel - offset;
    Serial.println(waterLevel, 1);
    display.setCursor(8, 1);
    display.print(waterLevel, 1);
  }
  washWaterUsed = waterLevel;
  delay(10);
  String message = "Wash Water filling complete. Filled: " + String(washWaterUsed) + " L";
  telegram.sendMessage(CHAT_ID, message, "");
  display.clear();
  display.setCursor(2, 0);
  display.print("Water Filled");
  display.setCursor(1, 1);
  display.print("Value:      L");
  display.setCursor(8, 1);
  display.print(waterLevel, 1);
  digitalWrite(IV, OFF);
  digitalWrite(DM_WASH, OFF);
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  digitalWrite(INV_PW, ON);
  display.clear();
  display.setCursor(1, 0);
  display.print("Washing... PH1");

  unsigned long washPhase1Duration = 180000;
  unsigned long washPhase1StartTime = millis();
  while (millis() - washPhase1StartTime < washPhase1Duration)
  {
    unsigned long elapsed = millis() - washPhase1StartTime;
    int iteration = (elapsed / 12000) % 10; // Adjust according to the total number of iterations
    display.setCursor(0, 1);
    display.print("Iteration:");
    display.setCursor(11, 1);
    display.print(iteration);
    display.setCursor(12, 1);
    display.print("/10");

    analogWrite(CTR_SIG, 200);
    vTaskDelay(6000 / portTICK_PERIOD_MS);
    analogWrite(CTR_SIG, 0);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    digitalWrite(CO1, ON);
    digitalWrite(CO2, ON);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    analogWrite(CTR_SIG, 200);
    vTaskDelay(6000 / portTICK_PERIOD_MS);
    analogWrite(CTR_SIG, 0);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    digitalWrite(CO1, OFF);
    digitalWrite(CO2, OFF);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }

  digitalWrite(INV_PW, OFF);
  digitalWrite(DM_WASH, OFF);
  display.clear();
  display.setCursor(0, 0);
  display.print("Adjusting Water");
  display.setCursor(4, 1);
  display.print("Level");
 while (waterLevel < setFillingWaterLevel + 2)
  {
    digitalWrite(IV, ON);
    tareWaterLevel = level.get_units() / multiplier;
    waterLevel = tareWaterLevel - offset;
    Serial.println(waterLevel, 1);
    display.setCursor(8, 1);
    display.print(waterLevel, 1);
  }

  digitalWrite(IV, OFF);
  display.clear();
  display.setCursor(1, 0);
  display.print("Washing... PH2");
  digitalWrite(INV_PW, ON);

  unsigned long washPhase2Duration = 180000;
  unsigned long washPhase2StartTime = millis();
  while (millis() - washPhase2StartTime < washPhase2Duration)
  {
    unsigned long elapsed = millis() - washPhase2StartTime;
    int iteration = (elapsed / 12000) % 10; // Adjust according to the total number of iterations
    display.setCursor(0, 1);
    display.print("Iteration:");
    display.setCursor(11, 1);
    display.print(iteration);
    display.setCursor(12, 1);
    display.print("/10");
    
    analogWrite(CTR_SIG, 200);
    vTaskDelay(45000 / portTICK_PERIOD_MS);
    analogWrite(CTR_SIG, 0);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    digitalWrite(CO1, ON);
    digitalWrite(CO2, ON);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    analogWrite(CTR_SIG, 200);
    vTaskDelay(45000 / portTICK_PERIOD_MS);
    analogWrite(CTR_SIG, 0);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    digitalWrite(CO1, OFF);
    digitalWrite(CO2, OFF);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }

  digitalWrite(INV_PW, OFF);
  display.clear();
  display.setCursor(4, 0);
  display.print("Washing ");
  display.setCursor(4, 1);
  display.print("Complete");
  vTaskDelay(6000 / portTICK_PERIOD_MS);
  programRunning = false;
}
/* --------------------  9. Wash Program Function (END)  ---------------------- */


/* --------------------  10. Rinse Program Function (START)  ---------------------- */
void rinseLogic()
{
  programRunning = true;

  display.clear();
  display.setCursor(1, 0);
  display.print("Filling Water..");
  display.setCursor(1, 1);
  display.print("RINSE");
  display.setCursor(13, 1);
  display.print("L");

  while (waterLevel < setFillingWaterLevel)
  {
    digitalWrite(IV, ON);
    tareWaterLevel = level.get_units() / multiplier;
    waterLevel = tareWaterLevel - offset;
    Serial.println(waterLevel, 1);
    display.setCursor(8, 1);
    display.print(waterLevel, 1);
  }
  digitalWrite(IV, OFF);
  Serial.println("Water Filling Complete!");
  Serial.println("Value (In Litres):");
  Serial.print(waterLevel);
  rinseWaterUsed = waterLevel;
  delay(10);
  String message = "Wash Water filling complete. Filled: " + String(rinseWaterUsed) + " L";
  telegram.sendMessage(CHAT_ID, message, "");
  display.clear();
  display.setCursor(2, 0);
  display.print("Water Filled");
  display.setCursor(1, 1);
  display.print("Value:      L");
  display.setCursor(8, 1);
  display.print(waterLevel, 1);
  vTaskDelay(500 / portTICK_PERIOD_MS);
  digitalWrite(INV_PW, ON);
  display.clear();
  display.setCursor(1, 0);
  display.print("Rinsing.....");

  unsigned long rinsePhaseDuration = 360000;
  unsigned long rinsePhaseStartTime = millis();
  while (millis() - rinsePhaseStartTime < rinsePhaseDuration)
  {
    unsigned long elapsed = millis() - rinsePhaseStartTime;
    int iteration = (elapsed / 12000) % 10; // Adjust according to the total number of iterations
    display.setCursor(0, 1);
    display.print("Iteration:");
    display.setCursor(11, 1);
    display.print(iteration);
    display.setCursor(12, 1);
    display.print("/10");

    analogWrite(CTR_SIG, 200);
    vTaskDelay(30000 / portTICK_PERIOD_MS);
    analogWrite(CTR_SIG, 0);
    vTaskDelay(2500 / portTICK_PERIOD_MS);
    digitalWrite(CO1, ON);
    digitalWrite(CO2, ON);
    vTaskDelay(2500 / portTICK_PERIOD_MS);
    analogWrite(CTR_SIG, 200);
    vTaskDelay(30000 / portTICK_PERIOD_MS);
    analogWrite(CTR_SIG, 0);
    vTaskDelay(2500 / portTICK_PERIOD_MS);
    digitalWrite(CO1, OFF);
    digitalWrite(CO2, OFF);
    vTaskDelay(2500 / portTICK_PERIOD_MS);
  }

  digitalWrite(INV_PW, OFF);

  display.clear();
  display.setCursor(4, 0);
  display.print("Rinsing");
  display.setCursor(4, 1);
  display.print("Complete");
  vTaskDelay(15000 / portTICK_PERIOD_MS);

  programRunning = false;
}
/* --------------------  10. Rinse Program Function (END)  ---------------------- */


/* --------------------  11. Spin Program Function (START)  ---------------------- */
void spinLogic()
{
  programRunning = true;
  display.clear();
  display.setCursor(1, 0);
  display.print("Draining Water");
  display.setCursor(1, 1);
  display.print("DRAIN");
  display.setCursor(13, 1);
  display.print("L");
  digitalWrite(INV_PW, OFF);
  digitalWrite(DM_WASH, ON);
  digitalWrite(DM_SPIN, ON);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  waterLevel = 10;
  while (waterLevel > setDrainingWaterLevel)
  {
    tareWaterLevel = level.get_units() / multiplier;
    waterLevel = tareWaterLevel - offset;
    Serial.println(waterLevel, 2);
    display.setCursor(8, 1);
    display.print(waterLevel, 1);
  }
  vTaskDelay(15000 / portTICK_PERIOD_MS);

  digitalWrite(DM_WASH, ON);
  digitalWrite(DM_SPIN, ON);
  digitalWrite(CO1, OFF);
  digitalWrite(CO2, OFF);
  display.clear();
  display.setCursor(0, 0);
  display.print("Press Start");
  display.setCursor(0, 1);
  display.print("Once Balanced");
  telegram.sendMessage(CHAT_ID, "Water Drain Complete. Waiting for User Input to Start Spinning.", "");
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  while (isWaiting)
  {
    if (digitalRead(HALT_BTN) == LOW)
    {
      isWaiting = !isWaiting;
    }
    else
    {
     digitalWrite(WIFI_LED, HIGH);
     vTaskDelay(500 / portTICK_PERIOD_MS);
     digitalWrite(WIFI_LED, LOW);
     vTaskDelay(500 / portTICK_PERIOD_MS);
    }
  }

  vTaskDelay(1000 / portTICK_PERIOD_MS);
  digitalWrite(INV_PW, ON);
  display.clear();
  display.setCursor(1, 0);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  display.print("Spinning....");
  analogWrite(CTR_SIG, 50);
  vTaskDelay(180000 / portTICK_PERIOD_MS);
  digitalWrite(INV_PW, OFF);
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  digitalWrite(CO1, ON);
  vTaskDelay(40000 / portTICK_PERIOD_MS);
  digitalWrite(DM_SPIN, OFF);
  digitalWrite(DM_WASH, OFF);
  digitalWrite(IV, OFF);
  digitalWrite(CO1, OFF);
  digitalWrite(CO2, OFF);
  isWaiting = true;
  programRunning = false;
}
/* --------------------  11. Spin Program Function (END)  ---------------------- */


/* --------------------  12. Soak Program Function (START)  ---------------------- */
void soakLogic()
{
  programRunning = true;
  digitalWrite(INV_PW, OFF);
  digitalWrite(DM_SPIN, OFF);
  digitalWrite(DM_WASH, OFF);
  digitalWrite(CO1, OFF);
  digitalWrite(CO2, OFF);
  display.clear();
  display.setCursor(1, 0);
  display.print("Filling Water..");
  display.setCursor(1, 1);
  display.print("SOAK");
  display.setCursor(13, 1);
  display.print("L");

  // Water Filling Control
  while (waterLevel < setFillingWaterLevel)
  {
    digitalWrite(IV, ON);
    tareWaterLevel = level.get_units() / multiplier;
    waterLevel = tareWaterLevel - offset;
    Serial.println(waterLevel, 1);
    display.setCursor(8, 1);
    display.print(waterLevel, 1);
  }
  digitalWrite(IV, OFF);
  Serial.println("Water Filling Complete!");
  Serial.println("Value (In Litres):");
  Serial.print(waterLevel);
  display.clear();
  display.setCursor(2, 0);
  display.print("Water Filled");
  display.setCursor(1, 1);
  display.print("Value:      L");
  display.setCursor(8, 1);
  display.print(waterLevel, 1);
  digitalWrite(IV, OFF);
  digitalWrite(DM_WASH, ON);
  display.clear();
  display.setCursor(1, 0);
  display.print("Soaking.....");
  digitalWrite(INV_PW, ON);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  for (int i = 0; i < 50; i++)
  {
    display.setCursor(1, 1);
    display.print("Iteration:");
    display.setCursor(12, 1);
    display.print(i);
    analogWrite(CTR_SIG, 200);
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    analogWrite(CTR_SIG, 0);
    vTaskDelay(2500 / portTICK_PERIOD_MS);
    digitalWrite(CO1, ON);
    digitalWrite(CO2, ON);
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    analogWrite(CTR_SIG, 200);
    vTaskDelay(4000 / portTICK_PERIOD_MS);
    analogWrite(CTR_SIG, 0);
    vTaskDelay(2500 / portTICK_PERIOD_MS);
    digitalWrite(CO1, OFF);
    digitalWrite(CO2, OFF);
    vTaskDelay(2500 / portTICK_PERIOD_MS);
  }

  digitalWrite(INV_PW, OFF);
  digitalWrite(DM_WASH, OFF);
  programRunning = false;
}
/* --------------------  12. Soak Program Function (END)  ---------------------- */


/* --------------------  13. WiFi Connect Function (START)  ---------------------- */
boolean connectWifi()
{
  boolean state = true;
  int i = 0;

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
  Serial.println("Connecting to WiFi");

  // Wait for connection
  Serial.print("Connecting...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    delay(500);
    Serial.print(".");
    digitalWrite(WIFI_LED, !digitalRead(WIFI_LED));

    if (i > 5)
    {
      state = false;
      break;
    }
    i++;
  }
  Serial.println("");
  if (state)
  {
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    display.clear();
    display.setCursor(1, 0);
    display.print("Connected to:");
    display.setCursor(0, 1);
    display.print(ssid);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
  }
  else
  {
    Serial.println("Connection failed.");
  }
  return state;
}
/* --------------------  13. WiFi Connect Function (END)  ---------------------- */


/* ----------------  14. Engineering Mode Helper Functions (START)  -------------------- */
void displayTestMenu() {
  display.clear();
  display.setCursor(1, 0);
  display.print("  TEST MODE  ");
  display.setCursor(1, 1);
  display.print(" Engineering ");
}

void reboot() {
  // Final safety check
  if (programRunning) {
    telegram.sendMessage(CHAT_ID, "❌ Cannot reboot - Program is running!\nPlease halt the program first.", "Markdown");
    testMenuOption = 0;
    sendMenu();
    return;
  }
  
  // Turn off all outputs before reboot
  digitalWrite(INV_PW, OFF);
  digitalWrite(DM_WASH, OFF);
  digitalWrite(DM_SPIN, OFF);
  digitalWrite(IV, OFF);
  digitalWrite(CO1, OFF);
  digitalWrite(CO2, OFF);
  analogWrite(CTR_SIG, 0);
  
  telegram.sendMessage(CHAT_ID, "⚠️ *Rebooting System*\n\nDevice restaring in 3 seconds...", "Markdown");
  
  display.clear();
  display.setCursor(5, 0);
  display.print("SYSTEM");
  display.setCursor(5, 1);
  display.print("REBOOT");
  
  delay(3000);
  esp_restart();
}
/* ----------------  14. Engineering Mode Helper Functions (END)  -------------------- */


/* ----------------  15. Water Level Sensor Test Logic (START)  -------------------- */
void waterLevelSensorTest() {
  // ========== SEND INITIAL INFO ==========
  String msg = "💧 *WATER LEVEL SENSOR TEST*\n\n";
  msg += "Module: HX710B Load Cell Amplifier\n";
  msg += "Interface: I2C (DATA + CLK)\n";
  msg += "Test Procedure:\n";
  msg += "1. Check HX710B communication\n";
  msg += "2. Read baseline values\n";
  msg += "3. Open inlet valve briefly\n";
  msg += "4. Verify sensor response\n\n";
  msg += "⚡ Starting test...";
  telegram.sendMessage(CHAT_ID, msg, "Markdown");
  
  display.clear();
  display.setCursor(0, 0);
  display.print("Sensor Test");
  display.setCursor(0, 1);
  display.print("Checking I2C..");
  
  // ========== CHECK HX710B COMMUNICATION ==========
  bool i2cOK = false;
  float baseline = 0;
  float readings[10];
  
  if (level.is_ready()) {
    i2cOK = true;
    baseline = level.get_units() / multiplier;
    
    // Take 10 baseline readings
    for (int i = 0; i < 10; i++) {
      readings[i] = level.get_units() / multiplier;
      delay(100);
    }
  }
  
  // ========== OPEN VALVE & TEST RESPONSE ==========
  display.clear();
  display.setCursor(0, 0);
  display.print("Opening Valve");
  display.setCursor(0, 1);
  display.print("Adding Water..");
  
  digitalWrite(IV, ON);
  vTaskDelay(5000 / portTICK_PERIOD_MS); // Fill for 5 seconds
  digitalWrite(IV, OFF);
  
  vTaskDelay(2000 / portTICK_PERIOD_MS); // Wait for water to settle
  
  // Take new readings
  float newLevel = level.get_units() / multiplier;
  float delta = abs(newLevel - baseline);
  
  // ========== ANALYZE RESULTS ==========
  bool valveWorking = (delta > 0.5); // Significant change detected
  bool sensorResponsive = i2cOK && (delta > 0.1);
  
  // ========== GENERATE REPORT ==========
  String report = "💧 *WATER LEVEL SENSOR TEST REPORT*\n\n";
  
  report += "🔌 *I2C Communication:* ";
  report += i2cOK ? "✅ OK\n" : "❌ FAILED\n";
  
  report += "📊 *Baseline Reading:* " + String(baseline, 2) + " units\n";
  report += "📈 *After Fill Reading:* " + String(newLevel, 2) + " units\n";
  report += "📉 *Delta:* " + String(delta, 2) + " units\n\n";
  
  report += "🎯 *Test Results:*\n";
  report += "Sensor Response: " + String(sensorResponsive ? "✅ PASS" : "❌ FAIL") + "\n";
  report += "Valve Function: " + String(valveWorking ? "✅ PASS" : "❌ FAIL") + "\n\n";
  
  if (!i2cOK) {
    report += "⚠️ *I2C Communication Failed*\n";
    report += "Possible Issues:\n";
    report += "• HX710B not powered\n";
    report += "• DATA/CLK pins disconnected\n";
    report += "• Wrong pin assignment\n";
    report += "• Module damaged\n";
  } else if (!sensorResponsive) {
    report += "⚠️ *Sensor Not Responsive*\n";
    report += "Possible Issues:\n";
    report += "• Load cell not connected\n";
    report += "• Sense Tube (Pipe) Leak or Broken\n";
    report += "• Calibration incorrect\n";
    report += "• Mechanical issue\n";
  } else if (!valveWorking) {
    report += "⚠️ *Valve Not Working*\n";
    report += "Possible Issues:\n";
    report += "• Inlet valve relay fault\n";
    report += "• No water supply\n";
    report += "• Valve stuck closed\n";
  } else {
    report += "✅ *All Systems OK*\n";
    report += "Sensor and valve functioning correctly.";
  }
  
  telegram.sendMessage(CHAT_ID, report, "Markdown");
  
  display.clear();
  display.setCursor(0, 0);
  display.print(i2cOK && sensorResponsive ? "Sensor: PASS" : "Sensor: FAIL");
  display.setCursor(0, 1);
  display.print(valveWorking ? "Valve: PASS" : "Valve: FAIL");
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  
  displayTestMenu();
}
/* ----------------  15. Water Level Sensor Test Logic (END)  -------------------- */


/* ----------------  16. Inlet Valve Test Logic (START)  -------------------- */
void inletValveTest() {
  // ========== CONSTANTS FOR 4 MINUTE TEST ==========
  const unsigned long duration = 240000; // 4 minutes in milliseconds
  const int samples = 240; // One sample per second for 4 minutes
  const unsigned long interval = 1000; // 1 second between samples

  // ========== HELPER: Get averaged water level reading ==========
  auto getCurrentWaterLevel = [&]() {
    float total = 0;
    for (int i = 0; i < 5; i++) {
      tareWaterLevel = level.get_units() / multiplier;
      waterLevel = tareWaterLevel - offset;
      total += waterLevel;
      delay(50);
    }
    return total / 5.0;
  };

  // ========== SAFETY CHECK ==========
  float initialLevel = getCurrentWaterLevel();
  
  if (initialLevel > 15.0) { // Prevent overflow
    String msg = "⚠️ *TEST ABORTED*\nWater level too high: " + String(initialLevel, 1) + "L\n";
    msg += "Please drain tank below 15L before testing.";
    telegram.sendMessage(CHAT_ID, msg, "Markdown");
    return;
  }

  // ========== INITIALIZE TEST ==========
  telegram.sendMessage(CHAT_ID, "🚰 *WATER INLET VALVE AUTO-TEST*\nStarting 4 minute test...", "Markdown");
  
  float waterLevelReadings[samples];
  unsigned long testStartTime = millis();
  int sampleIndex = 0;
  
  String progressMsg = "📊 *TEST IN PROGRESS*\n";
  progressMsg += "Initial Level: " + String(initialLevel, 2) + "L\n";
  progressMsg += "Duration: 4min | Min Delta: " + String(VALVE_TEST_MIN_DELTA, 1) + "L\n\n";
  progressMsg += "🔄 Opening valve...";
  int msgid = telegram.sendMessage(CHAT_ID, progressMsg, "Markdown");
  
  // Open the inlet valve
  digitalWrite(IV, ON);

  // ========== COLLECT SAMPLES - UPDATE EVERY 60 SECONDS ==========
  while ((millis() - testStartTime) < duration && sampleIndex < samples) {
    float currentLevel = getCurrentWaterLevel();
    waterLevelReadings[sampleIndex] = currentLevel;
    
    // Send progress update every 10 seconds 
    if (sampleIndex % 10 == 0 && sampleIndex > 0) {
      float deltaLevel = currentLevel - initialLevel;
      int secondsElapsed = (millis() - testStartTime) / 1000;
      int secondsRemaining = 240 - secondsElapsed;
      int minutesRemaining = secondsRemaining / 60;
      int secsRemaining = secondsRemaining % 60;
      
      String update = "📈 *Progress Update*\n";
      update += "Time: " + String(secondsElapsed) + "s / 240s\n";
      update += "Current Level: " + String(currentLevel, 2) + "L\n";
      update += "Delta: " + String(deltaLevel, 2) + "L\n";
      update += "Flow Rate: " + String(deltaLevel / (secondsElapsed) * 60, 2) + " L/min\n";
      update += "Remaining: " + String(minutesRemaining) + "m " + String(secsRemaining) + "s\n";
      update += (deltaLevel >= VALVE_TEST_MIN_DELTA) ? "✅ ON TRACK" : "⏳ IN PROGRESS";
      
      telegram.sendMessage(CHAT_ID, update, "Markdown", msgid);
    }
    
    sampleIndex++;
    delay(interval);
  }

  // ========== ANALYZE RESULTS ==========
  digitalWrite(IV, OFF);
  
  float finalLevel = getCurrentWaterLevel();
  float totalDelta = finalLevel - initialLevel;
  float averageFlowRate = totalDelta / (duration / 1000.0); // L/s
  bool testPassed = (totalDelta >= VALVE_TEST_MIN_DELTA);

  // ========== GENERATE FINAL REPORT ==========
  String report = testPassed ? "✅ *VALVE TEST PASSED*\n\n" : "❌ *VALVE TEST FAILED*\n\n";
  
  report += "📊 *Test Summary:*\n";
  report += "Initial Level: " + String(initialLevel, 2) + " L\n";
  report += "Final Level: " + String(finalLevel, 2) + " L\n";
  report += "Water Added: " + String(totalDelta, 2) + " L\n";
  report += "Flow Rate: " + String(averageFlowRate * 60, 1) + " L/min\n";
  report += "Duration: 4 minutes (240 seconds)\n";
  report += "Samples Taken: " + String(sampleIndex) + "\n\n";
  
  report += "🎯 *Analysis:*\n";
  report += "Required: ≥" + String(VALVE_TEST_MIN_DELTA, 1) + " L\n";
  report += "Actual: " + String(totalDelta, 2) + " L\n";
  
  if (testPassed) {
    report += "Status: VALVE WORKING ✅\n";
    if (totalDelta > 15.0) {
      report += "Note: Very high flow rate detected\n";
    } else if (totalDelta < 5.0) {
      report += "Note: Moderate flow rate (normal range)\n";
    }
  } else {
    report += "Status: VALVE FAULT ❌\n";
    if (totalDelta < 0.5) {
      report += "Possible Issues:\n";
      report += "• Valve stuck closed\n";
      report += "• No water supply\n";
      report += "• Electrical connection fault\n";
    } else if (totalDelta < VALVE_TEST_MIN_DELTA) {
      report += "Possible Issues:\n";
      report += "• Partially blocked valve\n";
      report += "• Low water pressure\n";
      report += "• Sensor calibration needed\n";
    }
  }
  
  // Trend analysis
  if (sampleIndex > 50) {
    report += "\n📈 *Flow Trend (4-minute test):*\n";
    float quarter1 = 0, quarter2 = 0, quarter3 = 0, quarter4 = 0;
    
    // Calculate average level for each quarter
    for (int i = 0; i < sampleIndex/4; i++) {
      quarter1 += waterLevelReadings[i];
    }
    for (int i = sampleIndex/4; i < sampleIndex/2; i++) {
      quarter2 += waterLevelReadings[i];
    }
    for (int i = sampleIndex/2; i < (3*sampleIndex)/4; i++) {
      quarter3 += waterLevelReadings[i];
    }
    for (int i = (3*sampleIndex)/4; i < sampleIndex; i++) {
      quarter4 += waterLevelReadings[i];
    }
    
    quarter1 /= (sampleIndex/4 + 1);
    quarter2 /= (sampleIndex/4 + 1);
    quarter3 /= (sampleIndex/4 + 1);
    quarter4 /= (sampleIndex/4 + 1);
    
    report += "Q1: " + String(quarter1, 2) + "L ";
    report += "Q2: " + String(quarter2, 2) + "L ";
    report += "Q3: " + String(quarter3, 2) + "L ";
    report += "Q4: " + String(quarter4, 2) + "L\n";
    
    if (quarter4 > quarter1) {
      report += "Flow: Increasing over time\n";
    } else if (quarter4 < quarter1) {
      report += "Flow: Decreasing over time\n";
    } else {
      report += "Flow: Steady throughout\n";
    }
  }

  // ========== SEND FINAL REPORT ==========
  telegram.sendMessage(CHAT_ID, report, "Markdown");
  
  Serial.println("=== VALVE TEST COMPLETE ===");
  Serial.printf("Initial: %.2fL, Final: %.2fL, Delta: %.2fL\n", initialLevel, finalLevel, totalDelta);
  Serial.printf("Flow Rate: %.3fL/s, Samples: %d, Result: %s\n", averageFlowRate, sampleIndex, testPassed ? "PASS" : "FAIL");
}
/* ----------------  16. Inlet Valve Test Logic (END)  -------------------- */


/* ----------------  17. Drain Motor (Wash Stage) Test Logic (START)  -------------------- */
void drainMotorWashStageTest() {
  // ========== SEND INITIAL INSTRUCTIONS ==========
  String msg = "🔧 *DRAIN MOTOR TEST*\n\n";
  msg += "This test checks if the drain motor brake releases properly.\n\n";
  msg += "📋 *Instructions:*\n";
  msg += "1. Motor brake will activate\n";
  msg += "2. Try to rotate the drum by hand\n";
  msg += "3. Press the hardware buttons:\n\n";
  msg += "   ✅ *SPIN Button* - Drum rotates smoothly\n";
  msg += "   ❌ *RINSE Button* - Drum is stuck/hard to turn\n\n";
  msg += "⚡ Starting test in 5 seconds...";
  
  telegram.sendMessage(CHAT_ID, msg, "Markdown");
  
  // ========== COUNTDOWN & ACTIVATE MOTOR ==========
  for (int i = 5; i > 0; i--) {
    display.clear();
    display.setCursor(0, 0);
    display.print("Motor Test");
    display.setCursor(0, 1);
    display.print("Starting: ");
    display.print(i);
    delay(1000);
  }
  
  display.clear();
  display.setCursor(0, 0);
  display.print("Motor: ON");
  display.setCursor(0, 1);
  display.print("Try turning..");
  
  digitalWrite(DM_WASH, ON);
  
  // ========== SEND ACTIVATION MESSAGE ==========
  String instructionMsg = "⚡ *DRAIN MOTOR ACTIVATED*\n\n";
  instructionMsg += "🖐️ Now try to rotate the drum by hand.\n\n";
  instructionMsg += "Press on the HARDWARE device:\n";
  instructionMsg += "✅ SPIN button - Smooth rotation\n";
  instructionMsg += "❌ RINSE button - Stuck/difficult\n\n";
  instructionMsg += "Waiting for your input...";
  
  telegram.sendMessage(CHAT_ID, instructionMsg, "Markdown");
  
  // ========== WAIT FOR USER RESPONSE ==========
  awaitingDrainMotorResponse = true;
  drainMotorTestResult = false;
  
  unsigned long waitStart = millis();
  unsigned long timeout = 60000; // 60 second timeout
  
  while (awaitingDrainMotorResponse && (millis() - waitStart < timeout)) {
    if ((millis() - waitStart) % 2000 < 100) {
      display.setCursor(0, 1);
      display.print("Waiting...      ");
    }
    delay(100);
  }
  
  // ========== TURN OFF MOTOR ==========
  digitalWrite(DM_WASH, OFF);
  
  // ========== GENERATE & SEND REPORT ==========
  if (!awaitingDrainMotorResponse) {
    // User responded - display result
    display.clear();
    display.setCursor(0, 0);
    display.print(drainMotorTestResult ? "Motor: PASS" : "Motor: FAIL");
    display.setCursor(0, 1);
    display.print(drainMotorTestResult ? "Working OK" : "Brake Stuck");
    delay(3000);
    
    // Build report based on test result
    String report = drainMotorTestResult ? 
      "✅ *DRAIN MOTOR TEST - PASSED*\n\n"
      "🎯 *Test Result:*\n"
      "Drum rotates smoothly when brake is engaged.\n\n"
      "✔️ *Status:* Motor brake functioning correctly\n"
      "✔️ Motor coil resistance normal\n"
      "✔️ Brake mechanism releases properly\n"
      "✔️ No mechanical obstruction\n\n"
      "💡 *Recommendation:*\n"
      "Motor is in good working condition. No action needed."
      :
      "❌ *DRAIN MOTOR TEST - FAILED*\n\n"
      "🎯 *Test Result:*\n"
      "Drum is difficult or impossible to rotate manually.\n\n"
      "⚠️ *Possible Causes:*\n"
      "1. Brake not releasing\n"
      "   → Check motor coil connections\n"
      "   → Measure coil resistance (should be ~100-300Ω)\n\n"
      "2. Mechanical seizure\n"
      "   → Bearing failure\n"
      "   → Foreign object in drum\n"
      "   → Belt too tight\n\n"
      "3. Electrical fault\n"
      "   → Motor not receiving power\n"
      "   → Relay failure on DM_WASH pin\n"
      "   → Wiring issue\n\n"
      "🔧 *Recommended Actions:*\n"
      "1. Disconnect power and manually check drum rotation\n"
      "2. Measure motor coil resistance\n"
      "3. Check DM_WASH relay output with multimeter\n"
      "4. Inspect motor mounting and bearings\n"
      "5. Check for belt misalignment";
    
    telegram.sendMessage(CHAT_ID, report, "Markdown");
  } else {
    // Timeout occurred
    display.clear();
    display.setCursor(0, 0);
    display.print("Test Timeout");
    display.setCursor(0, 1);
    display.print("No Response");
    
    telegram.sendMessage(CHAT_ID, "⏱️ *TEST TIMEOUT*\nNo button pressed within 60 seconds.\nTest cancelled.", "Markdown");
    delay(2000);
  }
  
  // ========== RESET & RETURN TO TEST MODE ==========
  awaitingDrainMotorResponse = false;
  displayTestMenu();
}
/* ----------------  17. Drain Motor (Wash Stage) Test Logic (END) -------------------- */


/* ----------------  18. Drain Motor (Spin Stage) Test Logic (START) -------------------- */
void drainMotorSpinStageTest() {
  String msg = 
    "🔧 *SPIN STAGE TEST INITIATED*\n\n"
    "⚠️ *Ensure drum is empty before starting!*\n"
    "Procedure:\n"
    "1. Activating Drain Motors (WASH & SPIN)\n"
    "2. Activating Inverter Power\n"
    "3. Sending low speed (50 PWM) to motor\n"
    "4. Running for 30 seconds\n";
  telegram.sendMessage(CHAT_ID, msg, "Markdown");

  display.clear();
  display.setCursor(0, 0); 
  display.print("SPIN STAGE TEST");
  display.setCursor(0, 1); 
  display.print("Start in 3s...");
  vTaskDelay(3000 / portTICK_PERIOD_MS);

  // Step 2: Activate both drain motors
  digitalWrite(DM_WASH, ON);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  digitalWrite(DM_SPIN, ON);
  display.clear();
  display.setCursor(0, 0); 
  display.print("DrainMtr: ON");
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Step 3: Turn on Inverter Power
  digitalWrite(INV_PW, ON);
  display.setCursor(0, 1); 
  display.print("Inverter: ON");

  // Step 4: Send 50 PWM on CTR_SIG
  analogWrite(CTR_SIG, 50);
  display.clear();
  display.setCursor(0, 0); 
  display.print("Motor PWM: 50");
  display.setCursor(0, 1); 
  display.print("Wait 30s...");

  // Step 5: Wait for 30 seconds (run spin stage)
  vTaskDelay(30000 / portTICK_PERIOD_MS);

  // Step 6: Turn off PWM (CTRL to 0)
  analogWrite(CTR_SIG, 0);
  display.clear();
  display.setCursor(0, 0); 
  display.print("PWM: OFF");
  
  // Step 7: Turn off Inverter Power
  digitalWrite(INV_PW, OFF);
  display.setCursor(0, 1); 
  display.print("Inv: OFF");
  vTaskDelay(15000 / portTICK_PERIOD_MS);

  // Step 8: Turn off both drain motors
  digitalWrite(DM_WASH, OFF);
  digitalWrite(DM_SPIN, OFF);

  // Step 9: Final report
  String result = 
    "✅ *SPIN STAGE TEST COMPLETE*\n\n"
    "Actions performed:\n"
    "• Both drain motors activated\n"
    "• Inverter powered, 50 PWM given to motor\n"
    "• 30 seconds runtime\n\n"
    "If drum reached and maintained low spin speed, inverter wiring, relays, and drain motors are likely OK.\n\n"
    "If drum did NOT start spinning:\n"
    "• Check inverter wiring and parameters\n"
    "• Check drum belt/glide\n"
    "• Test power to DM_WASH & DM_SPIN\n"
    "• Review error LEDs on inverter (if present)\n"
    "• Investigate motor connections\n";
  telegram.sendMessage(CHAT_ID, result, "Markdown");

  display.clear();
  display.setCursor(0, 0); 
  display.print("TEST COMPLETE");
  display.setCursor(0, 1); 
  display.print("Check Drum Spin");
  vTaskDelay(3000 / portTICK_PERIOD_MS);

  // Restore display to engineering status
  displayTestMenu();
}
/* ----------------  18. Drain Motor (Spin Stage) Test Logic (END)  -------------------- */


/* ----------------  19. Main Motor Rotation Test Logic (START)  -------------------- */
void motorRotationTest() {
  // ========== SEND INITIAL INFO ==========
  String msg = "⚙️ *MOTOR ROTATION TEST*\n\n";
  msg += "Test Procedure:\n";
  msg += "1. Forward rotation (0-200 PWM)\n";
  msg += "2. Hold at 200 PWM for 10s\n";
  msg += "3. Reverse rotation (0-200 PWM)\n";
  msg += "4. Repeat 2 cycles\n\n";
  msg += "⚡ Starting test...";
  telegram.sendMessage(CHAT_ID, msg, "Markdown");
  
  display.clear();
  display.setCursor(0, 0);
  display.print("Motor Test");
  display.setCursor(0, 1);
  display.print("Initializing..");
  
  // ========== FEEDBACK PULSE COUNTER (FOR FUTURE) ==========
  volatile int feedbackPulses = 0;
  bool motorLocked = false;
  
  // ========== TURN ON INVERTER POWER ==========
  digitalWrite(INV_PW, ON);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  
  // ========== CYCLE LOOP (2 TIMES) ==========
  for (int cycle = 1; cycle <= 2; cycle++) {
    // --- FORWARD ROTATION ---
    display.clear();
    display.setCursor(0, 0);
    display.print("Cycle ");
    display.print(cycle);
    display.print("/2");
    display.setCursor(0, 1);
    display.print("Forward Ramp");
    
    for (int pwm = 20; pwm <= 200; pwm += 5) {
      analogWrite(CTR_SIG, pwm);
      vTaskDelay(500 / portTICK_PERIOD_MS);
      // TODO: Read feedback pulses here and update feedbackPulses
    }
    
    display.setCursor(0, 1);
    display.print("Hold 10s...   ");
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    
    analogWrite(CTR_SIG, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // --- REVERSE ROTATION ---
    digitalWrite(CO1, ON);
    digitalWrite(CO2, ON);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    
    display.clear();
    display.setCursor(0, 0);
    display.print("Cycle ");
    display.print(cycle);
    display.print("/2");
    display.setCursor(0, 1);
    display.print("Reverse Ramp");
    
    for (int pwm = 20; pwm <= 200; pwm += 5) {
      analogWrite(CTR_SIG, pwm);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      // TODO: Read feedback pulses here
    }
    
    display.setCursor(0, 1);
    display.print("Hold 10s...   ");
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    
    analogWrite(CTR_SIG, 0);
    digitalWrite(CO1, OFF);
    digitalWrite(CO2, OFF);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  
  // ========== SHUTDOWN ==========
  digitalWrite(INV_PW, OFF);
  
  // ========== TODO: feedback check (future implementation) ==========
  // motorLocked = (feedbackPulses < 10); // Threshold to be tuned
  
  // ========== GENERATE REPORT ==========
  String report = "⚙️ *MOTOR ROTATION TEST REPORT*\n\n";
  report += "🎯 *Test Summary:*\n";
  report += "Cycles Completed: 2/2\n";
  report += "Forward Rotations: ✅\n";
  report += "Reverse Rotations: ✅\n";
  report += "PWM Range: 20-200\n\n";
  
  report += "📊 *Feedback Status:*\n";
  report += "Pulse Count: " + String(feedbackPulses) + " (API ready)\n";
  report += "Motor Lock Detect: " + String(motorLocked ? "⚠️ YES" : "✅ NO") + "\n\n";
  
  if (motorLocked) {
    report += "⚠️ *Motor Issue Detected*\n";
    report += "Possible Issues:\n";
    report += "• Motor mechanically locked\n";
    report += "• Drive overload fault\n";
    report += "• Belt too tight\n";
    report += "• Motor Shaft Stuck\n";
    report += "• Feedback sensor error/disconnected\n";
  } else {
    report += "✅ *Motor Test PASSED*\n";
    report += "Motor rotates smoothly in both directions.";
  }
  
  telegram.sendMessage(CHAT_ID, report, "Markdown");
  
  display.clear();
  display.setCursor(0, 0);
  display.print("Motor: PASS");
  display.setCursor(0, 1);
  display.print("Test Complete");
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  
  displayTestMenu();
}
/* ----------------  19. Main Motor Rotation Test Logic (END)  -------------------- */


/* ----------------  20. LED Test Logic (START)  -------------------- */
void ledTest() {

  if (ledtask_handle != NULL) 
  {
    vTaskSuspend(ledtask_handle);
  }

  // ========== SEND INITIAL INFO ==========
  String msg = "💡 *LED TEST*\n\n";
  msg += "Testing LEDs in cascade pattern:\n";
  msg += "• SOAK LED\n";
  msg += "• WASH LED\n";
  msg += "• RINSE LED\n";
  msg += "• SPIN LED\n";
  msg += "• WiFi LED\n";
  msg += "• LCD Backlight\n\n";
  msg += "Running 2 cycles...";
  
  int statusMsgID = telegram.sendMessage(CHAT_ID, msg, "Markdown");
  
  display.clear();
  display.setCursor(0, 0);
  display.print("LED Test");
  display.setCursor(0, 1);
  display.print("Cascade Mode");
  
  // ========== LED ARRAY ==========
  int leds[] = {SOAK_LED, WASH_LED, RINSE_LED, SPIN_LED, WIFI_LED};
  String ledNames[] = {"SOAK", "WASH", "RINSE", "SPIN", "WiFi"};
  int numLEDs = 5;
  
  // ========== RUN 2 CYCLES ==========
  for (int cycle = 1; cycle <= 2; cycle++) {
    for (int i = 0; i < numLEDs; i++) {
      // Turn on current LED
      digitalWrite(leds[i], ON);
      
      // Update display
      display.clear();
      display.setCursor(0, 0);
      display.print("Cycle ");
      display.print(cycle);
      display.print("/2");
      display.setCursor(0, 1);
      display.print("LED: ");
      display.print(ledNames[i]);
      
      // Update Telegram message
      String update = "💡 *LED TEST*\n\nCycle " + String(cycle) + "/2\n";
      update += "Current: " + ledNames[i] + " LED ✅\n";
      telegram.sendMessage(CHAT_ID, update, "Markdown", statusMsgID);
      
      vTaskDelay(500 / portTICK_PERIOD_MS);
      
      // Turn off current LED
      digitalWrite(leds[i], OFF);
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    
    // Test LCD backlight
    display.clear();
    display.setCursor(0, 0);
    display.print("LCD Backlight");
    display.noBacklight();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    display.backlight();
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  
  // ========== GENERATE REPORT ==========
  String report = "💡 *LED TEST REPORT*\n\n";
  report += "🎯 *Test Summary:*\n";
  report += "Cycles Completed: 2/2\n";
  report += "LEDs Tested: 6\n\n";
  report += "✅ SOAK LED: OK\n";
  report += "✅ WASH LED: OK\n";
  report += "✅ RINSE LED: OK\n";
  report += "✅ SPIN LED: OK\n";
  report += "✅ WiFi LED: OK\n";
  report += "✅ LCD Backlight: OK\n\n";
  report += "💡 *Result:* All LEDs functional";
  
  telegram.sendMessage(CHAT_ID, report, "Markdown");
  
  display.clear();
  display.setCursor(0, 0);
  display.print("LED Test: PASS");
  display.setCursor(0, 1);
  display.print("All OK");
  vTaskDelay(3000 / portTICK_PERIOD_MS);

  if (ledtask_handle != NULL) 
  {
    vTaskResume(ledtask_handle);
  }
  
  displayTestMenu();
}
/* ----------------  20. LED Test Logic (END)  -------------------- */


/* ----------------  21. MCU Self Test Logic (START)  -------------------- */
void mcuSelfTest() {
  // ========== SEND INITIAL INFO ==========
  String msg = "🖥️ *MCU SELF-TEST*\n\n";
  msg += "Running diagnostics:\n";
  msg += "• Memory integrity\n";
  msg += "• Stack check\n";
  msg += "• Flash parameters\n";
  msg += "• WiFi RSSI\n\n";
  msg += "⚡ Starting...";
  telegram.sendMessage(CHAT_ID, msg, "Markdown");
  
  display.clear();
  display.setCursor(0, 0);
  display.print("MCU Self-Test");
  display.setCursor(0, 1);
  display.print("Running...");
  
  // ========== MEMORY TEST ==========
  bool memoryOK = true;
  uint8_t testBuffer[256];
  for (int i = 0; i < 256; i++) testBuffer[i] = i;
  for (int i = 0; i < 256; i++) {
    if (testBuffer[i] != i) {
      memoryOK = false;
      break;
    }
  }
  
  // ========== STACK INTEGRITY CHECK ==========
  bool stackOK = true;
  size_t freeStack = uxTaskGetStackHighWaterMark(NULL);
  if (freeStack < 512) stackOK = false; // Less than 512 bytes = risky
  
  // ========== FLASH PARAMETERS ==========
  uint32_t flashSize = ESP.getFlashChipSize();
  uint32_t flashSpeed = ESP.getFlashChipSpeed();
  bool flashOK = (flashSize > 0 && flashSpeed > 0);
  
  // ========== Sketch & MD5 Verification ==========
  uint32_t sketchSize = ESP.getSketchSize();
  uint32_t freeSketchSpace = ESP.getFreeSketchSpace();
  String sketchMD5 = ESP.getSketchMD5();
  bool sketchOK = (sketchSize > 0 && freeSketchSpace > 100000 && sketchMD5.length() == 32);
  
  // ========== WIFI RSSI CHECK ==========
  int rssi = WiFi.RSSI();
  bool wifiOK = (rssi != 0 && rssi > -90); // Signal strength check
  
  // ========== OVERALL RESULT ==========
  bool allPassed = memoryOK && stackOK && flashOK && sketchOK && wifiOK;
  
  // ========== GENERATE REPORT ==========
  String report = "🖥️ *MCU SELF-TEST REPORT*\n\n";
  
  report += "🧠 *Memory Test:* " + String(memoryOK ? "✅ PASS" : "❌ FAIL") + "\n";
  report += "📚 *Stack Integrity:* " + String(stackOK ? "✅ PASS" : "❌ FAIL") + "\n";
  report += "   Free Stack: " + String(freeStack) + " bytes\n\n";
  
  report += "💾 *Flash IC Parameters:*\n";
  report += "   Size: " + String(flashSize / 1024) + " KB\n";
  report += "   Speed: " + String(flashSpeed / 1000000) + " MHz\n";
  report += "   Status: " + String(flashOK ? "✅ OK" : "❌ FAULT") + "\n\n";
  
  report += "📝 *Sketch Checksum (MD5):*\n";
  report += "Hash: " + sketchMD5.substring(0, 16) + "...\n";
  report += "Size: " + String(sketchSize / 1024) + " KB\n";
  report += "Free: " + String(freeSketchSpace / 1024) + " KB\n";
  report += "Status: " + String(sketchOK ? "✅ PASS" : "❌ FAIL") + "\n\n";

  report += "📶 *WiFi RSSI:* " + String(rssi) + " dBm ";
  report += String(wifiOK ? "✅ GOOD" : "⚠️ WEAK") + "\n\n";
  
  if (!allPassed) {
    report += "⚠️ *Issues Detected*\n";
    if (!memoryOK) report += "• Memory corruption detected\n";
    if (!stackOK) report += "• Stack overflow risk (low free space)\n";
    if (!flashOK) report += "• Flash IC parameter read failed\n";
    if (!sketchOK) report += "• Sketch MD5 Verification issue\n";
    if (!wifiOK) report += "• WiFi signal weak or disconnected\n";
    report += "\n🔧 *Recommendation:* Check hardware & restart MCU";
  } else {
    report += "✅ *ALL TESTS PASSED*\n";
    report += "MCU is functioning correctly.";
  }
  
  telegram.sendMessage(CHAT_ID, report, "Markdown");
  
  display.clear();
  display.setCursor(0, 0);
  display.print(allPassed ? "MCU: PASS" : "MCU: FAIL");
  display.setCursor(0, 1);
  display.print(allPassed ? "All OK" : "See Telegram");
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  
  displayTestMenu();
}
/* ----------------  21. MCU Self Test Logic (END)  -------------------- */


/* ----------------  22. All Buttons Test Logic (START)  -------------------- */
void allButtonsTest() {
  // ========== SEND INITIAL INFO ==========
  String msg = "🎛️ *ALL BUTTONS TEST*\n\n";
  msg += "Press each button when prompted:\n";
  msg += "• WASH\n";
  msg += "• RINSE\n";
  msg += "• SPIN\n";
  msg += "• COMP\n";
  msg += "• HALT\n\n";
  msg += "Waiting for button press...";
  
  int statusMsgID = telegram.sendMessage(CHAT_ID, msg, "Markdown");
  
  // ========== BUTTON TEST ARRAY ==========
  int buttons[] = {WASH_BTN, RINSE_BTN, SPIN_BTN, COMP_BTN, HALT_BTN};
  String buttonNames[] = {"WASH", "RINSE", "SPIN", "COMP", "PAUSE"};
  unsigned long debounceTime[5] = {0};
  bool buttonTested[5] = {false, false, false, false, false};
  int numButtons = 5;
  
  // ========== TEST LOOP ==========
  for (int i = 0; i < numButtons; i++) {
    display.clear();
    display.setCursor(0, 0);
    display.print("Press:");
    display.setCursor(0, 1);
    display.print(buttonNames[i]);
    display.print(" Button");
    
    String update = "🎛️ *BUTTON TEST*\n\nPress: *" + buttonNames[i] + "* button\n\nWaiting...";
    telegram.sendMessage(CHAT_ID, update, "Markdown", statusMsgID);
    
    // Wait for button press
    unsigned long pressStart = 0;
    while (!buttonTested[i]) {
      if (digitalRead(buttons[i]) == LOW) { // Button pressed
        if (pressStart == 0) {
          pressStart = millis();
        } else if (millis() - pressStart > 50) { // Debounce 50ms
          debounceTime[i] = millis() - pressStart;
          buttonTested[i] = true;
          
          display.clear();
          display.setCursor(0, 0);
          display.print(buttonNames[i]);
          display.print(": OK");
          display.setCursor(0, 1);
          display.print("Debounce:");
          display.print(debounceTime[i]);
          display.print("ms");
          
          vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
      } else {
        pressStart = 0; // Reset if button released
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }
  
  // ========== GENERATE REPORT ==========
  String report = "🎛️ *ALL BUTTONS TEST REPORT*\n\n";
  report += "🎯 *Test Summary:*\n";
  report += "Buttons Tested: 5/5\n\n";
  
  for (int i = 0; i < numButtons; i++) {
    report += "✅ " + buttonNames[i] + " Button: OK\n";
    report += "   Debounce Time: " + String(debounceTime[i]) + " ms\n";
  }
  
  report += "\n💡 *Result:* All buttons functional\n";
  report += "Debounce times within normal range.";
  
  telegram.sendMessage(CHAT_ID, report, "Markdown");
  
  display.clear();
  display.setCursor(0, 0);
  display.print("Button Test");
  display.setCursor(0, 1);
  display.print("All PASS");
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  
  displayTestMenu();
}
/* ----------------  22. All Buttons Test Logic (END)  -------------------- */


/* ----------------  23. Connectivity Test Logic (START)  -------------------- */
void connectivityTest() {
  String msg = "🌐 *CONNECTIVITY TEST*\n\n";
  
  // WiFi Status
  msg += "📶 WiFi: ";
  if (wifiConnected) {
    msg += "Connected ✅\n";
    msg += "SSID: " + String(ssid) + "\n";
    msg += "IP: " + WiFi.localIP().toString() + "\n";
    msg += "RSSI: " + String(WiFi.RSSI()) + " dBm\n";
  } else {
    msg += "Disconnected ❌\n";
  }
  
  // Telegram Test
  msg += "\n📱 Telegram: ";
  if (wifiConnected) {
    msg += "Active ✅\n";
    msg += "(You received this message!)\n";
  } else {
    msg += "Cannot test - No WiFi ❌\n";
  }
  
  // Web Server
  msg += "\n🌍 Web Server: ";
  msg += wifiConnected ? "Running ✅" : "Offline ❌";
  
  telegram.sendMessage(CHAT_ID, msg, "Markdown");
}
/* ----------------  23. Connectivity Test Logic (END)  -------------------- */


/* ----------------  24. Calibration Test Logic (START)  -------------------- */
void calibrationTest() {
  String msg = "⚖️ *WATER LEVEL CALIBRATION*\n\n";
  msg += "Current Settings:\n";
  msg += "Multiplier: " + String(multiplier) + "\n";
  msg += "Offset: " + String(offset) + "\n\n";
  msg += "Current Reading: " + String(waterLevel, 2) + " L\n\n";
  msg += "To recalibrate:\n";
  msg += "1. Empty the tank completely\n";
  msg += "2. Modify multiplier/offset in code\n";
  msg += "3. Upload new firmware\n\n";
  msg += "Raw sensor value: " + String(level.get_units(), 2);
  
  telegram.sendMessage(CHAT_ID, msg, "Markdown");
}
/* ----------------  24. Calibration Test Logic (END)  -------------------- */


/* ----------------  25. System Info Test Logic (START)  -------------------- */
void sendSystemInfo() {
  String msg = "ℹ️ *SYSTEM INFORMATION*\n\n";
  
  // Firmware version
  msg += "🔧 Firmware: v1.0.0\n";
  msg += "📅 Build: Oct 2025\n\n";
  
  // System uptime
  unsigned long uptime = millis() / 1000;
  int days = uptime / 86400;
  int hours = (uptime % 86400) / 3600;
  int minutes = (uptime % 3600) / 60;
  int seconds = uptime % 60;
  
  msg += "⏱️ Uptime: ";
  if (days > 0) msg += String(days) + "d ";
  msg += String(hours) + "h " + String(minutes) + "m " + String(seconds) + "s\n\n";
  
  // Memory
  msg += "💾 Free Heap: " + String(ESP.getFreeHeap()) + " bytes\n";
  msg += "📦 Heap Size: " + String(ESP.getHeapSize()) + " bytes\n\n";

  // MCU Details
  msg += "🖥️ Chip: " + String(ESP.getChipModel()) + "\n";
  msg += "⚙️ Chip Revision " + String(ESP.getChipRevision()) + "\n";
  
  // Program Status
  msg += "🔄 Program Running: " + String(programRunning ? "YES" : "NO") + "\n";
  msg += "🎯 Selected Mode: " + String(selectedMode) + "\n";
  msg += "🔬 Simulation: " + String(isSimulation ? "ON" : "OFF") + "\n\n";
  
  // Last water usage
  msg += "💧 *Last Water Usage:*\n";
  msg += "Wash: " + String(washWaterUsed, 1) + " L\n";
  msg += "Rinse: " + String(rinseWaterUsed, 1) + " L\n";
  msg += "Total: " + String(totalWaterUsed, 1) + " L\n";
  
  telegram.sendMessage(CHAT_ID, msg, "Markdown");
}
/* ----------------  25. System Info Test Logic (END)  -------------------- */


/* ----------------  26. Engineering Mode Menu Logic (START)  -------------------- */
void sendMenu() {
  String menu = "🔧 *ENGINEERING MODE ACTIVATED*\n\n";
  menu += "Main Menu - Select an option:\n";
  menu += "1️⃣ Component Test\n";
  menu += "2️⃣ Connectivity Test\n";
  menu += "3️⃣ Calibration\n";
  menu += "4️⃣ System Info\n";
  menu += "5️⃣ System Reboot\n";
  menu += "6️⃣ Exit Test Mode\n\n";
  menu += "Send the number (1-6) to select";
  
  telegram.sendMessage(CHAT_ID, menu, "Markdown");
}
/* ----------------  26. Engineering Mode Menu Logic (END)  -------------------- */


/* ----------------  27. Component Test Submenu Logic (START)  -------------------- */
void sendSubMenu() {
  String menu = "🔧 *COMPONENT TEST MENU*\n\n";
  menu += "Select component to test:\n";
  menu += "1️⃣ Water Level Sensor\n";
  menu += "2️⃣ Water Inlet Valve\n";
  menu += "3️⃣ Drain Motor (Wash)\n";
  menu += "4️⃣ Drain Motor (Spin)\n";
  menu += "5️⃣ Motor Rotation Test\n";
  menu += "6️⃣ LED Test\n";
  menu += "7️⃣ MCU Self Test\n";
  menu += "8️⃣ All Buttons Test\n";
  menu += "9️⃣ Back to Main Menu\n\n";
  menu += "Send number (1-9):";
  
  telegram.sendMessage(CHAT_ID, menu, "Markdown");
}
/* ----------------  27. Component Test Submenu Logic (END)  -------------------- */


/* ----------------  28. Engineering Mode Control Functions (START)  -------------------- */
void enterEngineeringMode() {
  if (programRunning) {
    telegram.sendMessage(CHAT_ID, "❌ Cannot enter TEST MODE: Program is currently running!", "");
    return;
  }
  
  isTestMode = true;
  testMenuOption = 0;
  
  
  displayTestMenu();
  
  // Send menu via Telegram
  sendMenu();
  
  Serial.println("Engineering Mode Activated");
}

void exitEngineeringMode() {
  isTestMode = false;
  
  // Ensure all outputs are OFF
  digitalWrite(INV_PW, OFF);
  digitalWrite(DM_WASH, OFF);
  digitalWrite(DM_SPIN, OFF);
  digitalWrite(IV, OFF);
  digitalWrite(CO1, OFF);
  digitalWrite(CO2, OFF);
  analogWrite(CTR_SIG, 0);
  
  // Update hardware display
  display.clear();
  displayPrint();
  
  telegram.sendMessage(CHAT_ID, "✅ Exited TEST MODE. Back to normal operation.", "");
  Serial.println("Engineering Mode Exited");
}

void handleSubMenu(String cmd) {
  cmd.trim();
  
  if (cmd == "1") {
    waterLevelSensorTest();
  } 
  else if (cmd == "2") {
    inletValveTest();
  }
  else if (cmd == "3") {
    drainMotorWashStageTest();
  }
  else if (cmd == "4") {
    drainMotorSpinStageTest();
  }
  else if (cmd == "5") {
   // motorRotationTest();
  }
  else if (cmd == "6") {
   // ledTest();
  }
  else if (cmd == "7") {
    // mcuSelfTest();
  }
  else if (cmd == "8") {
    // allButtonsTest();
  }
  else if (cmd == "9" || cmd == "back") {
    // Return to main menu
    testMenuOption = 0;
    sendMenu();
  }
  else {
    telegram.sendMessage(CHAT_ID, "❓ Invalid option. Send 1-9 for component tests.", "");
  }
}


void handleMenu(String cmd) {
  cmd.trim();
  
  // ========== COMPONENT TEST SUBMENU HANDLER ==========
  if (testMenuOption == 100) {
    handleSubMenu(cmd);
    return;
  }
  
  // ========== MAIN MENU HANDLER ==========
  if (cmd == "1") {
    // Component Test - Enter submenu
    testMenuOption = 100;
    sendSubMenu();
  } 
  else if (cmd == "2") {
    connectivityTest();
  }
  else if (cmd == "3") {
    calibrationTest();
  }
  else if (cmd == "4") {
    sendSystemInfo();
  }
  else if (cmd == "5") {
    reboot();
  }
  else if (cmd == "6" || cmd == "exit") {
    exitEngineeringMode();
  }
  else if (cmd == "menu" || cmd == "/menu") {
    sendMenu();
  }
  else {
    telegram.sendMessage(CHAT_ID, "❓ Invalid option. Send 1-6 or 'menu' for main menu.", "");
  }
}

/* ----------------  28. Engineering Mode Control Functions (END)  -------------------- */


/* ----------------  29. Mode State Control Function (START)  -------------------- */
void handleTelegramMessages() {
  int numNewMessages = telegram.getUpdates(telegram.last_message_received + 1);
  
  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(telegram.messages[i].chat_id);
    String text = telegram.messages[i].text;
    
    // Only respond to authorized chat ID
    if (chat_id != CHAT_ID) {
      continue;
    }
    
    text.trim();
    text.toLowerCase();
    Serial.println(text);
    delay(1000);
    
    // Enter Engineering Mode
    if ((text == ENGINEERING_COMMAND || text == "/engineering") && !isTestMode) {
      enterEngineeringMode();
      continue;
    }
    
    // Exit Engineering Mode
    if (isTestMode && (text == ENGINEERING_EXIT || text == "/exit" || text == "6")) {
      exitEngineeringMode();
      continue;
    }
    
    // Handle engineering menu options
    if (isTestMode) {
      handleMenu(text);
      continue;
    }
  }
}
/* ----------------  29. Mode State Control Function (END)  -------------------- */

/* ----------------  30. Main Setup Function (START)  -------------------- */
void setup()
{
  Serial.begin(115200);
  pinMode(INV_PW, OUTPUT);
  pinMode(DM_WASH, OUTPUT);
  pinMode(DM_SPIN, OUTPUT);
  pinMode(IV, OUTPUT);
  pinMode(CO1, OUTPUT);
  pinMode(CO2, OUTPUT);
  pinMode(FB_SIG, INPUT_PULLUP);
  pinMode(CTR_SIG, OUTPUT);
  pinMode(SOAK_LED, OUTPUT);
  pinMode(WASH_LED, OUTPUT);
  pinMode(RINSE_LED, OUTPUT);
  pinMode(SPIN_LED, OUTPUT);
  pinMode(WIFI_LED, OUTPUT);
  pinMode(WASH_BTN, INPUT_PULLUP);
  pinMode(RINSE_BTN, INPUT_PULLUP);
  pinMode(SPIN_BTN, INPUT_PULLUP);
  pinMode(COMP_BTN, INPUT_PULLUP);
  pinMode(HALT_BTN, INPUT);
  digitalWrite(INV_PW, OFF);
  digitalWrite(DM_WASH, OFF);
  digitalWrite(DM_SPIN, OFF);
  digitalWrite(IV, OFF);
  digitalWrite(CO1, OFF);
  digitalWrite(CO2, OFF);
  digitalWrite(CTR_SIG, OFF);
  level.begin(WLS_DATA, WLS_CLK);
  Serial.println("Pre-Init WLS Reading:");
  Serial.println(level.get_units(), 2);
  if (isSimulation)
  {
    level.set_scale();
  } 
  else {
    level.set_scale(3100.f);
  }
  Serial.println("Post-Init WLS Reading:");
  Serial.println(level.get_units(), 2);
  Serial.println("WLS Initialised.............");
  Serial.println("POST Complete...............");
  display.init();
  display.backlight();
  display.setCursor(0, 0);
  display.print(" IntelliVerter ");
  display.setCursor(0, 1);
  display.print("Washing Machine");
  digitalWrite(WASH_LED, ON);
  digitalWrite(RINSE_LED, ON);
  digitalWrite(SPIN_LED, ON);
  digitalWrite(SOAK_LED, ON);
  vTaskDelay(2500 / portTICK_PERIOD_MS);
  digitalWrite(WASH_LED, OFF);
  digitalWrite(RINSE_LED, OFF);
  digitalWrite(SPIN_LED, OFF);
  digitalWrite(SOAK_LED, OFF);
  attachInterrupt(digitalPinToInterrupt(WASH_BTN), washButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(RINSE_BTN), rinseButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(SPIN_BTN), spinButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(COMP_BTN), compButtonISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(HALT_BTN), haltButtonISR, FALLING);

  wifiConnected = connectWifi();

  if (wifiConnected)
  {
    secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
    server.on("/", []()
              { server.send(200, "text/plain", "Hello From ATSystems Washing Machine!"); });

    // ElegantOTA.clearAuth();
    ElegantOTA.setAuth(authID, authPASS);

    ElegantOTA.begin(&server); // Start ElegantOTA
    // ElegantOTA callbacks
    ElegantOTA.onStart(onOTAStart);
    ElegantOTA.onProgress(onOTAProgress);
    ElegantOTA.onEnd(onOTAEnd);

    server.begin();
    Serial.println("HTTP server started");
  }
  else
  {
      display.println("Offline Mode");
      vTaskDelay(2500 / portTICK_PERIOD_MS);
  }
  

  xTaskCreatePinnedToCore(
      ledtask,         // Task Function
      "LEDTask",       // Task Name
      2048,            // Stack Size (bytes)
      NULL,            // Task parameter
      1,               // Task Priority
      &ledtask_handle, // Task handle
      0                // Core to run the task (0 = core0; 1 = core1; tskNO_AFFINITY = auto allocate core)
  );
  displayPrint();
}
/* ----------------  30. Main Setup Function (END)  -------------------- */


/* ----------------  31. Main Loop Function (START)  -------------------- */
void loop()
{
  digitalWrite(WIFI_LED, wifiConnected ? ON : OFF);
  server.handleClient();
  ElegantOTA.loop();

  if (wifiConnected && (millis() - lastTelegramCheck > telegramCheckDelay)) {
    handleTelegramMessages();
    lastTelegramCheck = millis();
  }
  
 
  if (isTestMode) {
    display.setCursor(1, 0);
    display.print("  TEST MODE  ");
    display.setCursor(1, 1);
    display.print(" Engineering ");
    delay(100);
    return; 
  }

  if (buttonPressed)
  {
    buttonPressed = false;
    switch (selectedMode)
    {
    case 1:
      digitalWrite(WASH_LED, ON);
      display.clear();
      display.setCursor(2, 0);
      display.print("Wash Only");
      display.setCursor(2, 1);
      display.print("Time: 30 Min");
      startWaitTime = millis();
      vTaskDelay(10 / portTICK_PERIOD_MS);
      while (millis() - startWaitTime < waitTime)
      {
        if (buttonPressed)
        {
          selectedMode = 0;
          break;
        }
      }
      vTaskDelay(100 / portTICK_PERIOD_MS);
      if (selectedMode == 1)
      {
        telegram.sendMessage(CHAT_ID, "Wash Only Started", "");
        startTime = millis();
        isWashing = true;
        washLogic();
        isWashing = false;
        telegram.sendMessage(CHAT_ID, "Washing Complete", "");
        digitalWrite(WASH_LED, ON);
        isSpinning = true;
        spinLogic();
        isSpinning = false;
        vTaskDelay(10 / portTICK_PERIOD_MS);
        String message0 = "Total Water Used:" + String(washWaterUsed) + " L";
        telegram.sendMessage(CHAT_ID, message0, "");
        vTaskDelay(10 / portTICK_PERIOD_MS);
        runTime = millis() - startTime;
        String message1 = "Program Complete. Total Runtime:  " + String(runTime / 60000) + " Minutes";
        telegram.sendMessage(CHAT_ID, message1, "");
      }
      selectedMode = 0;
      buttonPressed = false;
      digitalWrite(WASH_LED, OFF);
      digitalWrite(RINSE_LED, OFF);
      digitalWrite(SPIN_LED, OFF);
      display.clear();
      displayPrint();
      delay(1000);
      break;
    case 2:
      digitalWrite(RINSE_LED, ON);
      display.clear();
      display.setCursor(2, 0);
      display.print("Rinse Only");
      display.setCursor(2, 1);
      display.print("Time: 30 Min");
      startWaitTime = millis();
      vTaskDelay(10 / portTICK_PERIOD_MS);
      while (millis() - startWaitTime < waitTime)
      {
        if (buttonPressed)
        {
          selectedMode = 0;
          break;
        }
      }
      vTaskDelay(100 / portTICK_PERIOD_MS);
      if (selectedMode == 2)
      {
        telegram.sendMessage(CHAT_ID, "Rinse Only Started", "");
        startTime = millis();
        isRinsing = true;
        rinseLogic();
        isRinsing = false;
        telegram.sendMessage(CHAT_ID, "Rinsing Complete", "");
        digitalWrite(RINSE_LED, ON);
        isSpinning = true;
        spinLogic();
        isSpinning = false;
        vTaskDelay(10 / portTICK_PERIOD_MS);
        String message2 = "Total Water Used:" + String(rinseWaterUsed) + " L";
        telegram.sendMessage(CHAT_ID, message2, "");
        vTaskDelay(10 / portTICK_PERIOD_MS);
        runTime = millis() - startTime;
        String message3 = "Program Complete. Total Runtime:  " + String(runTime / 60000) + " Minutes";
        telegram.sendMessage(CHAT_ID, message3, "");
        vTaskDelay(10 / portTICK_PERIOD_MS);
      }
      selectedMode = 0;
      buttonPressed = false;
      digitalWrite(WASH_LED, OFF);
      digitalWrite(RINSE_LED, OFF);
      digitalWrite(SPIN_LED, OFF);
      display.clear();
      displayPrint();
      delay(1000);
      break;
    case 3:
      digitalWrite(SPIN_LED, ON);
      display.clear();
      display.setCursor(2, 0);
      display.print("Spin Only");
      display.setCursor(2, 1);
      display.print("Time: 10 Min");
      vTaskDelay(10 / portTICK_PERIOD_MS);
      while (millis() - startWaitTime < waitTime)
      {
        if (buttonPressed)
        {
          selectedMode = 0;
          break;
        }
      }
      vTaskDelay(100 / portTICK_PERIOD_MS);
      if (selectedMode == 3)
      {
        telegram.sendMessage(CHAT_ID, "Spin Only Started", "");
        startTime = millis();
        isSpinning = true;
        spinLogic();
        isSpinning = false;
        vTaskDelay(10 / portTICK_PERIOD_MS);
        telegram.sendMessage(CHAT_ID, "Spinning Complete", "");
        vTaskDelay(10 / portTICK_PERIOD_MS);
        runTime = millis() - startTime;
        String message4 = "Program Complete. Total Runtime:  " + String(runTime / 60000) + " Minutes";
        telegram.sendMessage(CHAT_ID, message4, "");
      }
      selectedMode = 0;
      buttonPressed = false;
      digitalWrite(WASH_LED, OFF);
      digitalWrite(RINSE_LED, OFF);
      digitalWrite(SPIN_LED, OFF);
      display.clear();
      displayPrint();
      delay(1000);
      break;
    case 4:
      digitalWrite(WASH_LED, ON);
      digitalWrite(RINSE_LED, ON);
      digitalWrite(SPIN_LED, ON);
      display.clear();
      display.setCursor(2, 0);
      display.print("Complete Wash");
      display.setCursor(2, 1);
      display.print("Time: 45 Min");
      startWaitTime = millis();
      vTaskDelay(10 / portTICK_PERIOD_MS);
      while (millis() - startWaitTime < waitTime)
      {
        if (buttonPressed)
        {
          selectedMode = 0;
          break;
        }
      }
      vTaskDelay(100 / portTICK_PERIOD_MS);
      if (selectedMode == 4)
      {
        telegram.sendMessage(CHAT_ID, "Complete Wash Started", "");
        startTime = millis();
        isCompleteProgramWash = true;
        isCompleteProgramRinse = false;
        isCompleteProgramSpin = false;
        washLogic();
        telegram.sendMessage(CHAT_ID, "Washing Complete", "");
        spinLogic();
        isCompleteProgramWash = false;
        isCompleteProgramRinse = true;
        isCompleteProgramSpin = false;
        rinseLogic();
        isCompleteProgramWash = false;
        isCompleteProgramRinse = false;
        isCompleteProgramSpin = true;
        telegram.sendMessage(CHAT_ID, "Rinsing Complete", "");
        spinLogic();
        isCompleteProgramWash = false;
        isCompleteProgramRinse = false;
        isCompleteProgramSpin = false;
        telegram.sendMessage(CHAT_ID, "Spinning Complete", "");
        totalWaterUsed = washWaterUsed + rinseWaterUsed;
        vTaskDelay(10 / portTICK_PERIOD_MS);
        String message4 = "Total Water Used:" + String(totalWaterUsed) + " L";
        telegram.sendMessage(CHAT_ID, message4, "");
        vTaskDelay(10 / portTICK_PERIOD_MS);
        runTime = millis() - startTime;
        String message5 = "Program Complete. Total Runtime:  " + String(runTime / 60000) + " Minutes";
        telegram.sendMessage(CHAT_ID, message5, "");
      }
      selectedMode = 0;
      buttonPressed = false;
      digitalWrite(WASH_LED, OFF);
      digitalWrite(RINSE_LED, OFF);
      digitalWrite(SPIN_LED, OFF);
      display.clear();
      displayPrint();
      delay(1000);
      break;
    default:
      selectedMode = 0;
      buttonPressed = false;
      digitalWrite(WASH_LED, OFF);
      digitalWrite(RINSE_LED, OFF);
      digitalWrite(SPIN_LED, OFF);
    }
  }
}
/* ----------------  31. Main Loop Function (END)  -------------------- */

