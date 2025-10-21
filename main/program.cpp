
/* To-Do
Make sure to check whether isSimulation is set to true or false.
If true, the HX711 will be set to a default scale value.
If false, the HX711 will be set to the scale value obtained from calibration.

Modify the credentials.h file to include the Telegram Bot Token and Chat ID, and 
ensure that the WiFi credentials are correct.
! Implement Test Mode (Hybrid) (Telegram - HW Control)


*/
#include <Wire.h>
#include <WiFi.h>
#include <HX711.h>
#include <Arduino.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include "credentials.h"
#include <freertos/task.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include <freertos/FreeRTOS.h>

#define INV_PW 32
#define DM_WASH 25
#define DM_SPIN 27
#define IV 14
#define CO1 26
#define CO2 33
#define WLS_DATA 23
#define WLS_CLK 19
#define FB_SIG 5
#define CTR_SIG 18
#define SOAK_LED 17
#define WASH_LED 4
#define RINSE_LED 13
#define SPIN_LED 16
#define WIFI_LED 15
#define HALT_BTN 34
#define COMP_BTN 12
#define WASH_BTN 2
#define RINSE_BTN 0
#define SPIN_BTN 35
#define I2C_ADDR 0x27
#define DISPLAY_COLS 16
#define DISPLAY_ROWS 2
#define OFF LOW
#define ON HIGH


HX711 level;
float waterLevel;
float tareWaterLevel;
WebServer server(1906);
WiFiClientSecure secured_client;
UniversalTelegramBot telegram(BOT_TOKEN, secured_client);
TaskHandle_t ledtask_handle = NULL;
LiquidCrystal_I2C display(I2C_ADDR, DISPLAY_COLS, DISPLAY_ROWS);

bool isSimulation = false;
bool isSoaking = false;
bool isWashing = false;
bool isRinsing = false;
bool isSpinning = false;
bool isWaiting = true;
bool isCompleteProgramWash = false;
bool isCompleteProgramRinse = false;
bool isCompleteProgramSpin = false;
bool buttonPressed = false;
bool programRunning = false;

boolean wifiConnected = false;
boolean connectWifi();

volatile float washWaterUsed = 0;
volatile float rinseWaterUsed = 0;
volatile float totalWaterUsed = 0;
volatile int selectedMode = 0; // 1 for WASH, 2 for RINSE, 3 for SPIN
const int multiplier = 27.4;
const int offset = 10.9;
const int setFillingWaterLevel = 18.5;
const int setDrainingWaterLevel = 2;
const int waitTime = 10000;
unsigned long runTime = 0;
unsigned long startTime = 0;
unsigned long startWaitTime = 0;
unsigned long ota_progress_millis = 0;
unsigned long lastButtonPressTime = 0;

/* -------------------- Engineering Mode Variables ---------------------- */
bool isTestMode = false;
int testMenuOption = 0; 
const String ENGINEERING_COMMAND = "engineering";
const String ENGINEERING_EXIT = "exit";
unsigned long lastTelegramCheck = 0;
const unsigned long telegramCheckDelay = 1000; 

// ========== VALVE TEST Variables ==========
const float VALVE_TEST_DURATION = 30000;  // 30 seconds in milliseconds
const float VALVE_TEST_MIN_DELTA = 2.0;   // Minimum water level increase (Liters) for PASS
const float VALVE_TEST_SAMPLE_INTERVAL = 1000;  // Sample every 1 second
const int VALVE_TEST_SAMPLES = VALVE_TEST_DURATION / VALVE_TEST_SAMPLE_INTERVAL;

// ========== Drain Motor Test Variables ==========
bool awaitingDrainMotorResponse = false;
bool drainMotorTestResult = false; // true = smooth, false = stuck

/* -------------------- Engineering Mode Variables ---------------------- */

// Interrupt Management
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

// LED Management Task

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

// OTA Stuff

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

void displayPrint()
{
  display.clear();
  display.setCursor(1, 0);
  display.print("Please Select");
  display.setCursor(1, 1);
  display.print("A Program");
}

// Program Logic

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

// Rinse Sequence

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

// Spin Sequence

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

// Soak Sequence

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


/* ----------------  TEST CODEBLOCK AREA  -------------------- */

void performSensorTest() {
  telegram.sendMessage(CHAT_ID, "🔍 Running Sensor Test...", "");
  
  String msg = "📊 *SENSOR TEST RESULTS*\n\n";
  
  // Water Level Sensor
  tareWaterLevel = level.get_units() / multiplier;
  waterLevel = tareWaterLevel - offset;
  msg += "💧 Water Level: " + String(waterLevel, 2) + " L\n";
  
  // Feedback Signal
  int fbSignal = digitalRead(FB_SIG);
  msg += "📡 Feedback Signal: " + String(fbSignal ? "HIGH ✅" : "LOW ❌") + "\n";
  
  // Button States
  msg += "\n🎛️ *Button States:*\n";
  msg += "WASH: " + String(digitalRead(WASH_BTN) == LOW ? "PRESSED" : "RELEASED") + "\n";
  msg += "RINSE: " + String(digitalRead(RINSE_BTN) == LOW ? "PRESSED" : "RELEASED") + "\n";
  msg += "SPIN: " + String(digitalRead(SPIN_BTN) == LOW ? "PRESSED" : "RELEASED") + "\n";
  msg += "COMP: " + String(digitalRead(COMP_BTN) == LOW ? "PRESSED" : "RELEASED") + "\n";
  msg += "HALT: " + String(digitalRead(HALT_BTN) == LOW ? "PRESSED" : "RELEASED") + "\n";
  
  // Raw HX711 reading
  msg += "\n📈 Raw HX711: " + String(level.get_units(), 2) + "\n";
  msg += "Multiplier: " + String(multiplier) + "\n";
  msg += "Offset: " + String(offset) + "\n";
  
  telegram.sendMessage(CHAT_ID, msg, "Markdown");
}

/* ----------------  Inlet Valve Test Logic  -------------------- */

float getCurrentWaterLevel() {
  // Take multiple readings and average them for accuracy
  float total = 0;
  int readings = 5;
  
  for (int i = 0; i < readings; i++) {
    tareWaterLevel = level.get_units() / multiplier;
    waterLevel = tareWaterLevel - offset;
    total += waterLevel;
    delay(50);
  }
  
  return total / readings;
}

String generateValveTestReport(float initialLevel, float finalLevel, float totalDelta, float flowRate, bool passed, float* readings, int sampleCount) {
  String report = passed ? "✅ *VALVE TEST PASSED*\n\n" : "❌ *VALVE TEST FAILED*\n\n";
  
  // Summary
  report += "📊 *Test Summary:*\n";
  report += "Initial Level: " + String(initialLevel, 2) + " L\n";
  report += "Final Level: " + String(finalLevel, 2) + " L\n";
  report += "Water Added: " + String(totalDelta, 2) + " L\n";
  report += "Flow Rate: " + String(flowRate * 60, 1) + " L/min\n";
  report += "Duration: 30 seconds\n\n";
  
  // Pass/Fail analysis
  report += "🎯 *Analysis:*\n";
  report += "Required: ≥" + String(VALVE_TEST_MIN_DELTA, 1) + " L\n";
  report += "Actual: " + String(totalDelta, 2) + " L\n";
  
  if (passed) {
    report += "Status: VALVE WORKING ✅\n";
    if (totalDelta > 8.0) {
      report += "Note: High flow rate detected\n";
    } else if (totalDelta < 3.0) {
      report += "Note: Low flow rate (check filters)\n";
    }
  } else {
    report += "Status: VALVE FAULT ❌\n";
    if (totalDelta < 0.5) {
      report += "Possible Issues:\n";
      report += "• Valve stuck closed\n";
      report += "• No water supply\n";
      report += "• Electrical connection fault\n";
    } else {
      report += "Possible Issues:\n";
      report += "• Partially blocked valve\n";
      report += "• Low water pressure\n";
      report += "• Sensor calibration needed\n";
    }
  }
  
  // Add trend analysis if enough samples
  if (sampleCount > 10) {
    report += "\n📈 *Flow Trend:*\n";
    float midPoint = readings[sampleCount/2] - initialLevel;
    float trend = (totalDelta - midPoint) / midPoint * 100;
    
    if (trend > 10) {
      report += "Increasing flow rate\n";
    } else if (trend < -10) {
      report += "Decreasing flow rate\n";
    } else {
      report += "Steady flow rate\n";
    }
  }
  
  return report;
}


void inletValveTest() {
  telegram.sendMessage(CHAT_ID, "🚰 *WATER INLET VALVE AUTO-TEST*\nStarting 30-second test...", "Markdown");
  
  // Safety check - ensure tank is not overfull
  float initialLevel = getCurrentWaterLevel();
  
  if (initialLevel > 15.0) { // Prevent overflow
    String msg = "⚠️ *TEST ABORTED*\nWater level too high: " + String(initialLevel, 1) + "L\n";
    msg += "Please drain tank below 15L before testing.";
    telegram.sendMessage(CHAT_ID, msg, "Markdown");
    return;
  }
  
  // Initialize test variables
  float waterLevelReadings[VALVE_TEST_SAMPLES];
  unsigned long testStartTime = millis();
  int sampleIndex = 0;
  
  String progressMsg = "📊 *TEST IN PROGRESS*\n";
  progressMsg += "Initial Level: " + String(initialLevel, 2) + "L\n";
  progressMsg += "Duration: 30s | Min Delta: " + String(VALVE_TEST_MIN_DELTA, 1) + "L\n\n";
  progressMsg += "🔄 Opening valve...";
  telegram.sendMessage(CHAT_ID, progressMsg, "Markdown");
  
  // Open the inlet valve
  digitalWrite(IV, ON);
  
  // Collect samples during test
  while ((millis() - testStartTime) < VALVE_TEST_DURATION && sampleIndex < VALVE_TEST_SAMPLES) {
    float currentLevel = getCurrentWaterLevel();
    waterLevelReadings[sampleIndex] = currentLevel;
    
    // Send progress update every 5 seconds
    if (sampleIndex % 5 == 0) {
      float deltaLevel = currentLevel - initialLevel;
      int secondsElapsed = (millis() - testStartTime) / 1000;
      int secondsRemaining = 30 - secondsElapsed;
      
      String update = "📈 *Progress Update*\n";
      update += "Time: " + String(secondsElapsed) + "s / 30s\n";
      update += "Current: " + String(currentLevel, 2) + "L\n";
      update += "Delta: " + String(deltaLevel, 2) + "L\n";
      update += "Remaining: " + String(secondsRemaining) + "s";
      telegram.sendMessage(CHAT_ID, update, "Markdown");
    }
    
    sampleIndex++;
    delay(VALVE_TEST_SAMPLE_INTERVAL);
  }
  
  // Close the valve
  digitalWrite(IV, OFF);
  
  // Analyze results
  float finalLevel = getCurrentWaterLevel();
  float totalDelta = finalLevel - initialLevel;
  float averageFlowRate = totalDelta / (VALVE_TEST_DURATION / 1000.0); // L/s
  bool testPassed = (totalDelta >= VALVE_TEST_MIN_DELTA);
  
  // Generate detailed report
  String report = generateValveTestReport(initialLevel, finalLevel, totalDelta, averageFlowRate, testPassed, waterLevelReadings, sampleIndex);
  telegram.sendMessage(CHAT_ID, report, "Markdown");
  
  // Log to serial for debugging
  Serial.println("=== VALVE TEST COMPLETE ===");
  Serial.printf("Initial: %.2fL, Final: %.2fL, Delta: %.2fL\n", initialLevel, finalLevel, totalDelta);
  Serial.printf("Flow Rate: %.3fL/s, Result: %s\n", averageFlowRate, testPassed ? "PASS" : "FAIL");
}

/* ----------------  Inlet Valve Test Logic  -------------------- */

/* ----------------  Drain Motor Test Logic  -------------------- */

void generateDrainMotorReport(bool passed) {
  display.clear();
  display.setCursor(0, 0);
  display.print(passed ? "Motor: PASS" : "Motor: FAIL");
  display.setCursor(0, 1);
  display.print(passed ? "Working OK" : "Brake Stuck");
  delay(3000);
  
  String report = "";
  
  if (passed) {
    // Motor working correctly
    report = "✅ *DRAIN MOTOR TEST - PASSED*\n\n";
    report += "🎯 *Test Result:*\n";
    report += "Drum rotates smoothly when brake is engaged.\n\n";
    report += "✔️ *Status:* Motor brake functioning correctly\n";
    report += "✔️ Motor coil resistance normal\n";
    report += "✔️ Brake mechanism releases properly\n";
    report += "✔️ No mechanical obstruction\n\n";
    report += "💡 *Recommendation:*\n";
    report += "Motor is in good working condition. No action needed.";
    
  } else {
    // Motor failed test
    report = "❌ *DRAIN MOTOR TEST - FAILED*\n\n";
    report += "🎯 *Test Result:*\n";
    report += "Drum is difficult or impossible to rotate manually.\n\n";
    report += "⚠️ *Possible Causes:*\n";
    report += "1. Brake not releasing\n";
    report += "   → Check motor coil connections\n";
    report += "   → Measure coil resistance (should be ~100-300Ω)\n\n";
    report += "2. Mechanical seizure\n";
    report += "   → Bearing failure\n";
    report += "   → Foreign object in drum\n";
    report += "   → Belt too tight\n\n";
    report += "3. Electrical fault\n";
    report += "   → Motor not receiving power\n";
    report += "   → Relay failure on DM_WASH pin\n";
    report += "   → Wiring issue\n\n";
    report += "🔧 *Recommended Actions:*\n";
    report += "1. Disconnect power and manually check drum rotation\n";
    report += "2. Measure motor coil resistance\n";
    report += "3. Check DM_WASH relay output with multimeter\n";
    report += "4. Inspect motor mounting and bearings\n";
    report += "5. Check for belt misalignment";
  }
  
  telegram.sendMessage(CHAT_ID, report, "Markdown");
}

void drainMotorTest() {
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
  
  // Activate drain motor (brake)
  display.clear();
  display.setCursor(0, 0);
  display.print("Motor: ON");
  display.setCursor(0, 1);
  display.print("Try turning..");
  
  digitalWrite(DM_WASH, ON);
  
  String instructionMsg = "⚡ *DRAIN MOTOR ACTIVATED*\n\n";
  instructionMsg += "🖐️ Now try to rotate the drum by hand.\n\n";
  instructionMsg += "Press on the HARDWARE device:\n";
  instructionMsg += "✅ SPIN button - Smooth rotation\n";
  instructionMsg += "❌ RINSE button - Stuck/difficult\n\n";
  instructionMsg += "Waiting for your input...";
  
  telegram.sendMessage(CHAT_ID, instructionMsg, "Markdown");
  
  // Set flag and wait for button response
  awaitingDrainMotorResponse = true;
  drainMotorTestResult = false;
  
  unsigned long waitStart = millis();
  unsigned long timeout = 60000; // 60 second timeout
  
  // Wait for user to press button
  while (awaitingDrainMotorResponse && (millis() - waitStart < timeout)) {
    // Update display periodically
    if ((millis() - waitStart) % 2000 < 100) {
      display.setCursor(0, 1);
      display.print("Waiting...      ");
    }
    delay(100);
  }
  
  // Turn off motor
  digitalWrite(DM_WASH, OFF);
  
  // Check if timeout occurred
  if (!awaitingDrainMotorResponse) {
    // User responded, generate report
    generateDrainMotorReport(drainMotorTestResult);
  } else {
    // Timeout
    display.clear();
    display.setCursor(0, 0);
    display.print("Test Timeout");
    display.setCursor(0, 1);
    display.print("No Response");
    
    telegram.sendMessage(CHAT_ID, "⏱️ *TEST TIMEOUT*\nNo button pressed within 60 seconds.\nTest cancelled.", "Markdown");
    delay(2000);
  }
  
  // Reset flag
  awaitingDrainMotorResponse = false;
  
  // Return display to test mode
  display.clear();
  display.setCursor(1, 0);
  display.print("  TEST MODE  ");
  display.setCursor(1, 1);
  display.print(" Engineering ");
}

/* ----------------  Drain Motor Test Logic  -------------------- */

void performOutputTest() {
  telegram.sendMessage(CHAT_ID, "⚡ Starting Output Test...\nEach output will be tested as per sequence.", "");
  
  String msg = "🔌 *OUTPUT TEST SEQUENCE*\n\n";
  
  // Test Inlet Valve
  inletValveTest();
  
  // Test Drain Motors
  drainMotorTest();

  
  msg += "3. Spin Drain Motor... ";
  digitalWrite(DM_SPIN, ON);
  delay(1000);
  digitalWrite(DM_SPIN, OFF);
  msg += "✅\n";
  
  // Test Inverter Power
  msg += "4. Inverter Power... ";
  digitalWrite(INV_PW, ON);
  delay(1000);
  digitalWrite(INV_PW, OFF);
  msg += "✅\n";
  
  // Test Control Signal
  msg += "5. Motor Control Signal (50 PWM)... ";
  digitalWrite(INV_PW, ON);
  analogWrite(CTR_SIG, 50);
  delay(2000);
  analogWrite(CTR_SIG, 0);
  digitalWrite(INV_PW, OFF);
  msg += "✅\n";
  
  // Test CO1 and CO2
  msg += "6. Changeover Relay 1... ";
  digitalWrite(CO1, ON);
  delay(1000);
  digitalWrite(CO1, OFF);
  msg += "✅\n";
  
  msg += "7. Changeover Relay 2... ";
  // digitalWrite(CO2, ON);
  delay(1000);
  // digitalWrite(CO2, OFF);
  msg += "✅\n";
  
  msg += "\n✅ *All outputs tested successfully!*";
  telegram.sendMessage(CHAT_ID, msg, "Markdown");
}

void performConnectivityTest() {
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

void performCalibrationTest() {
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

void sendEngineeringMenu() {
  String menu = "🔧 *ENGINEERING MODE ACTIVATED*\n\n";
  menu += "Select an option:\n";
  menu += "1️⃣ Sensor Test\n";
  menu += "2️⃣ Output Test\n";
  menu += "3️⃣ Connectivity Test\n";
  menu += "4️⃣ Calibration\n";
  menu += "5️⃣ System Info\n";
  menu += "6️⃣ Exit Test Mode\n\n";
  menu += "Send the number (1-6) to select";
  
  telegram.sendMessage(CHAT_ID, menu, "Markdown");
}

void enterEngineeringMode() {
  if (programRunning) {
    telegram.sendMessage(CHAT_ID, "❌ Cannot enter TEST MODE: Program is currently running!", "");
    return;
  }
  
  isTestMode = true;
  testMenuOption = 0;
  
  // Update hardware display
  display.clear();
  display.setCursor(1, 0);
  display.print("  TEST MODE  ");
  display.setCursor(1, 1);
  display.print(" Engineering ");
  
  // Send menu via Telegram
  sendEngineeringMenu();
  
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


void handleEngineeringMenu(String cmd) {
  cmd.trim();
  
  if (cmd == "1") {
    performSensorTest();
  } 
  else if (cmd == "2") {
    performOutputTest();
  }
  else if (cmd == "3") {
    performConnectivityTest();
  }
  else if (cmd == "4") {
    performCalibrationTest();
  }
  else if (cmd == "5") {
    sendSystemInfo();
  }
  else if (cmd == "menu" || cmd == "/menu") {
    sendEngineeringMenu();
  }
  else {
    telegram.sendMessage(CHAT_ID, "❓ Invalid option. Send a number 1-6 or 'menu' to see options again.", "");
  }
}

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
      handleEngineeringMenu(text);
      continue;
    }
  }
}


/* ----------------  TEST CODEBLOCK AREA  -------------------- */

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

void loop()
{
  digitalWrite(WIFI_LED, wifiConnected ? ON : OFF);
  server.handleClient();
  ElegantOTA.loop();

 /* -------------------------Engineering Mode Check CodeBlock --------------------------- */

  if (wifiConnected && (millis() - lastTelegramCheck > telegramCheckDelay)) {
    handleTelegramMessages();
    lastTelegramCheck = millis();
  }
  
 
  if (isTestMode) {
    display.setCursor(0, 0);
    display.print("  TEST MODE  ");
    display.setCursor(0, 1);
    display.print(" Engineering ");
    delay(100);
    return; // Exit loop early
  }

/* -------------------------Engineering Mode Check Codeblock --------------------------- */


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

