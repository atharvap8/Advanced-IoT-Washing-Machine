
/* To-Do
• LCD Integration {Check}
• Wash, Rinse, Spin Button {Check}
• Wash, Rinse, Spin, Soak LEDs {Check}
• Add LED Power Button
• Add Emergency Inverter OFF Button
• Wifi Status LED {Check}
• Display Print Statements
• Control Logic
• Standalone Program LED Lightup
• CountDown Timer Integration {}


*/
#include <Wire.h>
#include <WiFi.h>
#include <HX711.h>
#include <Arduino.h>
#include <Espalexa.h> 
#include <WebServer.h>
#include <WiFiClient.h>
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
#define DISPLAY_COLS 20
#define DISPLAY_ROWS 4
#define OFF LOW
#define ON HIGH
#define BOT_TOKEN "6832887486:AAFgPAgZFV2qECq_RNY6XItXJEyK8nN4-9A"
#define CHAT_ID "1403405625"

HX711 level;
float waterLevel;
float tareWaterLevel;
Espalexa espalexa;
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
const int setFillingWaterLevel = 18;
const int setDrainingWaterLevel = 2;
const int waitTime = 10000;
unsigned long runTime = 0;
unsigned long startTime = 0;
unsigned long startWaitTime = 0;
unsigned long ota_progress_millis = 0;
unsigned long lastButtonPressTime = 0;


void firstLightChanged(uint8_t brightness);
void secondLightChanged(uint8_t brightness);
void thirdLightChanged(uint8_t brightness);
void fourthLightChanged(uint8_t brightness);
void fifthLightChanged(uint8_t brightness);

String Device1 = "Wash";
String Device2 = "Rinse";
String Device3 = "Spin";
String Device4 = "Complete Wash";
String Device5 = "Soak";

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

void IRAM_ATTR rinseButtonISR()
{
  if (!programRunning)
  {
    selectedMode = 2;
    lastButtonPressTime = millis();
    buttonPressed = true;
  }
}

void IRAM_ATTR spinButtonISR()
{
  if (!programRunning)
  {
    selectedMode = 3;
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
  for (;;)
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
    }
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
  digitalWrite(DM_WASH, ON);
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  digitalWrite(INV_PW, ON);
  display.clear();
  display.setCursor(1, 0);
  display.print("Washing... P1");

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
  display.print("Washing... P2");
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

// Alexa Stuff

void firstLightChanged(uint8_t brightness)
{

  if (brightness == 255)
  {
    Serial.println("Starting Wash Program.....");
    isSoaking = false;
    isWashing = true;
    isRinsing = false;
    isSpinning = false;
    washLogic();
    isSoaking = false;
    isWashing = false;
    isRinsing = false;
    isSpinning = false;
    spinLogic();
  }
}

void secondLightChanged(uint8_t brightness)
{

  if (brightness == 255)
  {
    Serial.println("Starting Rinse Program.....");
    startTime = millis();
    isSoaking = false;
    isWashing = false;
    isRinsing = true;
    isSpinning = false;
    rinseLogic();
    isSoaking = false;
    isWashing = false;
    isRinsing = false;
    isSpinning = true;
    spinLogic();
    isSoaking = false;
    isWashing = false;
    isRinsing = false;
    isSpinning = false;
  }
}

void thirdLightChanged(uint8_t brightness)
{

  if (brightness == 255)
  {
    Serial.println("Starting Spin Program.....");
    startTime = millis();
    isSoaking = false;
    isWashing = false;
    isRinsing = false;
    isSpinning = true;
    spinLogic();
    isSoaking = false;
    isWashing = false;
    isRinsing = false;
    isSpinning = false;
  }
}

void fourthLightChanged(uint8_t brightness)
{

  if (brightness == 255)
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
        delay(10);
        String message1 = "Total Water Used:" + String(totalWaterUsed) + " L";
        telegram.sendMessage(CHAT_ID, message1, "");
        delay(10);
        runTime = millis() - startTime;
        String message2 = "Program Complete. Total Runtime:  " + String(runTime / 60000) + " Minutes";
        telegram.sendMessage(CHAT_ID, message2, "");
  }
}

void fifthLightChanged(uint8_t brightness)
{
  if (brightness == 255)
  {
    Serial.println("Starting Soak.....");
    isSoaking = true;
    isWashing = false;
    isRinsing = false;
    isSpinning = false;
    soakLogic();
    isSoaking = false;
    isWashing = false;
    isRinsing = false;
    isSpinning = false;
  }
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

    // Define your devices here.
    espalexa.addDevice(Device1, firstLightChanged); // simplest definition, default state off
    espalexa.addDevice(Device2, secondLightChanged);
    espalexa.addDevice(Device3, thirdLightChanged);
    espalexa.addDevice(Device4, fourthLightChanged);
    espalexa.addDevice(Device5, fifthLightChanged);
    espalexa.begin();
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
  espalexa.loop();
  ElegantOTA.loop();
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

