#include <Arduino.h>
#include "MPU9250.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include "SixDOF.h"
#include "PHT.h"
#include <Adafruit_MS8607.h>
#include <math.h>
#include "GPS.h"
#include <map>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include <task.h>
#include <queue.h>
#include "LaunchState.h"

// Task handles
GPS GPS_NAV;
TaskHandle_t transmitTaskHandle;
TaskHandle_t receiveTaskHandle;
String canMessage;
int counter = 0;
volatile bool pinStatusUpdated[64] = {};
volatile int pinStatus[64][5] = {};
StaticJsonDocument<512> sensorDataGlobal;
SemaphoreHandle_t mutex_d; //dataserialize
// CAN TWAI message to send
twai_message_t txMessage;
int canTXRXcount[2] = { 0, 0 };
// #include "LaunchState.h"
#define SEALEVELPRESSURE_HPA (1013.25)
#define LSM_CS 5
#define LSM_SCK 18
#define LSM_MISO 19
#define LSM_MOSI 23

// Function prototypes
void transmitTask(void *pvParameters);
void receiveTask(void *pvParameters);
void commandTask(void *pvParameters);
String messageToCAN(String code);
// Define CAN pins
#define CAN_TX 16
#define CAN_RX 17
LaunchState current_state1 = LaunchState::PreIgnition;
SixDOF _6DOF;
MPU9250 mpu;
TwoWire I2C1 = TwoWire(0); // Default I2C bus
PHT Alt(I2C1);
TwoWire I2C2 = TwoWire(1); // Secondary I2C bus
uint16_t measurement_delay_us = 65535; // Delay between measurements for testing
GPS GPS1;
// LaunchState Halya;
int groundLevelAltitudeTest = 0;
std::map<string, bool> statesMapTest;
double AltArrayTest[10];
double previousMedianTest = 0;
bool CHECK = false;
const int baudrate = 921600;
const int rows = 4;
const int cols = 50;
int sendCAN = 1;
//printing helper variables
int waitforADS = 0;
int printADS[4] = { 1, 1, 1, 1 };
int printCAN = 1;
String loopprint;
int cycledelay = 2;
int preignitionCount = 0;
// Define a queue to hold the JSON data
QueueHandle_t jsonQueue;
uint8_t integerPartToHex(double dataValue)
{
    int integerPart = static_cast<int>(dataValue);
    if (integerPart > 255)
    {
        integerPart = 255;
    }
    return 0x00 + static_cast<uint8_t>(integerPart);
}
uint8_t decimalPartToHex(double dataValue)
{
    int integerPart = static_cast<int>(dataValue);
    double decimalPart = dataValue - integerPart;
    int scaledDecimal = static_cast<int>(decimalPart * 100);
    // if (scaledDecimal > 255)
    // {
    //     scaledDecimal = 255;
    // }
    uint8_t hexValue = 0x00 + static_cast<uint8_t>(scaledDecimal);
    return hexValue;
}
  // activate_SOL("1", 1, 1); turns on board 1 sol 1
void command2pin(String solboardIDnum, char solIndex, char mode) {
    twai_message_t txMessage_command;
  
    // Convert solboardIDnum to integer, ensuring it is valid
    int ID = solboardIDnum.toInt();
    if (ID < 0 || ID > 255) { // Limit range to prevent errors
        Serial.println("Invalid solboard ID");
        return;
    }
    
    Serial.print("Solboard ID: ");
    Serial.println(ID);
    
    txMessage_command.identifier = ID * 0x10 + 0x0F;  // Solenoid Board WRITE COMMAND 0xIDF
    txMessage_command.flags = TWAI_MSG_FLAG_EXTD;     // Extended frame format
    txMessage_command.data_length_code = 8;          // 8-byte message
  
    // Initialize all solenoid states to 0
    memset(txMessage_command.data, 0, sizeof(txMessage_command.data));
  
    // Determine solenoid index
    int statusPin = solIndex - '0';  // Convert char ('0'-'3') to int (0-3)
    if (statusPin < 0 || statusPin > 3) {
        Serial.println("Invalid solenoid index");
        return;
    }
  
    // Set the requested solenoid state
    if (mode == '0') {
        pinStatus[ID][statusPin] = 0;
        txMessage_command.data[statusPin] = 0;
    } else if (mode == '1') {
        pinStatus[ID][statusPin] = 1;
        txMessage_command.data[statusPin] = 1;
    } else {
        Serial.println("Invalid mode");
        return;
    }
  
    // Transmit the CAN message
    if (twai_transmit(&txMessage_command, pdMS_TO_TICKS(1)) == ESP_OK) {
        Serial.println("Solenoid " + String(solIndex) + " actuated successfully");
    } else {
        Serial.println( "Solenoid " +String(solIndex)+ " actuation failed");
    }
  }
  
// RTOS Tasks, not working as of 3/8 12:31 AM - Matthew
void SixDOF_Task(void *pvParameters){
  for(;;){
  String accel = _6DOF.printSensorData();
  Serial.println(accel);
  vTaskDelay(pdMS_TO_TICKS(200));
  vTaskDelete(NULL);
  }
}
double returnMedianTest(double arr[], int number)
{
  double temp[number];
  memcpy(temp, arr, sizeof(temp));
  std::sort(temp, temp + number);

  if (number % 2 == 0)
  {
    return (temp[number / 2 - 2] + temp[number / 2 - 1] + temp[number / 2] + temp[number / 2 + 1]) / 4.0;
  }
  else
  {
    return (temp[number / 2 - 1] + temp[number / 2] + temp[number / 2 + 1] + temp[number / 2 + 2] + temp[number / 2 + 3]) / 5.0;
    ;
  }
}
void PHT_Task(void *pvParameters){
  double altitude = Alt.getAltitude();
  Serial.println(String(altitude));

}
void Test_Task(void *pvParameters){
  for(;;){

  }
}
void allSol(){
  delay(100);
  command2pin("01", '0', '1');
  delay(100);
  command2pin("01", '0', '0');
  command2pin("01", '1', '1');
  delay(100);
  command2pin("01", '1', '0');
  command2pin("01", '2', '1');
  delay(100);
  command2pin("01", '2', '0');
  command2pin("01", '3', '1');
  delay(100);
  command2pin("01", '3', '0');
}
double calculateRateOfChangeTest(double AltArray[], int READINGS_LENGTH)
{
  double currentMedian = returnMedianTest(AltArray, READINGS_LENGTH);

  double rate_of_change = currentMedian - previousMedianTest;

  previousMedianTest = currentMedian;

  return rate_of_change;
}

int altCount = 0;
void BabyStateMachine(SixDOF &_6DOF1, PHT &_Alt, GPS &_GPS1)
 // Smaller scale, incomplete state machine used for testing.
 // 
{
    //Run an initial Senesor Check, verify 
    String userInput = "";
    switch(current_state1){
    case LaunchState::PreIgnition:{
        //Run an initial Senesor Check, verify 
        Serial.println(String(_6DOF1.getNetAccel()));
        double PHT_alt = _Alt.getAltitude();
        double GPS_alt = _GPS1.getAltitude();
        bool PHT_error = (PHT_alt == 0);
        bool GPS_error = !(GPS1.fix);
        double final_alt = 0;

        if(!PHT_error && !GPS_error){
          final_alt = (PHT_alt + GPS_alt) / 2;
        }
        else if (!PHT_error){
          final_alt = PHT_alt;
        }
        else if (!GPS_error){
          final_alt = GPS_alt;
        }

        AltArrayTest[altCount % 10] = final_alt;
        altCount  = (altCount + 1) % 10;
        
        char c = Serial.read();
            if (c == '\n') // if Enter pressed,
            {
                if(userInput == "ARM"){
                    Serial.println("ROCKET IS ARMED AND ON THE PAD. "); // MAYBE WE SHOULD HAVE AN AUDIBLE OR SOME PHYSICAL RESPONSE
                    delay(5000);
                    //Just for testing, each state will correspond to a solenoid.
                    //command2pin("01", '0', '1') PREIGNITION STATE PASSED
                    current_state1 = LaunchState::Ignition_to_Apogee;
                }
            }
            else{
                userInput += c;
            }
            if(_6DOF1.getNetAccel() >= 30){ // change to match flight accel
              preignitionCount += 1;
              Serial.println("PreignitionAccelReached");
            }
            if(preignitionCount > 4){
              allSol();
                Serial.println("Large acceleration experienced ");
                current_state1 = LaunchState::Ignition_to_Apogee;
            }
    //due to low priority of sensor readings, occasionally check every 8 seconds.
      //Save and Print GPS COORD;
      //Save and Print PHT Height;
      //Save and Print Acceleration;
      //Send CAN MSG, Compare with StateMachine2?
    delay(50);
    break;
    }
    case LaunchState::Ignition_to_Apogee: {
      Serial.println("Next state reached");
      double PHT_alt = _Alt.getAltitude();
        double GPS_alt = _GPS1.getAltitude();
        bool PHT_error = (PHT_alt == 0);
        bool GPS_error = !(GPS1.fix);
        double final_alt = 0;

        if(!PHT_error && !GPS_error){
          final_alt = (PHT_alt + GPS_alt) / 2; // should we try to detect outliers before, or use PHT altitude? GPS noise could make this relatively inaccurate
        }
        else if (!PHT_error){
          final_alt = PHT_alt;
        }
        else if (!GPS_error){
          final_alt = GPS_alt;
        }

        AltArrayTest[altCount % 10] = final_alt;
        altCount  += 1;
        if(altCount >= 10){
          double rate_of_change = calculateRateOfChangeTest(AltArrayTest, 10);
          if((fabs(rate_of_change))<0.1 || rate_of_change < 0){
            current_state1 = LaunchState::_1000ft;
          }
        }
      // delay(10000);
      break;
    }
    case LaunchState::_1000ft:{
      Serial.println("Next state reached");
      delay(10000);
      current_state1 = LaunchState::_900ft;
      break;
    }
    case LaunchState::_900ft:{
      current_state1 = LaunchState::_800ft;
      break;
    }
    case LaunchState::_800ft:{
      current_state1 = LaunchState::Descent;
      break;
    }
    case LaunchState::Descent:{
      current_state1 = LaunchState::Touchdown;
      break;
    }
    case LaunchState::Touchdown:{
      break;
    }

    }
}

uint8_t extraPrecision(double dataValue)
{
    int integerPart = static_cast<int>(dataValue);
    double decimalPart = dataValue - integerPart;
    int scaledDecimal = static_cast<int>(decimalPart * 10000);
    int filteredDecimal = scaledDecimal % 100;
    // if (scaledDecimal > 255)
    // {
    //     scaledDecimal = 255;
    // }
    uint8_t hexValue = 0x00 + static_cast<uint8_t>(filteredDecimal);
    return hexValue;
}

double returnAverageTest(double arr[], int number)
{
  double sum = 0;
  for (int count = 0; count < number - 1; count++)
  {
    sum += arr[count];
  }
  return sum / number;
}



// TWAI/CAN RECIEVE MESSAGE
void commandTask(String can_code) {
  //String canMessage = *(String *)pvParameters;  // Cast and dereference the passed parameter for FreeRTOS specific format

  while (1) {
    // Using the passed message
    String message = can_code;

    // Extract solboardIDnum, command, and mode from the message
    String solboardIDnum = message.substring(0, message.length() - 2);
    char command = message[message.length() - 2];
    char mode = message[message.length() - 1];

    switch (command) {
      case '0':
      case '1':
      case '2':
      case '3':
      case '4':
        command2pin(solboardIDnum, command, mode); // sends command to activate the solenoid HIGH
        break;
      default:
        vTaskDelay(10 / portTICK_PERIOD_MS);  // Delay for a while 
        yield();
        vTaskDelay(10);
    }
  }
}
void printTwaiStatus(){
  twai_status_info_t status;
  twai_get_status_info(&status);
  Serial.print("Status:  ");
  switch(status.state) {
    case(TWAI_STATE_STOPPED):
      Serial.println("Stopped");
      break;
   case(TWAI_STATE_RUNNING):
     Serial.println("Running");
     break;
   case(TWAI_STATE_BUS_OFF):
     Serial.println("Bus Off");
     break;
   case(TWAI_STATE_RECOVERING):
     Serial.println("Recovering");
     break;
   default:
     Serial.println("Unknown");
     break;
 }
 Serial.print("Tx Error Counter: ");
 Serial.println(status.tx_error_counter);
 Serial.print("Rx Error Counter: ");
 Serial.println(status.rx_error_counter);
 Serial.print("Bus Error Count: ");
 Serial.println(status.bus_error_count);
 Serial.print("Messages to Tx: ");
 Serial.println(status.msgs_to_tx);
 Serial.print("Messages to Rx: ");
 Serial.println(status.msgs_to_rx);
}

void setup()
{
  Serial.begin(921600);  // Initialize Serial communication
  Serial.println("Begin");
  while (!Serial)
  {
    Serial.print("Serial Failed to start");
    delay(10);
  }
  //CAN Setup:
  pinMode(CAN_TX, OUTPUT);
  pinMode(CAN_RX, INPUT);
  // Config CAN Speed
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); //TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  // Install and start the TWAI driver
  esp_err_t canStatus = twai_driver_install(&g_config, &t_config, &f_config);
  if (canStatus == ESP_OK) {
    Serial.println("CAN Driver installed");
  } else {
    Serial.println("CAN Driver installation failed");
  }
  if (twai_start() != ESP_OK) {
    Serial.println("Error starting TWAI!");
}
 //starts TWAI
  Serial.println("CAN/TWAI BUS STARTED");
// Create and assign tasks for each core
// xTaskCreatePinnedToCore(SixDOF_Task, "Six_DOF_Task", 2048, NULL, 1, NULL, 0);
// xTaskCreatePinnedToCore(PHT_Task, "PHT_Task", 2048, NULL, 2, NULL, 1); // PHT broke af rn
// xTaskCreatePinnedToCore(Test_Task, "Print", 2048, NULL, 2, NULL, 1);

  if (!(_6DOF.start_6DOF()))
  {
    Serial.println("6DOF Failed to start");
  }
  I2C1.begin(42, 41);
  if(!Alt.connectSensor()){
    Serial.println("Error connecting to PHT sensor");
  }
  else {
    Serial.println("Connected to Alt sensor");
    Alt.setSensorConfig();
  }
  GPS1.startGPS();
  // if(!Alt.startPHT()){
  //   Serial.println("Altimeter Failed to start");
  // }
  // Serial.println("Altimeter Started");       CHANGE ALTIMETER TO MSx07
  delay(300);
}

void loop()
{
  unsigned long past_time = millis();
// each pin works individually, but cannot actuate multiple at once. maybe a MSG priority issue
// BabyStateMachine(_6DOF); 
  // double pressure = Alt.getPressure();
  double altitude = Alt.getAltitude();
  // Serial.println("Pressure: " + String(pressure) + ", Altitude: " + String(altitude));
  Serial.println(String(altitude));
  // String accel = _6DOF.printSensorData();
  // Serial.println(accel);
  // Serial.println("Loop Time (ms): "+String(millis()-past_time)); // loop time
  // allSol();
  // printTwaiStatus();
  delay(100);
}