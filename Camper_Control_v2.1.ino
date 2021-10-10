/*
  Hi Colin, that's cool.  If you want to put a nominal amount through the site to give you feedback and the rest direct, just let me know :)

  Changes I've made to make it work were:
  -Moved the code that seems to deal with the relay board i2c buffer back into void setup
  -Changed the data_from_display hadnling code at the start of the loop back to the iteration from sketch2.
*/

#include <Wire.h> //i2c Library for relay board
#include "DHT.h"   //https://github.com/adafruit/DHT-sensor-library 
#include <OneWire.h>   //I2C for Temp Probes
#include <DallasTemperature.h>   //Temp Probes

#define BOARD_1 0x25    // i2c slave address of BOARD #1
#define ON 1
#define OFF 0
#define AUXDC_PIN A0
#define MAINDC_PIN A1
//#define PVDC_PIN A2
#define WATER_PIN A3
#define AC_PIN A4
#define uCmonitor_PIN A5
#define LPG_PIN A6
#define ONE_WIRE_BUS 4      // i2C Configuration Data wire is plugged into pin 4 on the Arduino
#define IRFOOT_PIN 8
#define DEFWIPERS_PIN 9
#define DEFJETS_PIN 10


// timing intervals
const long tempDHT22_interval = 60000;
const long tempDS18B20_interval = 10000;
const long voltRead_interval = 60000;
const long waterRead_interval = 60000;

unsigned long prevT_tempDH = 0;
unsigned long prevT_tempDS = 0;
unsigned long prevT_voltRead = 0;
unsigned long prevT_waterRead = 0;

unsigned char i2c_buffer;
unsigned char i2c_buffer_1;   // i2c relay variable buffer for #1 relay board

//Data handling from HMI Display
String data_from_display = "";
int topLightsStatus = 0; // 0 = OFF, 1 = ON
int bunkLightsStatus = 0;
int mainLightsStatus = 0;
int sceneLightsStatus = 0;
int musicAmpStatus = 0;
int waterPumpStatus = 0;
int irFloorStatus = 0;
int spareRelayStatus = 0;
int irFootwellStatus = 0;
int defrostWipersStatus = 0;
int defrostJetsStatus = 0;

// DHT sensors
DHT dhtCab(5, DHT22);
DHT dhtBunk(6, DHT22);
DHT dhtDry(7, DHT22);
float dhtCab_temp, dhtBunk_temp, dhtDry_temp; // Add more if required
float dhtCab_humi, dhtBunk_humi, dhtDry_humi; // Add more if required

// DS18B20
OneWire oneWire(ONE_WIRE_BUS);     // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature.
DeviceAddress T1 = { 0x28, 0xFD, 0xAA, 0x6D, 0x30, 0x19, 0x01, 0x7B }; //Outdoor Temp
DeviceAddress T2 = { 0x28, 0x71, 0x5B, 0x5B, 0x30, 0x19, 0x01, 0xC6 }; //Floor Temp
DeviceAddress T3 = { 0x28, 0x23, 0x47, 0x07, 0xD6, 0x01, 0x3C, 0xDE }; //Cold Water Tank
DeviceAddress T4 = { 0x28, 0x5B, 0xEE, 0x9E, 0x30, 0x19, 0x01, 0x9A }; //Hot Water Tank

float temp1, temp2, temp3, temp4;

// AC voltage monitor
double sensorValue1 = 0;
double sensorValue2 = 0;
int crosscount = 0;
int climb_flag = 0;
int val[100];
int max_v = 0;
double VmaxD = 0;
double VeffD = 0;
double Veff = 0;

// DC voltage monitor
float CutV = 12.10; // The Min Voltage allowed
float RestoreV = 12.40; // The Max Voltage allowed
//Aux Battery
int VoltIn_a = 0;      //Analog Input
float Vout_a = 0.0;    //Voltage In after voltage divider
float V_a = 0.0;       //Actual voltage after calculation
float R1 = 9950.0;  //R1 100K   (Actual measured value)
float R2 = 1984.0;   //R2 10K   (Actual measured value)
///Main Battery
int mVoltIn_a = 0;      //Analog Input
float mVout_a = 0.0;    //Voltage In after voltage divider
float mV_a = 0.0;       //Actual voltage after calculation
float mR1 = 9830.0;  //R1 100K    (Actual measured value)
float mR2 = 1990.0;   //R2 10K    (Actual measured value)

// Water Volume Sensor
int16_t dataVoltage;

void setup() {

  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);

  pinMode(AUXDC_PIN, INPUT); // AUX battery DC volts in
  pinMode(MAINDC_PIN, INPUT); // Main battery DC volts in
  pinMode(AC_PIN, INPUT); // AC voltage pin
  pinMode(WATER_PIN, INPUT); //Waterpump
  pinMode(IRFOOT_PIN, OUTPUT); //IR Footwell
  pinMode(DEFWIPERS_PIN, OUTPUT); //Wipers Defrost Heater
  pinMode(DEFJETS_PIN, OUTPUT); //Washer Jets Heater


  Wire.begin();

  dhtCab.begin();
  dhtBunk.begin();
  dhtDry.begin();

  sensors.begin();  // Start up the Dallas Temperature library
  sensors.setResolution(T1, 8);
  sensors.setResolution(T2, 8);
  sensors.setResolution(T3, 8);
  sensors.setResolution(T4, 8);



}

void channel_mode(unsigned char addr, unsigned char channel, unsigned char value) {

  switch (addr) {
    case BOARD_1:
      i2c_buffer = i2c_buffer_1;
      break;
  }

  channel = 8 - channel;
  i2c_buffer &= ~(1 << (channel));
  i2c_buffer |= value << channel;


  switch (addr) {
    case BOARD_1: i2c_buffer_1 = i2c_buffer; break;
  }

  Wire.beginTransmission(addr);
  Wire.write(~i2c_buffer);
  Wire.endTransmission();

}

void loop() {

  unsigned long currentTime = millis();

  read_serial(); // HMI to Rely Board Control

  if (currentTime - prevT_tempDH >= tempDHT22_interval) { // DHT22 Sensors
    prevT_tempDH = millis();
    read_dht();
  }

  if (currentTime - prevT_tempDS >= tempDS18B20_interval) { // DS18B20 Sensors
    prevT_tempDS = millis();
    read_ds18b20();
  }

  if (currentTime - prevT_voltRead >= voltRead_interval) { // Voltage Monitoring
    prevT_voltRead = millis();
    get_ACvolts();
    get_DCvolts();
  }

  if (currentTime - prevT_waterRead >= waterRead_interval) { // Water Level
    prevT_waterRead = millis();
    measure_water();
  }

}

void measure_water() {

  dataVoltage = analogRead(WATER_PIN);



  Serial.print("Litres ");

  if (dataVoltage <= (98)) {
    Serial.println("0-5");
    Serial1.print(F("l0.txt=\""));
    Serial1.print("0-5 L");
    Serial1.print(F("\""));
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("l0.bco=63488");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  if (dataVoltage >= (99) && dataVoltage <= (101)) {
    Serial.println("6-10");
    Serial1.print(F("l0.txt=\""));
    Serial1.print("6-10 L");
    Serial1.print(F("\""));
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("l0.bco=64512");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  if (dataVoltage >= (102) && dataVoltage <= (104)) {
    Serial.println("11-15");
    Serial1.print(F("l0.txt=\""));
    Serial1.print("11-15 L");
    Serial1.print(F("\""));
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("l0.bco=11");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  if (dataVoltage >= (105) && dataVoltage <= (107)) {
    Serial.println("16-18");
    Serial1.print(F("l0.txt=\""));
    Serial1.print("16-18 L");
    Serial1.print(F("\""));
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.print("l0.bco=11");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }

  Serial.print("Voltage ");
  Serial.println(dataVoltage);




  if (temp3 >= 5) {                //Warning indicator for Cold Water Tank Level  - Normal
    Serial1.print("t6.bco=11");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  else if (temp3 <= 4 && temp3 >= 2) { //Warning indicator for Cold Water Tank Level - Amber
    Serial1.print("t6.bco=64512");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  else if (temp3 <= 1) {  //Warning indicator for Cold Water Tank Level - Red
    Serial1.print("t6.bco=63488");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }

}

void get_DCvolts() {

  // Aux Battery
  VoltIn_a = analogRead(AUXDC_PIN);              //Read analog values
  Vout_a = (VoltIn_a * 5.0) / 1024.0;     //Convert 10bit input to an actual voltage
  V_a = Vout_a / (R2 / (R1 + R2));        //Using the voltage divider formula, work out the input voltage

  Serial.print ("Aux Battery Voltage...\n\r");
  Serial.println (V_a);

  Serial1.print(F("v2.txt=\""));  //Updates text field on HMI with battery reading
  Serial1.print(V_a, 1);
  Serial1.print(F("\""));
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial2.print(F("v2.txt=\""));  //Updates text field on HMI with battery reading
  Serial2.print(V_a, 1);
  Serial2.print(F("\""));
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  if (V_a >= 13.9) {                //Warning indicator for Aux Batt  - Charge
    Serial1.print("v2.bco=1413");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  else if (V_a <= 13.8 && V_a >= 13) { //Warning indicator for Aux Batt - Normal
    Serial1.print("v2.bco=11");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }

  else if (V_a <= 13 && V_a >= 12.4) { //Warning indicator for Aux Batt - Amber
    Serial1.print("v2.bco=64512");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  else if (V_a <= 12.3) {  //Warning indicator for Aux Batt - Red
    Serial1.print("v2.bco=63488");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }

  if (V_a >= 13.9) {                //Warning indicator for Aux Batt  - Charge
    Serial2.print("v2.bco=1413");
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);
  }
  else if (V_a <= 13.8 && V_a >= 13) { //Warning indicator for Aux Batt - Normal
    Serial2.print("v2.bco=11");
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);
  }

  else if (V_a <= 13 && V_a >= 12.4) { //Warning indicator for Aux Batt - Amber
    Serial2.print("v2.bco=64512");
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);
  }
  else if (V_a <= 12.3) {  //Warning indicator for Aux Batt - Red
    Serial2.print("v2.bco=63488");
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);
  }



  //Main Battery
  mVoltIn_a = analogRead(MAINDC_PIN);              //Read analog values
  mVout_a = (mVoltIn_a * 5.0) / 1024.0;     //Convert 10bit input to an actual voltage
  mV_a = mVout_a / (mR2 / (mR1 + mR2));        //Using the voltage divider formula, work out the input voltage

  /* I ADDED THE 2 LINES BELOW AS YOU DID THE SAME WITH AUX VOLTAGE ABOVE */
  Serial.print ("Main Battery Voltage...\n\r");
  Serial.println (mV_a);

  // Main battery output to HMI
  Serial1.print(F("v1.txt=\""));  //Updates text field on HMI with battery reading
  Serial1.print(mV_a, 1);
  Serial1.print(F("\""));
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial2.print(F("v1.txt=\""));  //Updates text field on HMI with battery reading
  Serial2.print(mV_a, 1);
  Serial2.print(F("\""));
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  if (mV_a >= 13.9) {                //Warning indicator for Main Batt  - Charge
    Serial1.print("v1.bco=1413");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  else if (mV_a <= 13.8 && mV_a >= 13) { //Warning indicator for Main Batt - Normal
    Serial1.print("v1.bco=11");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }

  else if (mV_a <= 13 && mV_a >= 12.4) { //Warning indicator for Main Batt - Amber
    Serial1.print("v1.bco=64512");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }
  else if (mV_a <= 12.3) {  //Warning indicator for Main Batt - Red
    Serial1.print("v1.bco=63488");
    Serial1.write(0xff);
    Serial1.write(0xff);
    Serial1.write(0xff);
  }

  if (mV_a >= 13.9) {                //Warning indicator for Main Batt  - Charge
    Serial2.print("v1.bco=1413");
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);
  }
  else if (mV_a <= 13.8 && mV_a >= 13) { //Warning indicator for Main Batt - Normal
    Serial2.print("v1.bco=11");
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);
  }

  else if (mV_a <= 13 && mV_a >= 12.4) { //Warning indicator for Main Batt - Amber
    Serial2.print("v1.bco=64512");
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);
  }
  else if (mV_a <= 12.3) {  //Warning indicator for Main Batt - Red
    Serial2.print("v1.bco=63488");
    Serial2.write(0xff);
    Serial2.write(0xff);
    Serial2.write(0xff);
  }
}

void get_ACvolts() {

  for ( int i = 0; i < 100; i++ ) {
    sensorValue1 = analogRead(AC_PIN);
    if (sensorValue1 > 511) {
      val[i] = sensorValue1;
    }
    else {
      val[i] = 0;
    }
    delay(1);
  }

  max_v = 0;

  for ( int i = 0; i < 100; i++ )
  {
    if ( val[i] > max_v )
    {
      max_v = val[i];
    }
    val[i] = 0;
  }
  if (max_v != 0) {


    VmaxD = max_v;
    VeffD = VmaxD / sqrt(2);
    Veff = (((VeffD - 420.76) / -90.24) * -210.2) + 210.2;
  }
  else {
    Veff = 0;
  }
  Serial.print("Voltage: ");
  Serial.println(Veff);
  VmaxD = 0;

  /* ONCE THE SKETCH IS WORKING, IT WOULD BE WORTH TRYING THIS FUNCTION WITHOUT DELAYS */
  delay(100);

}

void printTemperature(DeviceAddress deviceAddress) {

  float tempC = sensors.getTempC(deviceAddress);
  if (tempC == -127.00) {
    Serial.print("Error getting temperature");
  } else {
    Serial.print("C: ");
    Serial.print(tempC);
  }

}

void read_ds18b20() {

  temp1 = sensors.getTempC(T1);
  temp2 = sensors.getTempC(T2);
  temp3 = sensors.getTempC(T3);
  temp4 = sensors.getTempC(T4);


  //Send temps to displays
  Serial1.print(F("t3.txt=\""));  //prints to text field t3 on HMI
  Serial1.print(temp1, 1);
  Serial1.print(F("\""));
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial2.print(F("t3.txt=\""));  //prints to text field t3 on HMI
  Serial2.print(temp1, 1);
  Serial2.print(F("\""));
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial1.print(F("t5.txt=\""));
  Serial1.print(temp2, 1);
  Serial1.print(F("\""));
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print(F("t6.txt=\""));
  Serial1.print(temp3, 1);
  Serial1.print(F("\""));
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print(F("t7.txt=\""));
  Serial1.print(temp4, 1);
  Serial1.print(F("\""));
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
  {
    if (temp3 >= 5) {                //Warning indicator for Cold Water Tank  - Normal
      Serial1.print("t6.bco=11");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
    else if (temp3 <= 4 && temp3 >= 2) { //Warning indicator for Cold Water Tank - Amber
      Serial1.print("t6.bco=64512");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
    else if (temp3 <= 1) {  //Warning indicator for Cold Water Tank - Red
      Serial1.print("t6.bco=63488");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
  }
  {
    if (temp3 >= 5) {                //Warning indicator for Hot Water Tank  - Normal
      Serial1.print("t7.bco=11");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
    else if (temp3 <= 4 && temp3 >= 2) { //Warning indicator for Cold Water Tank - Amber
      Serial1.print("t7.bco=64512");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
    else if (temp3 <= 1) {  //Warning indicator for Cold Water Tank - Red
      Serial1.print("t7.bco=63488");
      Serial1.write(0xff);
      Serial1.write(0xff);
      Serial1.write(0xff);
    }
  }

  Serial.print("Getting temperatures...\n\r");
  sensors.requestTemperatures();

  Serial.print("T1 = ");
  printTemperature(T1);
  Serial.print("\n\r");
  Serial.print("T2 = ");
  printTemperature(T2);
  Serial.print("\n\r");
  Serial.print("T3 = ");
  printTemperature(T3);
  Serial.print("\n\r");
  Serial.print("T4 = ");
  printTemperature(T4);
  Serial.print("\n\r");



}

void read_dht() {

  // Read DHT temperature and humidity values
  dhtCab_temp = dhtCab.readTemperature();
  dhtCab_humi = dhtCab.readHumidity();

  dhtBunk_temp = dhtBunk.readTemperature();
  dhtBunk_humi = dhtBunk.readHumidity();

  dhtDry_temp = dhtDry.readTemperature();
  dhtDry_humi = dhtDry.readHumidity();

  Serial.print("Cab Readings:  ");
  Serial.print(dhtCab_temp, 1); Serial.print("\xc2\xb0"); Serial.print("C  ");
  Serial.print(dhtCab_humi, 1); Serial.println("%RH");
  Serial.println();

  Serial.print("Bunk Room Readings:  ");
  Serial.print(dhtBunk_temp, 1); Serial.print("\xc2\xb0"); Serial.print("C  ");
  Serial.print(dhtBunk_humi, 1); Serial.println("%RH");
  Serial.println();

  Serial.print("Dry Room Readings:  ");
  Serial.print(dhtDry_temp, 1); Serial.print("\xc2\xb0"); Serial.print("C  ");
  Serial.print(dhtDry_humi, 1); Serial.println("%RH");
  Serial.println();

  Serial1.print("t1.txt=Test");
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  //send temps to displays
  Serial1.print(F("t1.txt=\""));
  Serial1.print(dhtBunk_temp, 1);
  Serial1.print(F("\""));
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print("h1.txt=\"");
  Serial1.print(dhtBunk_humi, 1);
  Serial1.print("\"");
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print(F("t2.txt=\""));
  Serial1.print(dhtCab_temp, 1);
  Serial1.print(F("\""));
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial2.print(F("t2.txt=\""));
  Serial2.print(dhtCab_temp, 1);
  Serial2.print(F("\""));
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial1.print(F("h2.txt=\""));
  Serial1.print(dhtCab_humi, 1);
  Serial1.print(F("\""));
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);

  Serial1.print(F("t4.txt=\""));
  Serial1.print(dhtDry_temp, 1);
  Serial1.print(F("\""));
  Serial1.write(0xff);
  Serial1.write(0xff);
  Serial1.write(0xff);
}

void read_serial() {

  while (Serial1.available() > 0) {

    char c = Serial1.read();
    data_from_display += c;
    delay(50);

  }

  while (Serial2.available() > 0) {

    char c = Serial2.read();
    data_from_display += c;
    delay(50);

  }

  data_from_display.trim();

  if (data_from_display.length() > 0) {

    Serial.println(data_from_display);
    toggle_relay(data_from_display); // if serial data is present, call the function
    data_from_display = ""; // reset this to empty

  }

}

void toggle_relay(String data_from_display) {

  Serial.print("Toggle");

  if (data_from_display == "c") {

    Serial.println(" Top Lights");

    if (topLightsStatus == 0) { // light is OFF

      topLightsStatus = 1; // turn it ON
      channel_mode(BOARD_1, 2, ON);
      Serial.println("Top Lights ON");

    }

    else if (topLightsStatus == 1) { // light is ON

      topLightsStatus = 0; // turn it OFF
      channel_mode(BOARD_1, 2, OFF);
      Serial.println("Top Lights OFF");

    }
  }

  else if (data_from_display == "a") {

    Serial.println(" IR Floor");

    if (irFloorStatus == 0) { // light is OFF

      irFloorStatus = 1; // turn it ON
      channel_mode(BOARD_1, 7, ON);
      Serial.println("IR Floor ON");

    }

    else if (irFloorStatus == 1) { // light is ON

      irFloorStatus = 0; // turn it OFF
      channel_mode(BOARD_1, 7, OFF);
      Serial.println("IR Floor OFF");

    }
  }

  else if (data_from_display == "b") {

    Serial.println(" Bunk Lights");

    if (bunkLightsStatus == 0) { // light is OFF

      bunkLightsStatus = 1; // turn it ON
      channel_mode(BOARD_1, 1, ON);
      Serial.println("Bunk ON");

    }

    else if (bunkLightsStatus == 1) { // light is ON

      bunkLightsStatus = 0; // turn it OFF
      channel_mode(BOARD_1, 1, OFF);
      Serial.println("Bunk OFF");

    }
  }

  else if (data_from_display == "d") {

    Serial.println(" Main Light");

    if (mainLightsStatus == 0) { // light is OFF

      mainLightsStatus = 1; // turn it ON
      channel_mode(BOARD_1, 3, ON);
      Serial.println("Main ON");

    }

    else if (mainLightsStatus == 1) { // light is ON

      mainLightsStatus = 0; // turn it OFF
      channel_mode(BOARD_1, 3, OFF);
      Serial.println("Main OFF");

    }
  }

  else if (data_from_display == "e") {

    Serial.println(" Scene Lights");

    if (sceneLightsStatus == 0) { // light is OFF

      sceneLightsStatus = 1; // turn it ON
      channel_mode(BOARD_1, 4, ON);
      Serial.println("Scene ON");

    }

    else if (sceneLightsStatus == 1) { // light is ON

      sceneLightsStatus = 0; // turn it OFF
      channel_mode(BOARD_1, 4, OFF);
      Serial.println("Scene OFF");

    }
  }

  else if (data_from_display == "f") {

    Serial.println(" Music Amp");

    if (musicAmpStatus == 0) { // light is OFF

      musicAmpStatus = 1; // turn it ON
      channel_mode(BOARD_1, 5, ON);
      Serial.println("Music ON");

    }

    else if (musicAmpStatus == 1) { // light is ON

      musicAmpStatus = 0; // turn it OFF
      channel_mode(BOARD_1, 5, OFF);
      Serial.println("Music OFF");

    }
  }

  else if (data_from_display == "g") {

    Serial.println(" Water Pump");

    if (waterPumpStatus == 0) { // light is OFF

      waterPumpStatus = 1; // turn it ON
      channel_mode(BOARD_1, 6, ON);
      Serial.println("Water ON");

    }

    else if (waterPumpStatus == 1) { // light is ON

      waterPumpStatus = 0; // turn it OFF
      channel_mode(BOARD_1, 6, OFF);
      Serial.println("Water OFF");

    }
  }
  else if (data_from_display == "F") {

    Serial.println(" IR Footwell");

    if (irFootwellStatus == 0) { // light is OFF

      irFootwellStatus = 1; // turn it ON
      digitalWrite (IRFOOT_PIN, HIGH);
      Serial.println("IR Footwell ON");

    }

    else if (irFootwellStatus == 1) { // light is ON

      irFootwellStatus = 0; // turn it OFF
      digitalWrite (IRFOOT_PIN, LOW);
      Serial.println("IR Footwell OFF");

    }
  }
  else if (data_from_display == "W") {

    Serial.println(" Wipers Defrost");

    if (defrostWipersStatus == 0) { // Heater is OFF

      defrostWipersStatus = 1; // turn it ON
      digitalWrite (DEFWIPERS_PIN, HIGH);
      Serial.println("WIpers Heater ON");

    }

    else if (defrostWipersStatus == 1) { // Heater is ON

      defrostWipersStatus = 0; // turn it OFF
      digitalWrite (DEFWIPERS_PIN, LOW);
      Serial.println("Wipers Heater OFF");

    }
  }
  else if (data_from_display == "J") {

    Serial.println(" Washer Jets Defrost");

    if (defrostJetsStatus == 0) { // Heater is OFF

      defrostJetsStatus = 1; // turn it ON
      digitalWrite (DEFJETS_PIN, HIGH);
      Serial.println("Jets Heater ON");

    }

    else if (defrostJetsStatus == 1) { // Heater is ON

      defrostJetsStatus = 0; // turn it OFF
      digitalWrite (DEFJETS_PIN, LOW);
      Serial.println("Jets Heater OFF");

    }
  }
}
