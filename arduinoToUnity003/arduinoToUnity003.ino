#include <SoftwareSerial.h>
#include <SerialCommand.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define LED 13
#define FAN 3
#define TEMP 2
#define PUMP 4
SerialCommand sCmd;
OneWire oneWire(TEMP);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;

//Light sensor variables ///////////
int light = 0;
bool ledIsOn = false;

//Temperature sensor variables////////////
int tempCs = 0;
bool fanIsOn = false;

//Moisture Sensor variables////////
//A0 Value1: Air;
//A0 Value2: Water;
const int wet = 359;
const int dry = 763;

//Water Pump variables/////////////
int percentageHumidity = 0;
bool pumpIsOn = false;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  
  pinMode(LED, OUTPUT);
  pinMode(FAN, OUTPUT);
  pinMode(PUMP, OUTPUT);
  
  sCmd.addCommand("LEDUP", ledOn);
  sCmd.addCommand("LEDDOWN", ledOff);
  sCmd.addCommand("FANUP", fanOn);
  sCmd.addCommand("FANDOWN", fanOff);
  sCmd.addCommand("PUMPUP", pumpOn);
  sCmd.addCommand("PUMPDOWN", pumpOff);

  sCmd.addCommand("GETLIGHT", sendLightToUnity);
  sCmd.addCommand("GETTEMP", sendTempToUnity);
  sCmd.addCommand("GETHUMIDITY", sendHumidityToUnity);
  startTempSensor();
  ledOff();
  fanOff();
  pumpOff();
  delay(1000);
}

void loop() {
  lightCheck();
  delay(200);
  tempCheck();
  delay(200);
  humidityCheck();
  delay(200);
  if (Serial.available() > 0)
      sCmd.readSerial();
  delay(200);
}
////////////////////////////////////////////////////////////////////////////////////////////////
////////// Light Management //////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
void lightCheck(const char *command){
  void lightCheck();
}
void lightCheck(){
  light = analogRead(A0);
  //Serial.println(light);
}
void sendLightToUnity(const char *command){
  sendLightToUnity();
  }
void sendLightToUnity(){
  writeToUnity(ledIsOn, light, "l");
}

void ledOn (const char *command){
  ledOn();
}
void ledOn (){
  ledIsOn = true;
  switchPin(LED,ledIsOn);
}
void ledOff (const char *command){
  ledOff();
}
void ledOff (){
    ledIsOn = false;
    switchPin(LED,ledIsOn);
}

/**
 * Voltage to Lux (Lumen/m^2) L = BR^m 
 * L: Light intensity
 * B:
 * R: Resistance
 * m: 
 */
void voltageToLux (){
  
}
////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////// Temp Management ////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
void tempCheck(const char *command){
  tempCheck();
}
void tempCheck(){
  sensors.requestTemperatures();
  tempCs = printTemperature(insideThermometer);
  //writeToUnity(fanIsOn, tempCs, "t");
}
void sendTempToUnity(const char *command){
  sendTempToUnity();
  }
void sendTempToUnity(){
  writeToUnity(fanIsOn, tempCs, "t");
}


void fanOn (const char *command){
  fanOn();
}
void fanOn (){
  fanIsOn=true;
  switchPin(FAN,fanIsOn);
}
void fanOff (const char *command){
  fanOff();
}
void fanOff (){
  fanIsOn=false;
  switchPin(FAN,fanIsOn);
}
////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////// Temp Measurement ////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

void startTempSensor(){
  sensors.begin();
  Serial.print(sensors.getDeviceCount(), DEC);
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  printAddress(insideThermometer);
  sensors.setResolution(insideThermometer, 9);
}

// function to print the temperature for a device
float printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    //Serial.println("Error: Could not read temperature data");
    return;
  }
  //Serial.print("Temp C: ");
  //Serial.println(tempC);

  return tempC;
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    //if (deviceAddress[i] < 16) Serial.print("0");
    //Serial.print(deviceAddress[i], HEX);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// Capacitive Moisture Sensor /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Check earth humidity.
 */
void humidityCheck(const char *command){
  humidityCheck();
}
void humidityCheck(){
   int sensorVal = analogRead(A1);
   percentageHumidity = map(sensorVal, wet, dry, 100, 0); 
//For now between 40-60%
   //writeToUnity(pumpIsOn, percentageHumidity, "h");
}

void sendHumidityToUnity(const char *command){
  sendHumidityToUnity();
  }
void sendHumidityToUnity(){
  writeToUnity(pumpIsOn, percentageHumidity, "h");
}

void pumpOn (const char *command){
  pumpOn();
}
void pumpOn (){
  if(!pumpIsOn){
    pumpIsOn = true;
    switchPin(PUMP,pumpIsOn);
  }
}
void pumpOff (const char *command){
  pumpOff();
}
void pumpOff (){
  if(pumpIsOn){
    pumpIsOn = false;
    switchPin(PUMP,pumpIsOn);
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// General Functions /////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Function to turn an equipment on or off, depending on it´s desired state.
 */
void switchPin(int pinNumber, bool equipmentState){
  if(equipmentState){
    digitalWrite(pinNumber, HIGH);
  }else{
    digitalWrite(pinNumber,LOW);
  }
}
/**
 * 
 */
void writeToUnity(bool equipmentON, int sensorValue, String type){
  String actualState = "";
  actualState.concat(type);
  actualState.concat(" ");
  actualState.concat(sensorValue);  
  actualState.concat(" ");
  if(equipmentON){
    actualState.concat(1);
  }else{
    actualState.concat(0);
  }
  Serial.println(actualState);
  Serial.flush();
  delay(50);
}

void checkSensorStatus(){
  //Must implement
}

//Have to update the write to Unity method, so that it works with the new schema
