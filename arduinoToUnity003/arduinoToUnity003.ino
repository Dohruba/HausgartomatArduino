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

//Light sensor///////////
int light = 0;
bool ledIsOn = false;
bool mayTurnOnLED = false;

//Temperature////////////
bool fanIsOn = false;
bool mayTurnOnFan = false;

//Moisture Sensor////////
//A0 Value1: Air;
//A0 Value2: Water;
const int wet = 328;
const int dry = 796;

//Water Pump/////////////
bool pumpIsOn = false;
bool mayTurnOnPump = false;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  pinMode(LED, OUTPUT);
  pinMode(FAN, OUTPUT);
  sCmd.addCommand("LEDUP", ledOn);
  sCmd.addCommand("LEDDOWN", ledOff);
  sCmd.addCommand("FANUP", fanOn);
  sCmd.addCommand("FANDOWN", fanOff);
  startTempSensor();
  delay(1000);
}

void loop() {
  ledCheck();
  delay(150);
  fanCheck();
  delay(150);
  checkHumidity();
  delay(150);
  if (Serial.available() > 0)
      sCmd.readSerial();
  delay(1000);
  /*Serial.println("Lamp, Fan, Pump may turn on: " );
  Serial.println(mayTurnOnLED);
  Serial.println(mayTurnOnFan);
  Serial.println(mayTurnOnPump);*/
}
////////////////////////////////////////////////////////////////////////////////////////////////
////////// Light Management //////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
void ledCheck(){
  light = analogRead(A0);
  String states[6] = {"a","b","c","d","e","f"} ;
  writeToUnity(100,200,states, ledIsOn, light, mayTurnOnLED, "e");
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
////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////// Temp Management ////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
//void writeToUnity(int minimum, int maximum, char state[], bool sensor, int value){
void fanCheck(){
  sensors.requestTemperatures();
  float tempC = printTemperature(insideThermometer);
  String states[6] = {"g","h","i","j","k","l"} ;
  writeToUnity(18,20,states, fanIsOn, tempC, mayTurnOnFan,"g");  
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

void checkHumidity(){
   int sensorVal = analogRead(A1);
   int percentageHumidity = map(sensorVal, wet, dry, 100, 0); 
//For now between 40-60%
//If camera available, we could try to stablish humidity percentages by growing stages
   String states[6] = {"m","n","o","p","q","r"} ;
   writeToUnity(40,60,states, false, percentageHumidity, mayTurnOnPump, "q");
}

void switchPin(int pinNumber, bool sensorState){
  if(sensorState){
    digitalWrite(pinNumber, HIGH);
  }else{
    digitalWrite(pinNumber,LOW);
  }
}

void writeToUnity(int minimum, int maximum, String state[], 
                  bool sensor, int value, 
                  bool &mayTurnOn, String noTurnOnCondition){
  String actualState;
  if(value < minimum){
    if(!sensor){
      Serial.println(state[0]);
      actualState = state[0];
    }else{
      Serial.println(state[1]);
      actualState = state[1];
    }
  }
  if(value >= minimum && value <= maximum){
    if(!sensor){
      Serial.println(state[2]);
      actualState = state[2];
    }else{
      Serial.println(state[3]);
      actualState = state[3];
    }
  }
  if(value > maximum){
    if(!sensor){
      Serial.println(state[4]);
      actualState = state[4];
    }else{
      Serial.println(state[5]);
      actualState = state[5];
    }
  }
  if(actualState==noTurnOnCondition)
    mayTurnOn = false;
  else
    mayTurnOn = true;
  Serial.flush();
  delay(20);
}
