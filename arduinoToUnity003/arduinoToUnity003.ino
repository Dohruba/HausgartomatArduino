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
bool mayTurnOnLED = true;

//Temperature sensor variables////////////
bool fanIsOn = false;
bool mayTurnOnFan = true;

//Moisture Sensor variables////////
//A0 Value1: Air;
//A0 Value2: Water;
const int wet = 328;
const int dry = 796;

//Water Pump variables/////////////
bool pumpIsOn = false;
bool mayTurnOnPump = true;

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
  startTempSensor();
  ledOff();
  delay(1000);
}

void loop() {
  lightCheck();
  delay(1000);
  if (Serial.available() > 0)
      sCmd.readSerial();
}
////////////////////////////////////////////////////////////////////////////////////////////////
////////// Light Management //////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
void lightCheck(){
  light = analogRead(A0);
  //Serial.println(light);
  String states[6] = {"a","b","c","d","e","f"} ;
  writeToUnity(700,800,states, ledIsOn, light, mayTurnOnLED, "e");
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
void tempCheck(){
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
/**
 * Check earth humidity.
 */
void humidityCheck(){
   int sensorVal = analogRead(A1);
   int percentageHumidity = map(sensorVal, wet, dry, 100, 0); 
//For now between 40-60%
//If camera available, we could try to stablish humidity percentages by growing stages
   String states[6] = {"m","n","o","p","q","r"} ;
   writeToUnity(40,60,states, pumpIsOn, percentageHumidity, mayTurnOnPump, "q");
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
 * Function to write from arduino to the serial port, which will be read in Unity.
 * This delivers the "state" in which a condition of the plant finds itself. 
 * (Temperature, Humidity, Light quantity)
 * The "states" are defined as follows and apply to all the conditions:
 * There are 3 main states: "Too low", "Optimum" and "Too much" 
 * Each main state is divided in 2, depending on a relevant equipment being on or off.
 * For example: "Too low light, lamp off", "Too low light, lamp ON"
 * For a total of 6 States per conditon.
 * The order of the conditions must be from low to high, 
 * alternating the state of the relevant equipment, starting with "OFF".
 * If there is no equipment attached, then it´s state will always be "OFF".
 * 
 * noTurnOnCondition is the condition in wich an equipment may not be turned on,
 * to reduce the amount of possible errors from the user.
 * Example: "Earth too Wet, Pump OFF" or "Light too intense, LED off"
 */
void writeToUnity(int minimum, int maximum, String state[], 
                  bool equipmentON, int sensorValue, 
                  bool &mayTurnOn, String noTurnOnCondition){
  String actualState;
  //Too low
  if(sensorValue < minimum){
    if(!equipmentON){
      Serial.println(state[0]);
      actualState = state[0];
    }else{
      Serial.println(state[1]);
      actualState = state[1];
    }
  }
  //Optimum
  if(sensorValue >= minimum && sensorValue <= maximum){
    if(!equipmentON){
      Serial.println(state[2]);
      actualState = state[2];
    }else{
      Serial.println(state[3]);
      actualState = state[3];
    }
  }
  //Too high
  if(sensorValue > maximum){
    if(!equipmentON){
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
