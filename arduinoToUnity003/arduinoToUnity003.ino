#include <SoftwareSerial.h>
#include <SerialCommand.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define LED 13
#define FAN 3
#define TEMP 2
SerialCommand sCmd;
OneWire oneWire(TEMP);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;
/*
Blue led by the sensor, no external delivers 140-180
Blue led plus flash (Ambient) from 20cm(ambient) delivers over 300
No Led, No external, under 50
No Led, Flash from 20cm(ambient), 140-180

flash lvl 4 samambientg a70, from 20~ cm
resistor 1kOhm
*/
//Light sensor stuff
int light = 0;
bool ledIsOn = true;


//Temperature stuff
bool fanIsOn = false;


//Moisture Sensor
//A0 Value1: Air;
//A0 Value2: Water;
const int wet = 328;
const int dry = 796;



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
  
  //Temporary for test
  ledOn();
  fanOn();
  delay(3000);
  ledOff();
  fanOff();
  //Temp end
  
  delay(2000);
}

void loop() {
  ledCheck();
  delay(100);
  fanCheck();
  delay(100);
  checkHumidity();
  delay(100);
  if (Serial.available() > 0)
      sCmd.readSerial();
  delay(1000);
}
////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////// Light Management ////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

void ledCheck(){
  light = analogRead(A0);
  //Serial.print("Amount of light: ");
  //Serial.print(light);
  //Serial.print(", "); 
  // Case 1
  // Too much light, Led + ambient 
  // Case 2
  // Too much light, Only ambient
  if(light > 200){
    //ledOff();//TESTING
    if(ledIsOn){
      //Serial.println("Case 1");
      Serial.println("a");
    }else {
      //Serial.println("Case 2");
      Serial.println("b");
    }
    Serial.flush();
    delay(20);
    }

  // Case 3
  // Good enough light, ambient + led
  // Case 4
  // Good enough light, only ambient
  if(light < 201 && light > 100){
    if(ledIsOn){
      //Serial.println("Case 3");
      Serial.println("c");
    }else {
      //Serial.println("Case 4");
      Serial.println("d");
    }
    Serial.flush();
    delay(20);
    
    }
  // Case 5
  // Too little light, ambient only
  // Case 6
  // Too little light, ambient + ledOn (busted, obstructed)
  if(light < 100){
    //ledOn ();//TESTING
    if(!ledIsOn){
      //Serial.println("Case 5");
      Serial.println("e");
    }else {
      //Serial.println("Case 6");
      Serial.println("f");
    }
    Serial.flush();
    delay(20);
    }
}

void ledOn (const char *command){
  ledOn();
}
void ledOn (){
  if(!ledIsOn){
  digitalWrite(LED, LOW);
  ledIsOn = true;
  }
}
void ledOff (const char *command){
  ledOff();
}
void ledOff (){
  if(ledIsOn){
  digitalWrite(LED, HIGH);
  ledIsOn = false;
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////// Temp Management ////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

void fanCheck(){
  sensors.requestTemperatures();
  float tempC = printTemperature(insideThermometer);
  //Serial.println(tempC);
  // Case 7
  // Too hot, fan on 
  // Case 8
  // Too hot, fan off
  if(tempC > 20){
    //ledOff();//TESTING
    if(fanIsOn){
      //Serial.println("Case 7");
      Serial.println("g");
    }else {
      Serial.println("h");
    }
    Serial.flush();
    delay(20);
    }

  // Case 9
  //  Good, fan on
  // Case 10
  // Good, fan off
  if(tempC > 18 && tempC <= 20){
    if(fanIsOn){
      Serial.println("i");
    }else {
      Serial.println("j");
    }
    Serial.flush();
    delay(20);
    
    }
  // Case 11
  // Too cold, fan on
  // Case 12
  // // Too cold, fan off
  if(tempC <= 18){
    //fanOn ();//TESTING
    if(fanIsOn){
      Serial.println("k");
    }else {
      Serial.println("l");
    }
    Serial.flush();
    delay(20);
    }
}

void fanOn (const char *command){
  fanOn();
}
void fanOn (){
  if(!fanIsOn){
  digitalWrite(FAN, HIGH);
  fanIsOn = true;
  }
}
void fanOff (const char *command){
  fanOff();
}
void fanOff (){
  if(fanIsOn){
  digitalWrite(FAN, LOW);
  fanIsOn = false;
  }
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
   /*Serial.print(sensorVal);
   Serial.print(",  ");
   Serial.print(percentageHumididy);
   Serial.println("%");*/
   //States m,n
   Serial.println(percentageHumidity);
   if(percentageHumidity < 60 && percentageHumidity > 40){
      Serial.println("m");// Good, pump should be off
   }else if(percentageHumidity < 40){
      Serial.println("n");//Turn on pump
   }else if(percentageHumidity > 60){
      Serial.println("o");// Require User Attention
   }
   
   delay(400);
}
