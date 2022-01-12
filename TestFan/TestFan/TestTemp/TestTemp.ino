// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 2
#define FAN_PIN 3

OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors(&oneWire);

DeviceAddress insideThermometer;

bool fanOn = false;
void setup(void)
{
  // start serial port
  Serial.begin(9600);
  Serial.println("Dallas Temperature IC Control Library Demo");
  pinMode(FAN_PIN, OUTPUT);

  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  
  Serial.print("Device 0 Address: ");
  printAddress(insideThermometer);
  Serial.println();

  sensors.setResolution(insideThermometer, 9);
 
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();
}

// function to print the temperature for a device
void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = sensors.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp C: ");
  Serial.println(tempC);
  Serial.print("Fan ");
  if(tempC >= 22){
      fanOn = true;
      Serial.println("ON ");
    }
  else {
      fanOn = false;
      Serial.println("OFF");
    }
    
}
void loop(void)
{ 
  delay(1000);
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); 
  Serial.println("DONE");
  printTemperature(insideThermometer);
  fanOnOff(fanOn);
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void fanOnOff(bool state){
  if(state){
  digitalWrite(FAN_PIN, HIGH);
  }else{
  digitalWrite(FAN_PIN, LOW);
  }
}
