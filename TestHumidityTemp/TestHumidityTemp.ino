#include "dht.h"
dht DHT;
#define DHT11_PIN 4
void setup(){
  pinMode(4,INPUT);
Serial.begin(9600);
}
void loop()
{
int d = DHT.read11(DHT11_PIN);
Serial.print("Temperature = ");
Serial.println(DHT.temperature);
Serial.print("Humidity = ");
Serial.println(DHT.humidity);
delay(1000);
}
