//A0 Value1: Air = 812;
//A0 Value2: Water = 416;
const int wet = 386;
const int dry = 805;
void setup(){
  Serial.begin(9600);
}
void loop()
 {
   int sensorVal = analogRead(A0);
 
   int percentageHumididy = map(sensorVal, wet, dry, 100, 0); 

   Serial.print(sensorVal);
   Serial.print(",  ");
   Serial.print(percentageHumididy);
   Serial.println("%");
   
   delay(400);
 }
