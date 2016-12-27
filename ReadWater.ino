int ReadWater(int pin)
{
   
   int sensorValue= analogRead(pin);
    if (sensorValue >= 820 && oldWaterValue<820)
 {
 Serial.println("Reading very high - no water");
 send(msgWater.set(false));
}
else if (sensorValue >= 615  && sensorValue < 820 && oldWaterValue<615)
 {
 Serial.println(" high - some water detected");
 send(msgWater.set(false));
}  
else if (sensorValue >= 410 && sensorValue < 615 && oldWaterValue>615)
 {
 Serial.println(" medium - more water detected - TRIGGER THE ALARM");
 send(msgWater.set(true));
 
}    
else if (sensorValue >= 250 && sensorValue < 410 && oldWaterValue>410)
 {
 Serial.println(" low - lots of water detected!!");
 send(msgWater.set(true));
}
else if (sensorValue >= 0 && sensorValue < 250 && oldWaterValue>250)
 {
 Serial.println(" very low - FLOODING!");
 send(msgWater.set(true));
}
 oldWaterValue=sensorValue;
 return(sensorValue);
}

