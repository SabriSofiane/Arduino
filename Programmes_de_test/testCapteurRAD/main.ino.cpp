#include <Arduino.h>
#include <Wire.h>
#include <BME280I2C.h>
#include <RadiationWatch.h>


//Le capteur lis les rayons gamma, unité: CPM

RadiationWatch radiationWatch(32,33);

void onRadiation()
{
 
  Serial.println("Un rayon gamma est apparu");
  Serial.print(radiationWatch.uSvh());
  Serial.print(" uSv/h +/- ");
  Serial.println(radiationWatch.uSvhError());
  Serial.print(" CPM : ");
  Serial.println(radiationWatch.cpm());  
}

void onNoise()
{
  Serial.println("Argh, bruit, SVP arreter de bouger");
}

void setup()
{
  Serial.begin(115200);
  radiationWatch.setup();
 
   //Register the callbacks.
  radiationWatch.registerRadiationCallback(&onRadiation);
  radiationWatch.registerNoiseCallback(&onNoise);
}

void loop()
{
  radiationWatch.loop();
}