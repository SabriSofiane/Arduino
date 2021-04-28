#include <Arduino.h>
#include <Wire.h>
#include <BME280I2C.h>
#define SERIAL_BAUD 115200

void printBME280Data(Stream* );

BME280I2C::Settings settings(
BME280::OSR_X1,
        BME280::OSR_X1,
        BME280::OSR_X1,   BME280::Mode_Forced,
        BME280::StandbyTime_1000ms,
        BME280::Filter_Off,
        BME280::SpiEnable_False,
        BME280I2C::I2CAddr_0x77      // 0x77 I2C address pour BME 280 Adafruit.
);
BME280I2C bme(settings);



void setup()
{
  Serial.begin(SERIAL_BAUD);

  while(!Serial) {} // Wait

  Wire.begin();

  while(!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  switch(bme.chipModel())
  {
     case BME280::ChipModel_BME280:
       Serial.println("Found BME280 sensor! Success.");
       break;
     case BME280::ChipModel_BMP280:
       Serial.println("Found BMP280 sensor! No Humidity available.");
       break;
     default:
       Serial.println("Found UNKNOWN sensor! Error!");
  }
}

//////////////////////////////////////////////////////////////////
void loop()
{
   printBME280Data(&Serial);
   delay(500);
}

//////////////////////////////////////////////////////////////////
void printBME280Data(Stream* client)
{
   float temp(NAN), hum(NAN), pres(NAN);
  
   int temperature;
   int humidite;
   BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
   BME280::PresUnit presUnit(BME280::PresUnit_hPa);

   bme.read(pres, temp, hum, tempUnit, presUnit);

   client->print("Temp: ");
   temperature = (int)temp;
   client->print(temp); // temp
   client->print("°"+ String(tempUnit == BME280::TempUnit_Celsius ? 'C' :'F'));
   client->print("\t\tHumidity: ");
   humidite = (int)hum;
   client->print(hum); //hum
   client->print("% RH");
   client->print("\t\tPressure: ");
   client->print(pres);
   client->println("hPa");

   //delay(60000); //Récuperation des données toutes les minutes
}