#include <Arduino.h>
#include <Wire.h>
#include <BME280I2C.h>
#include <RadiationWatch.h>
#include <TinyGPS.h>
#include <HardwareSerial.h>
#define SERIAL_BAUD 115200
#define LED 22
SemaphoreHandle_t mutex;



RadiationWatch radiationWatch(32, 33);
TinyGPS gps;
HardwareSerial serialGps(2); // sur hardware serial 2


typedef struct {
  byte seconde;
  byte minute;
  byte heure;
} typeHeure;

typedef struct {
  byte jour;
  byte mois;
  unsigned int annee;
} typeDate;

typedef struct {
  float temperature;
  float humidite;
  float pression;
  float cpm;

} typeDonneesCapteurs;

typedef struct {
  float altitude;
  float latitude;
  float longitude;
} typePosition;

typedef struct {
  typePosition position;
  typeHeure heures;
  typeDate date;
  typeDonneesCapteurs DonneesCapteurs;
} typeDonnees;

typeDonneesCapteurs capteur;
typePosition donneesGPS;
typeDate dateGPS;
typeHeure heureGPS;


void tacheAffichage(void* parameter) {
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  delay(5000);

  for (;;) {
    //affichage des structures
    Serial.println("Structure GPS:");
    Serial.println(donneesGPS.latitude, 6);
    Serial.println(donneesGPS.longitude, 6);
    Serial.println(donneesGPS.altitude);
    Serial.println(dateGPS.jour);
    Serial.println(dateGPS.mois);
    Serial.println(dateGPS.annee);
    Serial.println(heureGPS.heure);
    Serial.println(heureGPS.minute);
    Serial.println(heureGPS.seconde);
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(40000)); // reveille toutes les 40s

  }

}

void tacheGPS(void* parameter) {


  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED, OUTPUT);

  Serial.print("Simple TinyGPS library v. ");
  Serial.println(TinyGPS::library_version());

  for (;;) {
    bool newData = false;
    unsigned long chars;
    unsigned short sentences, failed;

    // Pendant une seconde lecture des caractères envoyés par le GPS
    for (unsigned long start = millis(); millis() - start < 1000;) {
      while (serialGps.available()) {
        char c = serialGps.read();
        // Serial.write(c);
        if (gps.encode(c)) // si des caratères sont reçus alors newdata = true
          newData = true;
      }
    }

    if (newData) {
      float lat, lon, alt;
      unsigned long date, time;
      unsigned long age;
      //HEURE
      byte seconde;
      byte minute;
      byte heure;

      //DATE
      byte jour;
      byte mois;
      byte hundredths;
      int annee;
      gps.f_get_position(&lat, &lon, &age);
      gps.get_datetime(&date, &time, &age);
      gps.crack_datetime(&annee, &mois, &jour, &heure, &minute, &seconde, &hundredths, &age);
      //annee= annee %100;
      heure = heure + 2;
      Serial.print("LAT=");
      Serial.print(lat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : lat, 6);
      Serial.print(" LON=");
      Serial.print(lon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : lon, 6);
      //Serial.print(" SAT=");
      //Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
      //Serial.print(" PREC=");
      //Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
      Serial.print(" ALT=");
      Serial.println(alt = gps.f_altitude());
      Serial.print("date=");
      Serial.print(jour == TinyGPS::GPS_INVALID_DATE ? 0.0 : jour, 0);
      Serial.print("/");
      Serial.print(mois == TinyGPS::GPS_INVALID_DATE ? 0.0 : mois, 0);
      Serial.print("/");
      Serial.println(annee == TinyGPS::GPS_INVALID_DATE ? 0.0 : annee, 0);



      // Serial.println(date == TinyGPS::GPS_INVALID_DATE ? 0.0 : date, 0);
      Serial.print("heure=");
      Serial.print(heure == TinyGPS::GPS_INVALID_TIME ? 0.0 : heure, 0);
      Serial.print("h");
      Serial.print(minute == TinyGPS::GPS_INVALID_TIME ? 0.0 : minute, 0);
      Serial.print("m");
      Serial.println(seconde == TinyGPS::GPS_INVALID_TIME ? 0.0 : seconde, 0);
      xSemaphoreTake(mutex, portMAX_DELAY);
      donneesGPS.latitude = lat;
      donneesGPS.longitude = lon;
      donneesGPS.altitude = alt;
      dateGPS.jour = jour;
      dateGPS.mois = mois;
      dateGPS.annee = annee;
      heureGPS.heure = heure;
      heureGPS.minute = minute;
      heureGPS.seconde = seconde;
      xSemaphoreGive(mutex);
      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(30000)); // reveille toutes les 30s


    }

    gps.stats(&chars, &sentences, &failed);
    /*Serial.print(" CHARS=");
      Serial.print(chars);
      Serial.print(" SENTENCES=");
      Serial.print(sentences);
      Serial.print(" CSUM ERR=");
      Serial.println(failed);*/
    if (chars == 0)
      Serial.println("** No characters received from GPS: check wiring **");

  }

}

void setup() {
  Serial.begin(115200);
  serialGps.begin(4800, SERIAL_8N1, 16, 17);


  //mutex
  mutex = xSemaphoreCreateMutex();
  xTaskCreate(
    tacheGPS, /* Task function. */
    "tacheGPS", /* name of task. */
    10000, /* Stack size of task */
    NULL, /* parameter of the task */
    2, /* priority of the task */
    NULL); /* Task handle to keep track of created task */

  xTaskCreate(
    tacheAffichage, /* Task function. */
    "tacheAffichage", /* name of task. */
    2000, /* Stack size of task */
    NULL, /* parameter of the task */
    1, /* priority of the task */
    NULL); /* Task handle to keep track of created task */









}

void loop() {

}
