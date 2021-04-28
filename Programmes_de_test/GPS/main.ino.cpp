#include <TinyGPS.h>

#include <HardwareSerial.h>



#define LED 22

TinyGPS gps;
HardwareSerial serialGps(2); // sur hardware serial 2

void setup() {
    Serial.begin(115200);
    serialGps.begin(4800, SERIAL_8N1, 16, 17);

    // initialize digital pin LED_BUILTIN as an output.
    pinMode(LED, OUTPUT);

    Serial.print("Simple TinyGPS library v. ");
    Serial.println(TinyGPS::library_version());
}

void loop() {
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
        float lat, lon;
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
        heure = heure + 1;
        Serial.print("LAT=");
        Serial.print(lat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : lat, 6);
        Serial.print(" LON=");
        Serial.print(lon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : lon, 6);
        //Serial.print(" SAT=");
        //Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
        //Serial.print(" PREC=");
        //Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
        Serial.print(" ALT=");
        Serial.println(gps.f_altitude());
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



