// created by Sergej Kolmogorov, Instagram: https://www.instagram.com/sergej.kolmogorov/
// YouTube: https://www.youtube.com/@SergejKolmogorov
// Code is free to use after subscribing my channel =)

#include <Wire.h>
#include <Adafruit_INA238.h> // Falls Sie die Adafruit-Bibliothek nutzen

Adafruit_INA238 ina238;

// Pins definieren
const int statusIndikator = 40; // Pin zum Anzeigen von Batteriestatus
const int gatePin = 41; // Pin zum Schalten des MOSFETs

// Einstellungen für die Entladung
const float CUTOFF_VOLTAGE = 2.6; // Sichere Abschaltspannung in Volt
bool isDischarging = true;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wartet auf den Seriellen Monitor

  pinMode(statusIndikator, OUTPUT);
  digitalWrite(statusIndikator, HIGH);
  pinMode(gatePin, OUTPUT);
  digitalWrite(gatePin, LOW); // Sicherstellen, dass die Lampe anfangs AUS ist

  Serial.println("Initialisiere INA238...");
  if (!ina238.begin()) {
    Serial.println("Fehler: INA238 nicht gefunden! Überprüfe die Verkabelung.");
    while (1) delay(10);
  }
  Serial.println("INA238 erfolgreich verbunden.");

  float voltage = ina238.readBusVoltage(); // Gibt die Spannung in Volt zurück
  while(voltage < 0.1) {
    voltage = ina238.readBusVoltage();  // Spannung vom INA238 auslesen
    Serial.println(voltage);
    if(voltage > 0.1){
      Serial.println("Batterie angeschlossen");
    } else {
      delay(1000);
    }
  }

  if (voltage > (CUTOFF_VOLTAGE + 0.6)) {
      Serial.println("Batterie ausreichend geladen");
      digitalWrite(gatePin, HIGH); // Lampe einschalten
      Serial.println("Entladevorgang gestartet...");
      digitalWrite(statusIndikator, LOW); // Fertig Anzeige ausmachen
      isDischarging = true;
  } else {
    isDischarging = false;
    Serial.println("Batterie darf nicht weiter entladen werden!");
  }
  delay(3000);
}

void loop() {
  // Spannung vom INA238 auslesen
  float voltage = ina238.readBusVoltage(); // Gibt die Spannung in Volt zurück

  Serial.print("Aktuelle Batteriespannung: ");
  Serial.print(voltage, 3);
  Serial.println(" V ");

  if (isDischarging) {
    // Prüfen, ob die kritische Grenze erreicht oder unterschritten wurde
    if (voltage < CUTOFF_VOLTAGE) {
      digitalWrite(gatePin, LOW); // Lampe SOFORT ausschalten
      digitalWrite(statusIndikator, HIGH); // Entladung als fertig anzeigen
      isDischarging = false;
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      Serial.println("ACHTUNG: Zielspannung erreicht! Entladung GESTOPPT.");
      Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }
  } else {
      digitalWrite(gatePin, LOW);
      digitalWrite(statusIndikator, HIGH);
  }

  delay(1000); // Jede Sekunde messen
}
