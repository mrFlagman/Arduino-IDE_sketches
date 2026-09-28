// created by Sergej Kolmogorov, Instagram: https://www.instagram.com/sergej.kolmogorov/
// YouTube: https://www.youtube.com/@SergejKolmogorov
// Code is free to use after subscribing my channel =)

const int hallPin = 14;   // Teensy Pin 14
const int ledPin  = 40;
int minValue = 1000;
int maxValue = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  int hallValue = analogRead(hallPin);

  // Ohne Magnet liegt der Wert ungefähr in der Mitte.
  // LED leuchtet bei deutlicher Abweichung.
  Serial.print(minValue);
  Serial.print("mV, ");
  Serial.print(maxValue);
  Serial.print("mV, ");
  Serial.println(hallValue);

  if (minValue > hallValue) {
    minValue = hallValue;
  }

  if (maxValue < hallValue) {
    maxValue = hallValue;
  }

  if (hallValue < 530 || hallValue > 550) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }

  delay(10);
}
