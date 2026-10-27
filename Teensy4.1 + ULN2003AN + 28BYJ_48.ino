// created by Sergej Kolmogorov, Instagram: https://www.instagram.com/sergej.kolmogorov/
// YouTube: https://www.youtube.com/@SergejKolmogorov
// Code is free to use after subscribing my channel =)

#include <AccelStepper.h>

// Verdrahtung Teensy 4.1 → ULN2003-Board
const int PIN_IN1 = 2;   // Teensy Pin 2 → IN1
const int PIN_IN2 = 3;   // Teensy Pin 3 → IN2
const int PIN_IN3 = 4;   // Teensy Pin 4 → IN3
const int PIN_IN4 = 5;   // Teensy Pin 5 → IN4
// Teensy GND → "−" am ULN2003-Board

// Wichtig: AccelStepper erwartet die Reihenfolge IN1, IN3, IN2, IN4
AccelStepper motor(AccelStepper::HALF4WIRE, PIN_IN1, PIN_IN3, PIN_IN2, PIN_IN4);

void setup() {
  motor.setMaxSpeed(800);       // Halbschritte/s
  motor.setAcceleration(400);
  motor.moveTo(4076);           // 1 Abtriebsumdrehung
}

void loop() {
  if (motor.distanceToGo() == 0) {
    motor.moveTo(-motor.currentPosition());  // hin und zurück
  }
  motor.run();
}
