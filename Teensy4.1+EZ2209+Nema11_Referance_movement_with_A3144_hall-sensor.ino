// created by Sergej Kolmogorov, Instagram: https://www.instagram.com/sergej.kolmogorov/
// YouTube: https://www.youtube.com/@SergejKolmogorov
// Code is free to use after subscribing my channel =)

#include <TMCStepper.h>

// Pins
#define STEP_PIN 2
#define DIR_PIN  3
#define EN_PIN   4
#define LED_PIN  40
#define HALL_PIN 39          // A3144, schaltet auf LOW bei Magnet

// Treiber
#define R_SENSE     0.11f
#define DRIVER_ADDR 0
#define MOTOR_MA    800
#define MICROSTEPS  4

// Mechanik
#define REV_PER_MM    1.0    // gemessen: 100 Umdrehungen = 100 mm
#define STEPS_PER_REV (200 * MICROSTEPS)
#define STEPS_PER_MM  (STEPS_PER_REV * REV_PER_MM)

// Richtung
#define DIR_CCW LOW
#define DIR_CW  HIGH
#define HOME_DIR_CW  true    // Drehrichtung, die zum Sensor (Nullpunkt) fuehrt

// Referenzfahrt
#define HOME_DELAY_US  1250  // ca. 60 U/min
#define SAFETY_MM      1.0   // Nullpunkt liegt so weit hinter dem Schaltpunkt
#define HYST_CLEAR_MM  15.0  // Weg zum sicheren Verlassen des Sensorbereichs
#define HOME_MAX_MM    170.0 // Abbruch, falls der Sensor nie ausloest
#define FREE_MAX_MM    30.0  // max. Weg zum Freifahren vom Sensor

TMC2209Stepper *driver = nullptr;
int32_t posSteps = 0;        // Position ab Nullpunkt, in Schritten
int8_t  dirSign = 1;

// ---------- Umrechnung ----------
uint32_t mmToSteps(float mm)  { return (uint32_t)(mm * STEPS_PER_MM + 0.5f); }
float    stepsToMM(int32_t s) { return (float)s / STEPS_PER_MM; }

// ---------- Hallsensor ----------
bool hallTriggered() {
  for (int i = 0; i < 3; i++) {
    if (digitalRead(HALL_PIN) != LOW) return false;
    delayMicroseconds(300);
  }
  return true;
}

// ---------- Richtung ----------
// true = Richtung MAX (weg vom Sensor), false = Richtung Sensor
void setDirection(bool towardMax) {
  bool cw = towardMax ? !HOME_DIR_CW : HOME_DIR_CW;
  digitalWriteFast(DIR_PIN, cw ? DIR_CW : DIR_CCW);
  dirSign = towardMax ? 1 : -1;
  delay(5);
}

// ---------- Einzelschritt ----------
void stepOnce() {
  digitalWriteFast(STEP_PIN, HIGH);
  delayMicroseconds(5);
  digitalWriteFast(STEP_PIN, LOW);
  delayMicroseconds(HOME_DELAY_US);
  posSteps += dirSign;
}

void errorStop(const char *msg) {
  digitalWrite(EN_PIN, HIGH);          // Treiber aus
  Serial.println(msg);
  while (true) {                       // LED blinkt schnell
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(HALL_PIN, INPUT_PULLUP);
  digitalWrite(EN_PIN, HIGH);

  Serial.begin(115200);
  while (!Serial && millis() < 5000) {}

  Serial1.begin(115200);
  delay(500);

  static TMC2209Stepper drv(&Serial1, R_SENSE, DRIVER_ADDR);
  driver = &drv;

  // auf 24 V warten
  while (driver->version() != 0x21) {
    driver->begin();
    Serial.println("Warte auf VM (24 V)...");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }

  driver->I_scale_analog(false);
  driver->toff(4);
  driver->blank_time(24);
  driver->rms_current(MOTOR_MA, 0.3);
  driver->iholddelay(6);
  driver->TPOWERDOWN(10);
  driver->microsteps(MICROSTEPS);
  driver->intpol(true);
  driver->en_spreadCycle(true);
  driver->GSTAT(0x07);
  delay(100);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(EN_PIN, LOW);           // Treiber an
  Serial.println("Referenzfahrt startet...");

  // --- Schritt 1: Schaltpunkt anfahren ---
  if (hallTriggered()) {
    Serial.println("Sensor aktiv - fahre erst frei");
    setDirection(true);
    uint32_t n = 0;
    while (hallTriggered()) {
      if (++n > mmToSteps(FREE_MAX_MM)) errorStop("FEHLER: Sensor bleibt aktiv");
      stepOnce();
    }
  }
  setDirection(false);
  uint32_t n = 0;
  while (!hallTriggered()) {
    if (++n > mmToSteps(HOME_MAX_MM)) errorStop("FEHLER: Sensor nicht gefunden");
    stepOnce();
  }
  Serial.println("Schaltpunkt erreicht");

  // --- Schritt 2: Hysterese verlassen, am Stueck ---
  setDirection(true);
  uint32_t away = mmToSteps(SAFETY_MM + HYST_CLEAR_MM);
  for (uint32_t i = 0; i < away; i++) stepOnce();

  if (hallTriggered()) errorStop("FEHLER: Sensor noch aktiv - HYST_CLEAR_MM erhoehen");

  // --- Schritt 3: Position setzen und auf 0 zurueck ---
  posSteps = mmToSteps(HYST_CLEAR_MM);
  setDirection(false);
  for (uint32_t i = 0; i < mmToSteps(HYST_CLEAR_MM); i++) stepOnce();

  Serial.print("Fertig. Position: ");
  Serial.print(stepsToMM(posSteps), 3);
  Serial.println(" mm (Nullpunkt)");
  Serial.print("Sensor jetzt: ");
  Serial.println(hallTriggered() ? "AKTIV (SAFETY_MM erhoehen)" : "frei");

  digitalWrite(LED_PIN, HIGH);         // LED an = fertig, Schlitten steht
}

void loop() {
  // nichts, der Schlitten bleibt stehen und haelt die Position
}
