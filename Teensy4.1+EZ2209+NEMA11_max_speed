// created by Sergej Kolmogorov, Instagram: https://www.instagram.com/sergej.kolmogorov/
// YouTube: https://www.youtube.com/@SergejKolmogorov
// Code is free to use after subscribing my channel =)

#include <TMCStepper.h>

// Pins
#define STEP_PIN 2
#define DIR_PIN  3
#define EN_PIN   4
#define LED_PIN  40

// Treiber
#define R_SENSE     0.11f
#define DRIVER_ADDR 0
#define MOTOR_MA    800     // Motor max. 800 mA
#define MICROSTEPS  4

// Bewegung
#define STEPS_PER_REV (200 * MICROSTEPS)
#define ROTATIONS    80.0   // zu fahrende Umdrehungen (auch Kommazahlen, z.B. 2.5)
#define START_RPM    200     // Startdrehzahl ohne Rampe
#define MAX_RPM      1400  // 12 V ca. 700, 24 V ca. 1200-1500
#define ACCEL_RPM_S  900 // Beschleunigung in U/min pro Sekunde
#define PAUSE_MS     500    // Pause vor Richtungswechsel

TMC2209Stepper *driver = nullptr;
IntervalTimer stepTimer;
volatile uint32_t stepCount = 0;
volatile uint32_t targetSteps = 0;
volatile bool stepHigh = false;
bool timerRunning = false;
bool dir = false;

// ---------- Schrittgenerator (Hardware-Timer) ----------
void stepISR() {
  if (!stepHigh && stepCount >= targetSteps) return;  // Ziel erreicht, keine Impulse mehr
  stepHigh = !stepHigh;
  digitalWriteFast(STEP_PIN, stepHigh);
  if (stepHigh) stepCount++;
}

void setRPM(float rpm) {
  if (rpm <= 0) {
    stepTimer.end();
    timerRunning = false;
    stepHigh = false;
    digitalWriteFast(STEP_PIN, LOW);
    return;
  }
  float stepsPerSec = rpm * STEPS_PER_REV / 60.0f;
  float halfPeriodUs = 500000.0f / stepsPerSec;
  if (!timerRunning) {
    stepTimer.begin(stepISR, halfPeriodUs);
    timerRunning = true;
  } else {
    stepTimer.update(halfPeriodUs);
  }
}

// Fährt eine bestimmte Anzahl Umdrehungen mit Rampe
void moveRevolutions(float revs) {
  uint32_t target = (uint32_t)(revs * STEPS_PER_REV + 0.5f);
  noInterrupts();
  stepCount = 0;
  targetSteps = target;
  interrupts();

  float rpm = START_RPM;
  const float delta = ACCEL_RPM_S / 1000.0f;   // Änderung pro ms
  bool printed = false;    
  setRPM(rpm);

  while (stepCount < target) {
    uint32_t remaining = target - stepCount;
    // Schritte, die zum Abbremsen auf START_RPM nötig sind
    float brakeSteps = (rpm * rpm - (float)START_RPM * START_RPM)
                       / (120.0f * ACCEL_RPM_S) * STEPS_PER_REV;

    if (remaining <= brakeSteps) rpm -= delta;       // bremsen
    else if (rpm < MAX_RPM)      rpm += delta;       // beschleunigen
    rpm = constrain(rpm, (float)START_RPM, (float)MAX_RPM);

    setRPM(rpm);

    // NEU: einmal bei Höchstdrehzahl den Strom prüfen
    if (!printed && rpm >= MAX_RPM) {
      delay(200);                                    // kurz einschwingen lassen
      Serial.print("Bei "); Serial.print(MAX_RPM); Serial.print(" U/min -> ");
      Serial.print("Sollwert IRUN: ");      Serial.print(driver->irun());
      Serial.print(" | Istwert CS_ACTUAL: "); Serial.println(driver->cs_actual());
      printed = true;
    }

    delay(1);
  }
  delayMicroseconds(1000);                           // letzten Impuls sauber beenden
  setRPM(0);
}

// ---------- Treiber ----------
void waitForDriver() {
  Serial.println("Warte auf VM...");
  while (true) {
    driver->begin();
    if (driver->version() == 0x21) return;
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }
}

void configureDriver() {
  driver->I_scale_analog(false);
  driver->toff(4);
  driver->blank_time(24);
  driver->rms_current(MOTOR_MA, 0.3);   // Haltestrom 30 %
  driver->iholddelay(6);
  driver->TPOWERDOWN(10);
  driver->microsteps(MICROSTEPS);
  driver->intpol(true);                 // Interpolation auf 1/256
  driver->en_spreadCycle(true);         // SpreadCycle für hohe Drehzahl
  driver->GSTAT(0x07);
  delay(100);

  Serial.print("Strom: ");         Serial.print(driver->rms_current()); Serial.println(" mA");
  Serial.print("Mikroschritte: "); Serial.println(driver->microsteps());
}

void initDriver() {
  setRPM(0);
  digitalWrite(EN_PIN, HIGH);
  waitForDriver();
  configureDriver();
  Serial.println("UART OK - Treiber an");
  digitalWrite(LED_PIN, LOW);
  digitalWrite(EN_PIN, LOW);
}

void checkDriver() {
  if (driver->version() != 0x21) {
    Serial.println("Keine Antwort - VM ausgefallen?");
    initDriver();
    return;
  }
  if (driver->GSTAT() & 0x01) {
    Serial.println("Treiber-Reset erkannt, konfiguriere neu");
    initDriver();
  }
}

// ---------- Programm ----------
void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);

  Serial.begin(115200);
  while (!Serial && millis() < 5000) {}

  Serial1.begin(115200);
  delay(500);

  static TMC2209Stepper drv(&Serial1, R_SENSE, DRIVER_ADDR);
  driver = &drv;

  initDriver();
}

void loop() {
  checkDriver();

  digitalWriteFast(DIR_PIN, dir);
  delayMicroseconds(10);
  digitalWrite(LED_PIN, HIGH);

  uint32_t t0 = millis();
  moveRevolutions(ROTATIONS);
  uint32_t dt = millis() - t0;

  digitalWrite(LED_PIN, LOW);

  Serial.print("Richtung: ");       Serial.print(dir ? "rechts" : "links");
  Serial.print(" | Umdrehungen: "); Serial.print((float)stepCount / STEPS_PER_REV, 2);
  Serial.print(" | Dauer: ");       Serial.print(dt); Serial.print(" ms");
  Serial.print(" | Temp-Warnung: "); Serial.print(driver->otpw() ? "JA" : "nein");
  Serial.print(" | Kurzschluss: ");  Serial.println((driver->s2ga() || driver->s2gb()) ? "JA" : "nein");

  dir = !dir;
  delay(PAUSE_MS);
}
