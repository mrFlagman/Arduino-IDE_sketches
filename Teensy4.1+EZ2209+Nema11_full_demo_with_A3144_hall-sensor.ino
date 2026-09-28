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
#define MOTOR_MA    800      // Motor max. 800 mA
#define MICROSTEPS  4

// Mechanik
#define REV_PER_MM   1.0     // gemessen: 100 Umdrehungen = 100 mm
#define STEPS_PER_REV (200 * MICROSTEPS)
#define STEPS_PER_MM  (STEPS_PER_REV * REV_PER_MM)

// Richtung
#define DIR_CCW LOW
#define DIR_CW  HIGH
#define HOME_DIR_CW  true    // Drehrichtung, die zum Nullpunkt fuehrt

// Referenzfahrt
#define HOME_DELAY_US  1250  // ca. 60 U/min
#define SAFETY_MM      1.0   // Sicherheitsabstand an beiden Enden
#define HYST_CLEAR_MM  15.0  // Weg zum sicheren Verlassen des Sensorbereichs
#define HOME_MAX_MM    170.0 // Abbruch, falls der Sensor nie ausloest
#define FREE_MAX_MM    30.0  // max. Weg zum Freifahren vom Sensor

// Fahrweg
#define TRAVEL_TOTAL_MM 138.0                          // ab Nullpunkt bis zum Anschlag
#define MAX_POS_MM      (TRAVEL_TOTAL_MM - SAFETY_MM)  // nutzbare Obergrenze = 137 mm

// Bewegung
#define START_RPM    200
#define MAX_RPM      1400
#define ACCEL_RPM_S  900
#define PAUSE_MS     500
#define VM_CHECK_MS  200     // VM-Ueberwachung nur bei Hoechstdrehzahl

TMC2209Stepper *driver = nullptr;
IntervalTimer stepTimer;
volatile uint32_t stepCount = 0;
volatile uint32_t targetSteps = 0;
volatile int32_t  posSteps = 0;     // Position ab Nullpunkt, in Schritten
volatile int8_t   dirSign = 1;
volatile bool stepHigh = false;
bool timerRunning = false;
bool homed = false;

// ---------- Umrechnung ----------
uint32_t mmToSteps(float mm)  { return (uint32_t)(mm * STEPS_PER_MM + 0.5f); }
float    stepsToMM(int32_t s) { return (float)s / STEPS_PER_MM; }
float    getPositionMM()      { return stepsToMM(posSteps); }

// ---------- Hallsensor (robust entprellt) ----------
bool hallTriggered() {
  for (int i = 0; i < 3; i++) {
    if (digitalRead(HALL_PIN) != LOW) return false;
    delayMicroseconds(300);
  }
  return true;
}

// ---------- Schrittgenerator (Hardware-Timer) ----------
void stepISR() {
  if (!stepHigh && stepCount >= targetSteps) return;
  stepHigh = !stepHigh;
  digitalWriteFast(STEP_PIN, stepHigh);
  if (stepHigh) { stepCount++; posSteps += dirSign; }
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
  if (!timerRunning) { stepTimer.begin(stepISR, halfPeriodUs); timerRunning = true; }
  else               { stepTimer.update(halfPeriodUs); }
}

void stopNow() {
  noInterrupts();
  targetSteps = 0;
  stepCount = 1;
  interrupts();
  setRPM(0);
}

// ---------- Richtung ----------
// true  = Richtung groessere Position (weg vom Nullpunkt)
// false = Richtung Nullpunkt (Homing-Richtung)
void setDirection(bool towardMax) {
  bool cw = towardMax ? !HOME_DIR_CW : HOME_DIR_CW;
  digitalWriteFast(DIR_PIN, cw ? DIR_CW : DIR_CCW);
  dirSign = towardMax ? 1 : -1;
  delayMicroseconds(10);
}

// ---------- VM-Ueberwachung ----------
bool vmOk() { return driver->version() == 0x21; }

void emergencyStop(const char *msg) {
  stopNow();
  digitalWrite(EN_PIN, HIGH);          // Treiber aus
  homed = false;                       // erzwingt neue Referenzfahrt
  Serial.println(msg);
}

// ---------- Einzelschritt (langsam, fuer die Referenzfahrt) ----------
void stepOnce(uint32_t pauseUs) {
  digitalWriteFast(STEP_PIN, HIGH);
  delayMicroseconds(5);
  digitalWriteFast(STEP_PIN, LOW);
  delayMicroseconds(pauseUs);
  posSteps += dirSign;
}

bool runMM(float mm, bool towardMax);   // vorwaerts deklariert

// ---------- Referenzfahrt ----------
bool homing() {
  Serial.println("Referenzfahrt startet...");

  if (hallTriggered()) {
    // Sensor bereits aktiv -> weg vom Nullpunkt freifahren
    Serial.println("Sensor aktiv - fahre frei");
    setDirection(true);
    delay(5);
    uint32_t n = 0;
    while (hallTriggered()) {
      if (++n > mmToSteps(FREE_MAX_MM)) { Serial.println("FEHLER: Sensor bleibt aktiv"); return false; }
      stepOnce(HOME_DELAY_US);
    }
    // zurueck, damit der Schaltpunkt sauber angefahren wird
    setDirection(false);
    delay(5);
    n = 0;
    while (!hallTriggered()) {
      if (++n > mmToSteps(FREE_MAX_MM + 5.0)) { Serial.println("FEHLER: Sensor nicht gefunden"); return false; }
      stepOnce(HOME_DELAY_US);
    }
  } else {
    // Sensor frei -> in Homing-Richtung fahren, bis er ausloest
    setDirection(false);
    delay(5);
    uint32_t n = 0;
    while (!hallTriggered()) {
      if (++n > mmToSteps(HOME_MAX_MM)) { Serial.println("FEHLER: Sensor nicht gefunden"); return false; }
      stepOnce(HOME_DELAY_US);
    }
  }
  Serial.println("Schaltpunkt erreicht");

  // Am Stueck SAFETY_MM + HYST_CLEAR_MM Richtung MAX, damit der Sensor loslaesst
  uint32_t away = mmToSteps(SAFETY_MM + HYST_CLEAR_MM);
  setDirection(true);
  delay(5);
  for (uint32_t i = 0; i < away; i++) stepOnce(HOME_DELAY_US);

  if (hallTriggered()) {
    Serial.println("FEHLER: Sensor noch aktiv - HYST_CLEAR_MM erhoehen");
    return false;
  }

  // Der Schlitten steht jetzt HYST_CLEAR_MM ueber dem Nullpunkt
  noInterrupts(); posSteps = mmToSteps(HYST_CLEAR_MM); interrupts();
  homed = true;

  if (!vmOk()) { emergencyStop("VM ausgefallen"); return false; }

  // Zurueck auf Position 0, ohne den Sensor zu beruehren
  if (!runMM(HYST_CLEAR_MM, false)) return false;

  Serial.print("Nullpunkt gesetzt, Position: ");
  Serial.print(getPositionMM(), 3); Serial.print(" mm | nutzbar bis ");
  Serial.print(MAX_POS_MM, 1); Serial.println(" mm");
  return true;
}

// ---------- Fahrten ----------
bool runMM(float mm, bool towardMax) {
  setDirection(towardMax);
  uint32_t target = mmToSteps(mm);
  if (target == 0) return true;
  noInterrupts(); stepCount = 0; targetSteps = target; interrupts();

  float rpm = START_RPM;
  const float delta = ACCEL_RPM_S / 1000.0f;
  uint32_t lastCheck = millis();
  setRPM(rpm);

  while (stepCount < target) {
    if (hallTriggered()) {                     // Notstopp Endschalter
      emergencyStop("NOTSTOPP: Endschalter im Betrieb ausgeloest");
      return false;
    }
    if (rpm >= MAX_RPM && millis() - lastCheck >= VM_CHECK_MS) {
      lastCheck = millis();
      if (!vmOk()) { emergencyStop("NOTSTOPP: 24 V ausgefallen"); return false; }
    }

    uint32_t remaining = target - stepCount;
    float brakeSteps = (rpm * rpm - (float)START_RPM * START_RPM)
                       / (120.0f * ACCEL_RPM_S) * STEPS_PER_REV;

    if (remaining <= brakeSteps) rpm -= delta;
    else if (rpm < MAX_RPM)      rpm += delta;
    rpm = constrain(rpm, (float)START_RPM, (float)MAX_RPM);

    setRPM(rpm);
    delay(1);
  }
  delayMicroseconds(1000);
  setRPM(0);
  return true;
}

// ABSOLUT: faehrt auf die Position targetMM (0 ... MAX_POS_MM)
bool moveToMM(float targetMM) {
  if (!homed) { Serial.println("Nicht referenziert"); return false; }
  if (targetMM < -0.01 || targetMM > MAX_POS_MM + 0.01) {
    Serial.print("Ziel ausserhalb: "); Serial.print(targetMM, 2);
    Serial.print(" mm (erlaubt 0 ... "); Serial.print(MAX_POS_MM, 1); Serial.println(" mm)");
    return false;
  }
  float diff = targetMM - getPositionMM();
  if (fabs(diff) < 0.001) return true;
  return runMM(fabs(diff), diff > 0);
}

// RELATIV: faehrt um deltaMM weiter (positiv = vom Nullpunkt weg)
bool moveRelativeMM(float deltaMM) {
  if (!homed) { Serial.println("Nicht referenziert"); return false; }
  float endMM = getPositionMM() + deltaMM;
  if (endMM < -0.01 || endMM > MAX_POS_MM + 0.01) {
    Serial.print("Relative Fahrt abgelehnt, Ziel waere ");
    Serial.print(endMM, 2); Serial.println(" mm");
    return false;
  }
  if (fabs(deltaMM) < 0.001) return true;
  return runMM(fabs(deltaMM), deltaMM > 0);
}

// ---------- Treiber ----------
void waitForDriver() {
  Serial.println("Warte auf VM (24 V)...");
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
  driver->intpol(true);
  driver->en_spreadCycle(true);
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
  if (!vmOk()) {
    Serial.println("Keine Antwort - VM ausgefallen?");
    initDriver();
    homed = false;
    return;
  }
  if (driver->GSTAT() & 0x01) {
    Serial.println("Treiber-Reset erkannt, konfiguriere neu");
    initDriver();
    homed = false;
  }
}

// ---------- Ausgabe ----------
void report(const char *what) {
  Serial.print(what);
  Serial.print(" | Position: ");        Serial.print(getPositionMM(), 3);
  Serial.print(" mm | Temp-Warnung: "); Serial.print(driver->otpw() ? "JA" : "nein");
  Serial.print(" | Kurzschluss: ");     Serial.println((driver->s2ga() || driver->s2gb()) ? "JA" : "nein");
}

// ---------- Programm ----------
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

  initDriver();
}

void loop() {
  checkDriver();                         // wartet notfalls auf 24 V

  if (!homed) {
    digitalWrite(EN_PIN, LOW);
    digitalWrite(LED_PIN, HIGH);
    bool ok = homing();
    digitalWrite(LED_PIN, LOW);
    if (!ok) { delay(PAUSE_MS); return; }
  }

  digitalWrite(LED_PIN, HIGH);

  // Pendeln ueber die volle nutzbare Strecke: 0 -> 137 mm -> 0
  if (!moveToMM(MAX_POS_MM)) { digitalWrite(LED_PIN, LOW); return; }
  report("nach Max");
  delay(PAUSE_MS);

  if (!moveToMM(0.0)) { digitalWrite(LED_PIN, LOW); return; }
  report("nach 0");

  digitalWrite(LED_PIN, LOW);
  delay(PAUSE_MS);
}
