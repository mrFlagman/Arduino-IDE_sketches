// created by Sergej Kolmogorov, Instagram: https://www.instagram.com/sergej.kolmogorov/
// YouTube: https://www.youtube.com/@SergejKolmogorov
// Code is free to use after subscribing my channel =)

#include <TMCStepper.h>

// ================= Geschwindigkeiten (U/min, 1 U = 1 mm) =================
#define HOME_RPM     60      // Referenzfahrt: Anfahrt und Freifahren (1 mm/s)
#define CREEP_RPM    30      // Kriechgang mit reduziertem Strom (0,5 mm/s)
#define START_RPM    200     // Start-/Stoppdrehzahl der Rampe
#define CRUISE_RPM   1400    // Reisegeschwindigkeit
#define ACCEL_RPM_S  900     // Beschleunigung / Verzoegerung in U/min pro s
// =========================================================================

// Pins
#define STEP_PIN 2
#define DIR_PIN  3
#define EN_PIN   4
#define LED_PIN  40
#define HALL_PIN 14          // 49E analog (A0) - Sensor mit 3,3 V versorgen!

// Treiber
#define R_SENSE      0.11f
#define DRIVER_ADDR  0
#define MOTOR_MA     800     // Motor max. 800 mA
#define HOME_SOFT_MA 200     // reduzierter Strom im Kriechgang -> rutscht am Anschlag durch
#define MICROSTEPS   4

// Mechanik
#define REV_PER_MM    1.0    // gemessen: 100 Umdrehungen = 100 mm
#define STEPS_PER_REV (200 * MICROSTEPS)
#define STEPS_PER_MM  (STEPS_PER_REV * REV_PER_MM)

// Schrittpause aus Drehzahl (fuer die langsamen Einzelschritte)
#define RPM_TO_DELAY_US(rpm) ((uint32_t)(60000000.0 / ((rpm) * STEPS_PER_REV)) - 5)
#define HOME_DELAY_US  RPM_TO_DELAY_US(HOME_RPM)
#define CREEP_DELAY_US RPM_TO_DELAY_US(CREEP_RPM)

// Richtung
#define DIR_CCW LOW
#define DIR_CW  HIGH
#define HOME_DIR_CW  true    // Drehrichtung, die zum Nullpunkt fuehrt

// Hallsensor 49E (10 bit)
#define HALL_IDLE_NOM    540   // Ruhewert ohne Magnet
#define HALL_DETECT_DEV  20    // Abweichung, ab der Kriechgang beginnt
#define HALL_PLATEAU_MIN 540   // Anschlag nur in diesem Wertebereich akzeptieren
#define HALL_PLATEAU_MAX 800
#define TICK_MM          0.5   // Messintervall im Kriechgang
#define PLATEAU_TOL      3     // max. Aenderung je Tick = "steht"
#define PLATEAU_TICKS    5     // so viele ruhige Ticks in Folge

// Referenzfahrt
#define SAFETY_MM      1.0   // Nullpunkt liegt 1 mm vor dem Anschlag
#define HYST_CLEAR_MM  15.0  // Zusatzweg nach dem Freifahren
#define HOME_MAX_MM    170.0 // Abbruch, falls kein Magnetfeld erkannt wird
#define FREE_MAX_MM    40.0  // max. Weg zum Freifahren
#define SOFT_MAX_MM    25.0  // max. Weg im Kriechgang

// Fahrweg
#define TRAVEL_TOTAL_MM 138.0                          // ab Nullpunkt bis zum Anschlag
#define MAX_POS_MM      (TRAVEL_TOTAL_MM - SAFETY_MM)  // nutzbare Obergrenze = 137 mm

// Sonstiges
#define PAUSE_MS     500
#define VM_CHECK_MS  200     // VM-Ueberwachung nur bei Reisegeschwindigkeit

TMC2209Stepper *driver = nullptr;
IntervalTimer stepTimer;
volatile uint32_t stepCount = 0;
volatile uint32_t targetSteps = 0;
volatile int32_t  posSteps = 0;     // Position ab Nullpunkt, in Schritten
volatile int8_t   dirSign = 1;
volatile bool stepHigh = false;
bool timerRunning = false;
bool homed = false;

int hallBaseline  = HALL_IDLE_NOM;  // Ruhewert ohne Magnet
int hallRefValue  = 0;              // Wert am mechanischen Anschlag
int hallZeroValue = 0;              // Wert am Nullpunkt
int hallLimitDev  = 0;              // Notstopp-Schwelle (0 = aus)

// ---------- Umrechnung ----------
uint32_t mmToSteps(float mm)  { return (uint32_t)(mm * STEPS_PER_MM + 0.5f); }
float    stepsToMM(int32_t s) { return (float)s / STEPS_PER_MM; }
float    getPositionMM()      { return stepsToMM(posSteps); }

// ---------- Hallsensor ----------
int readHall() { return analogRead(HALL_PIN); }   // Hardware-Mittelung aktiv

int readHallAvg(int n) {
  long sum = 0;
  for (int i = 0; i < n; i++) { sum += analogRead(HALL_PIN); delayMicroseconds(100); }
  return (int)(sum / n);
}

int hallDev(int v) { return abs(v - hallBaseline); }

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
  hallLimitDev = 0;
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

bool homingFail(const char *msg) {
  driver->rms_current(MOTOR_MA, 0.3);  // vollen Strom wiederherstellen
  Serial.println(msg);
  return false;
}

// ---------- Referenzfahrt ----------
bool homing() {
  Serial.println("Referenzfahrt startet...");
  hallLimitDev = 0;                     // Endzonen-Ueberwachung aus
  driver->rms_current(MOTOR_MA, 0.3);

  // 1) Steht der Schlitten bereits im Magnetfeld? -> Richtung MAX freifahren
  if (abs(readHallAvg(16) - HALL_IDLE_NOM) > HALL_DETECT_DEV) {
    Serial.println("Magnetfeld aktiv - fahre frei");
    setDirection(true);
    delay(5);
    uint32_t n = 0;
    while (abs(readHall() - HALL_IDLE_NOM) > HALL_DETECT_DEV) {
      if (++n > mmToSteps(FREE_MAX_MM)) return homingFail("FEHLER: Magnetfeld bleibt aktiv");
      stepOnce(HOME_DELAY_US);
    }
    uint32_t extra = mmToSteps(HYST_CLEAR_MM);
    for (uint32_t i = 0; i < extra; i++) stepOnce(HOME_DELAY_US);
  }

  // 2) Ruhewert ohne Magnet messen
  delay(20);
  hallBaseline = readHallAvg(64);
  Serial.print("Hall-Ruhewert: "); Serial.println(hallBaseline);

  // 3) Anfahrt mit vollem Strom, bis das Magnetfeld erkannt wird
  setDirection(false);
  delay(5);
  uint32_t n = 0;
  while (hallDev(readHall()) <= HALL_DETECT_DEV) {
    if (++n > mmToSteps(HOME_MAX_MM)) return homingFail("FEHLER: Magnetfeld nicht gefunden");
    stepOnce(HOME_DELAY_US);
  }
  Serial.println("Magnetfeld erkannt - Strom reduziert, Kriechgang");

  // 4) Kriechgang mit reduziertem Strom, bis sich der Sensorwert nicht mehr aendert
  driver->rms_current(HOME_SOFT_MA, 0.5);
  delay(20);

  const uint32_t tickSteps = mmToSteps(TICK_MM);
  uint32_t stepsInTick = 0;
  uint32_t stepsInRev  = 0;
  uint32_t rev = 0;
  int lastTickVal = readHallAvg(8);
  int lastRevVal  = lastTickVal;
  int calm = 0;
  n = 0;

  while (true) {
    if (++n > mmToSteps(SOFT_MAX_MM)) return homingFail("FEHLER: Kein Anschlag erkannt");
    stepOnce(CREEP_DELAY_US);

    // Ausgabe pro Umdrehung
    if (++stepsInRev >= STEPS_PER_REV) {
      stepsInRev = 0;
      rev++;
      int v = readHallAvg(8);
      Serial.print("Kriechgang U "); Serial.print(rev);
      Serial.print(" | Hall: ");     Serial.print(v);
      Serial.print(" | Aenderung/U: "); Serial.println(v - lastRevVal);
      lastRevVal = v;
    }

    // Plateau-Erkennung pro Tick
    if (++stepsInTick < tickSteps) continue;
    stepsInTick = 0;

    int v = readHallAvg(8);
    int change = abs(v - lastTickVal);
    lastTickVal = v;

    bool inRange = (v >= HALL_PLATEAU_MIN && v <= HALL_PLATEAU_MAX);
    if (inRange && change <= PLATEAU_TOL) {
      if (++calm >= PLATEAU_TICKS) break;
    } else {
      calm = 0;
    }
  }

  // 5) Anschlag erreicht -> Referenzwert speichern, vollen Strom zurueck
  hallRefValue = readHallAvg(64);
  driver->rms_current(MOTOR_MA, 0.3);
  delay(50);
  Serial.print("Anschlag erkannt, Hall-Referenz: "); Serial.println(hallRefValue);

  // Schlitten steht am Anschlag = -SAFETY_MM
  noInterrupts(); posSteps = -(int32_t)mmToSteps(SAFETY_MM); interrupts();
  homed = true;

  if (!vmOk()) { emergencyStop("VM ausgefallen"); return false; }

  // 6) 1 mm Richtung MAX auf den Nullpunkt
  if (!runMM(SAFETY_MM, true)) return false;
  delay(50);

  hallZeroValue = readHallAvg(64);
  int refDev  = hallDev(hallRefValue);
  int zeroDev = hallDev(hallZeroValue);
  hallLimitDev = zeroDev + (refDev - zeroDev) / 2;   // Notstopp bei ca. halbem Weg zum Anschlag

  if (refDev - zeroDev < 2 * PLATEAU_TOL)
    Serial.println("WARNUNG: geringer Signalunterschied - Endzonen-Ueberwachung unsicher");

  Serial.print("Nullpunkt gesetzt, Position: ");
  Serial.print(getPositionMM(), 3); Serial.print(" mm | Hall Null: ");
  Serial.print(hallZeroValue); Serial.print(" | Notstopp ab Abweichung: ");
  Serial.print(hallLimitDev); Serial.print(" | nutzbar bis ");
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
  uint8_t zoneHits = 0;
  setRPM(rpm);

  while (stepCount < target) {
    // Notstopp: Endzone vor dem Anschlag (nur bei Fahrt Richtung Null)
    if (!towardMax && hallLimitDev > 0) {
      if (hallDev(readHall()) >= hallLimitDev) {
        if (++zoneHits >= 3) {
          emergencyStop("NOTSTOPP: Endzone im Betrieb erreicht");
          return false;
        }
      } else {
        zoneHits = 0;
      }
    }
    if (rpm >= CRUISE_RPM && millis() - lastCheck >= VM_CHECK_MS) {
      lastCheck = millis();
      if (!vmOk()) { emergencyStop("NOTSTOPP: 24 V ausgefallen"); return false; }
    }

    uint32_t remaining = target - stepCount;
    float brakeSteps = (rpm * rpm - (float)START_RPM * START_RPM)
                       / (120.0f * ACCEL_RPM_S) * STEPS_PER_REV;

    if (remaining <= brakeSteps) rpm -= delta;
    else if (rpm < CRUISE_RPM)   rpm += delta;
    rpm = constrain(rpm, (float)START_RPM, (float)CRUISE_RPM);

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
    hallLimitDev = 0;
    return;
  }
  if (driver->GSTAT() & 0x01) {
    Serial.println("Treiber-Reset erkannt, konfiguriere neu");
    initDriver();
    homed = false;
    hallLimitDev = 0;
  }
}

// ---------- Ausgabe ----------
void report(const char *what) {
  Serial.print(what);
  Serial.print(" | Position: ");        Serial.print(getPositionMM(), 3);
  Serial.print(" mm | Hall: ");         Serial.print(readHallAvg(8));
  Serial.print(" | Temp-Warnung: ");    Serial.print(driver->otpw() ? "JA" : "nein");
  Serial.print(" | Kurzschluss: ");     Serial.println((driver->s2ga() || driver->s2gb()) ? "JA" : "nein");
}

// ---------- Programm ----------
void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);

  analogReadResolution(10);
  analogReadAveraging(16);

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
