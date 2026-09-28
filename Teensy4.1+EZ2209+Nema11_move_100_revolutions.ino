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
#define MOTOR_MA    800      // Motor max. 800 mA
#define MICROSTEPS  4
#define STEPS_PER_REV (200 * MICROSTEPS)

// Richtung
#define DIR_CCW LOW
#define DIR_CW  HIGH
#define HOME_DIR_CW  true    // Drehrichtung Richtung Nullpunkt

// Bewegung
#define ROTATIONS    100.0   // Umdrehungen pro Fahrt
#define START_RPM    200
#define MAX_RPM      1400
#define ACCEL_RPM_S  900
#define PAUSE_MS     3000   // 30 s Pause

TMC2209Stepper *driver = nullptr;
IntervalTimer stepTimer;
volatile uint32_t stepCount = 0;
volatile uint32_t targetSteps = 0;
volatile bool stepHigh = false;
bool timerRunning = false;

// ---------- Schrittgenerator (Hardware-Timer) ----------
void stepISR() {
  if (!stepHigh && stepCount >= targetSteps) return;
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
  if (!timerRunning) { stepTimer.begin(stepISR, halfPeriodUs); timerRunning = true; }
  else               { stepTimer.update(halfPeriodUs); }
}

// true = Richtung MAX, false = Richtung 0
void setDirection(bool towardMax) {
  bool cw = towardMax ? !HOME_DIR_CW : HOME_DIR_CW;
  digitalWriteFast(DIR_PIN, cw ? DIR_CW : DIR_CCW);
  delayMicroseconds(10);
}

// Fahrt mit Beschleunigungs- und Bremsrampe
void runRevolutions(float revs, bool towardMax) {
  setDirection(towardMax);
  uint32_t target = (uint32_t)(revs * STEPS_PER_REV + 0.5f);
  noInterrupts(); stepCount = 0; targetSteps = target; interrupts();

  float rpm = START_RPM;
  const float delta = ACCEL_RPM_S / 1000.0f;
  setRPM(rpm);

  while (stepCount < target) {
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
}

// ---------- Treiber ----------
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

  while (driver->version() != 0x21) {   // auf 24 V warten
    driver->begin();
    Serial.println("Warte auf VM (24 V)...");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(500);
  }
  configureDriver();
  digitalWrite(LED_PIN, LOW);
  digitalWrite(EN_PIN, LOW);            // Treiber an
  Serial.println("Start");
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  runRevolutions(ROTATIONS, true);
  digitalWrite(LED_PIN, LOW);
  Serial.println("Richtung MAX fertig - 30 s Pause");
  delay(PAUSE_MS);

  digitalWrite(LED_PIN, HIGH);
  runRevolutions(ROTATIONS, false);
  digitalWrite(LED_PIN, LOW);
  Serial.println("Richtung 0 fertig - 30 s Pause");
  delay(PAUSE_MS);
}
