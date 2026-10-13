/*
 * ================================================================
 *  STEER-BY-WIRE (SBW) + ADAPTIVE CRUISE CONTROL (ACC)
 *  Single Arduino Uno
 * ================================================================
 *  PIN ASSIGNMENTS
 * ----------------------------------------------------------------
 *  KY-040 (Input Encoder, 18°/step):
 *    CLK  → D2  (INT0 - hardware interrupt)
 *    DT   → D4
 *    VCC  → 5V | GND → GND
 *
 *  Feedback Encoder (12°/step):
 *    CLK  → D3  (INT1 - hardware interrupt)
 *    DT   → D7
 *    VCC  → 5V | GND → GND
 *
 *  LG 90-S Servo Motor:
 *    Signal → D9  (PWM)
 *    VCC    → Separate 5V/2A supply (NOT Arduino 5V pin)
 *    GND    → Common GND
 *
 *  HC-SR04 Ultrasonic:
 *    TRIG → D5
 *    ECHO → D6
 *    VCC  → 5V | GND → GND
 *
 *  L298N Motor Driver:
 *    ENA  → D10  (PWM - left motor speed)
 *    IN1  → A0   (left motor dir A)
 *    IN2  → A1   (left motor dir B)
 *    ENB  → D11  (PWM - right motor speed)
 *    IN3  → A2   (right motor dir A)
 *    IN4  → A3   (right motor dir B)
 *    Motor supply 9-12V → L298N +12V terminal
 *    GND  → Common GND
 *
 *  16x2 I2C LCD:
 *    SDA → A4
 *    SCL → A5
 *    VCC → 5V | GND → GND
 *    I2C address: 0x27 (change to 0x3F if LCD stays blank)
 * ================================================================
 *  LOOP TIMING
 *    SBW PID  : runs every loop (~10ms = 100Hz)
 *    ACC      : runs every 100ms (10Hz)
 *    LCD      : updates every 500ms
 * ================================================================
 */

#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ── Encoder Pins ─────────────────────────────────────────────
#define ENC_INPUT_CLK   2   // KY-040 CLK  → INT0
#define ENC_INPUT_DT    4   // KY-040 DT
#define ENC_FB_CLK      3   // Feedback CLK → INT1
#define ENC_FB_DT       7   // Feedback DT

// ── Servo ────────────────────────────────────────────────────
#define SERVO_PIN       9

// ── Ultrasonic ───────────────────────────────────────────────
#define TRIG_PIN        5
#define ECHO_PIN        6

// ── L298N Motor Driver ───────────────────────────────────────
#define ENA             10   // PWM left
#define IN1             A0
#define IN2             A1
#define ENB             11   // PWM right
#define IN3             A2
#define IN4             A3

// ── PID Tuning ───────────────────────────────────────────────
const float Kp = 2.5f;
const float Ki = 0.08f;
const float Kd = 0.35f;
const float INTEGRATOR_LIMIT = 50.0f;

// ── Servo Limits ─────────────────────────────────────────────
const int   SERVO_MIN    = 45;
const int   SERVO_MAX    = 135;
const int   SERVO_CENTRE = 90;
const float STEER_RANGE  = 45.0f;   // ±45° driver input max

// ── ACC Parameters ───────────────────────────────────────────
const int   CRUISE_PWM  = 180;   // 0-255
const int   SLOW_PWM    = 80;
const float SAFE_DIST   = 30.0f; // cm - start slowing
const float BRAKE_DIST  = 15.0f; // cm - full stop
const float MAX_DIST    = 400.0f;

// ── Globals ──────────────────────────────────────────────────
volatile long inputCount = 0;
volatile long fbCount    = 0;

float pidErrorPrev  = 0.0f;
float integrator    = 0.0f;

Servo steeringServo;
LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long tPrev     = 0;
unsigned long tACC      = 0;
unsigned long tLCD      = 0;

int   accPWM   = 0;
float accDist  = 0.0f;
char  accMode[3] = "CR";

// ================================================================
//  ISRs
// ================================================================
void isrInputCLK() {
  bool clk = digitalRead(ENC_INPUT_CLK);
  bool dt  = digitalRead(ENC_INPUT_DT);
  inputCount += (clk == dt) ? +1 : -1;
}

void isrFbCLK() {
  bool clk = digitalRead(ENC_FB_CLK);
  bool dt  = digitalRead(ENC_FB_DT);
  fbCount += (clk == dt) ? +1 : -1;
}

// ================================================================
//  Encoder → degrees
// ================================================================
inline float inputToDeg(long c) { return c * 18.0f; }  // 20PPR→18°/step
inline float fbToDeg(long c)    { return c * 12.0f; }  // 30PPR→12°/step

// ================================================================
//  PID
// ================================================================
float pidUpdate(float sp, float meas, float dt) {
  float err   = sp - meas;
  integrator += err * dt;
  integrator  = constrain(integrator, -INTEGRATOR_LIMIT, INTEGRATOR_LIMIT);
  float deriv = (err - pidErrorPrev) / dt;
  pidErrorPrev = err;
  return Kp * err + Ki * integrator + Kd * deriv;
}

// ================================================================
//  Motor helpers
// ================================================================
void driveForward(int spd) {
  analogWrite(ENA, spd); analogWrite(ENB, spd);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void brakeMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ================================================================
//  Ultrasonic
// ================================================================
float measureDistance() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 25000UL); // 25ms timeout ~4.3m
  if (dur == 0) return MAX_DIST;
  return constrain((dur * 0.0343f) / 2.0f, 0.0f, MAX_DIST);
}

// ================================================================
//  LCD update (called every 500ms to avoid flickering)
// ================================================================
void updateLCD(float dist, float steerDeg) {
  // Row 0: distance + ACC mode
  lcd.setCursor(0, 0);
  lcd.print("D:");
  lcd.print(dist, 0);
  lcd.print("cm ");
  lcd.print(accMode);
  lcd.print("       ");  // clear trailing chars

  // Row 1: steering angle
  lcd.setCursor(0, 1);
  lcd.print("Steer:");
  if (steerDeg >= 0) lcd.print(" ");
  lcd.print(steerDeg, 0);
  lcd.print((char)223);  // degree symbol
  lcd.print("       ");
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  // Encoder pins
  pinMode(ENC_INPUT_CLK, INPUT_PULLUP);
  pinMode(ENC_INPUT_DT,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_INPUT_CLK), isrInputCLK, CHANGE);

  pinMode(ENC_FB_CLK, INPUT_PULLUP);
  pinMode(ENC_FB_DT,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_FB_CLK), isrFbCLK, CHANGE);

  // Servo
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTRE);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Motor driver
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  brakeMotors();

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("SBW + ACC");
  lcd.setCursor(0, 1); lcd.print("Initialising...");
  delay(1500);
  lcd.clear();

  tPrev = micros();
  tACC  = millis();
  tLCD  = millis();
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  unsigned long now = millis();

  // ────────────────────────────────────────────────────────────
  //  SBW — runs every loop iteration (~100Hz)
  // ────────────────────────────────────────────────────────────
  unsigned long tNow = micros();
  float dt = (tNow - tPrev) * 1e-6f;
  tPrev = tNow;
  if (dt <= 0.0f) dt = 0.001f;

  // Atomic read
  noInterrupts();
  long iCnt = inputCount;
  long fCnt = fbCount;
  interrupts();

  float sp   = constrain(inputToDeg(iCnt), -STEER_RANGE, STEER_RANGE);
  float meas = fbToDeg(fCnt);

  float u   = pidUpdate(sp, meas, dt);
  int   cmd = (int)constrain(u + meas + SERVO_CENTRE,
                              (float)SERVO_MIN, (float)SERVO_MAX);
  steeringServo.write(cmd);

  // ────────────────────────────────────────────────────────────
  //  ACC — runs every 100ms (10Hz)
  //  NOTE: pulseIn() blocks for up to 25ms — acceptable at 10Hz
  // ────────────────────────────────────────────────────────────
  if (now - tACC >= 100) {
    tACC = now;

    accDist = measureDistance();

    if (accDist >= SAFE_DIST) {
      accPWM = CRUISE_PWM;
      strcpy(accMode, "CR");
      driveForward(accPWM);

    } else if (accDist >= BRAKE_DIST) {
      float ratio = (accDist - BRAKE_DIST) / (SAFE_DIST - BRAKE_DIST);
      accPWM = SLOW_PWM + (int)(ratio * (CRUISE_PWM - SLOW_PWM));
      strcpy(accMode, "SL");
      driveForward(accPWM);

    } else {
      accPWM = 0;
      strcpy(accMode, "BR");
      brakeMotors();
    }

    // Optional: reduce motor speed during sharp turns > 30°
    // if (abs(meas) > 30.0f && accPWM > 0) {
    //   driveForward((int)(accPWM * 0.8f));
    // }
  }

  // ────────────────────────────────────────────────────────────
  //  LCD — update every 500ms
  // ────────────────────────────────────────────────────────────
  if (now - tLCD >= 500) {
    tLCD = now;
    updateLCD(accDist, meas);
  }

  delay(10);  // 100Hz SBW loop
}
