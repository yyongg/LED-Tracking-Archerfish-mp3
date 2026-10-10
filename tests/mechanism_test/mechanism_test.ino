#include <Servo.h>
#include <Adafruit_MotorShield.h>

// per-sensor calibration (OFF_*, GAIN_*) and MIN_BRIGHTNESS, calibrated = (raw - OFF) * GAIN
// generated from the latest calibration run by calibration/apply.py, do not edit by hand
#include "calibration_values.h"

// Mechanism test sketch: test each subsystem on its own from the Serial Monitor.
// Open the Serial Monitor at 115200 baud with line ending "Newline", then type a
// command and press Enter. Type ? for the command list. Any input stops a running test.
//
// Suggested order on new hardware:
//   1. h, then p/t/a/d/w/x   - servos move, home points straight ahead and level
//   2. s                     - full sweep: no binding, wires have slack, tilt limits clear the mount
//   3. r                     - live sensors: cover each sensor and check its column drops
//   4. k                     - tracking only (no laser, no motor): head turns TOWARD a handheld LED
//   5. b                     - servo stress: if "=== BOOT ===" reappears, the Arduino browned out
//   6. n                     - wheel ramp: note the lowest speed where the wheel turns reliably
//   7. l                     - laser pulse
//
// Pins, limits and tracking math match tracking_mechanism.ino. If you change them
// there, change them here too.

// constants
const int PAN_HOME = 90;                    // points straight ahead
const int TILT_HOME = 90;                   // points level
const int PAN_MIN = 0, PAN_MAX = 180;
// tilt angles in this code: higher = up, lower = down (90 = level)
const bool PAN_REVERSED = false;            // true = pan servo turns the opposite way, so write 180 - pan to it
const bool TILT_REVERSED = true;            // servo is mounted flipped, so write 180 - tilt to it
const int TILT_MIN = 80, TILT_MAX = 150;    // TILT_MIN = farthest down, TILT_MAX = farthest up; keep the sensor from hitting the mount
const int SETTLE_MS = 60;                   // delay after each servo step
const int MAX_DEGREE_JUMP = 5;              // maximum number of degrees the servos can jump at once
const float TRACK_GAIN = 30;                // degrees of servo step per unit of x_diff / y_diff, copy from tracking_mechanism.ino
const float FOUND_THRESHOLD = 0.02;         // error below which an axis stops moving, copy from tracking_mechanism.ino

// initialize sensor pins (2x2 square, as seen from behind the head looking at the LED)
const int tl_sensor = A0;   // top left
const int tr_sensor = A1;   // top right
const int bl_sensor = A2;   // bottom left
const int br_sensor = A3;   // bottom right

// servos and laser pin numbers
const int pan_pin = 9;
const int tilt_pin = 10;
const int laser_pin = 8;

// initialize servos
Servo panServo, tiltServo;

// initialize motor shield and bug wheel motor (port M1)
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *bugMotor = AFMS.getMotor(1);
bool shieldFound = false;

// initialize current pan and tilt positions
int curPan = PAN_HOME, curTilt = TILT_HOME;

// send a tilt angle (higher = up) to the servo, undoing the flipped mount
void writeTilt(int tilt) {
  tiltServo.write(TILT_REVERSED ? 180 - tilt : tilt);
}

// move pan and tilt servos, then wait for them to stop moving
void moveTo(int pan, int tilt) {
  curPan = constrain(pan, PAN_MIN, PAN_MAX);
  curTilt = constrain(tilt, TILT_MIN, TILT_MAX);
  panServo.write(PAN_REVERSED ? 180 - curPan : curPan);
  writeTilt(curTilt);
  delay(SETTLE_MS);
}

void printPosition() {
  Serial.print("pan ");
  Serial.print(curPan);
  Serial.print("  tilt ");
  Serial.println(curTilt);
}

// true if the user typed anything, which stops a running test
bool stopRequested() {
  if (!Serial.available()) return false;
  while (Serial.available()) Serial.read();
  Serial.println("stopped");
  return true;
}

// apply calibration to a raw reading, never below 0
float calibrate(int raw, float off, float gain) {
  return max(0.0, (raw - off) * gain);
}

// read all sensors, print them, and return the normalized differences
// returns false if too dim to track
bool readSensors(float &x_diff, float &y_diff) {
  int tl = analogRead(tl_sensor);
  int tr = analogRead(tr_sensor);
  int bl = analogRead(bl_sensor);
  int br = analogRead(br_sensor);

  float c_tl = calibrate(tl, OFF_TL, GAIN_TL);
  float c_tr = calibrate(tr, OFF_TR, GAIN_TR);
  float c_bl = calibrate(bl, OFF_BL, GAIN_BL);
  float c_br = calibrate(br, OFF_BR, GAIN_BR);
  float total = c_tl + c_tr + c_bl + c_br;

  bool bright = total >= MIN_BRIGHTNESS;
  x_diff = bright ? ((c_tl + c_bl) - (c_tr + c_br)) / total : 0;
  y_diff = bright ? ((c_tl + c_tr) - (c_bl + c_br)) / total : 0;

  Serial.print(millis());   Serial.print(',');
  Serial.print(tl);         Serial.print(',');
  Serial.print(tr);         Serial.print(',');
  Serial.print(bl);         Serial.print(',');
  Serial.print(br);         Serial.print(',');
  Serial.print(x_diff, 4);  Serial.print(',');
  Serial.print(y_diff, 4);  Serial.print(',');
  Serial.print(curPan);     Serial.print(',');
  Serial.print(curTilt);    Serial.print(',');
  Serial.println(bright);
  return bright;
}

// sweep both servos across their full range and back
void sweepTest() {
  Serial.println("sweeping pan, then tilt (any key stops)");
  moveTo(PAN_HOME, TILT_HOME);
  for (int a = PAN_HOME; a >= PAN_MIN; a--) { moveTo(a, curTilt); if (stopRequested()) return; }
  for (int a = PAN_MIN; a <= PAN_MAX; a++) { moveTo(a, curTilt); if (stopRequested()) return; }
  for (int a = PAN_MAX; a >= PAN_HOME; a--) { moveTo(a, curTilt); if (stopRequested()) return; }
  for (int a = TILT_HOME; a >= TILT_MIN; a--) { moveTo(curPan, a); if (stopRequested()) return; }
  for (int a = TILT_MIN; a <= TILT_MAX; a++) { moveTo(curPan, a); if (stopRequested()) return; }
  for (int a = TILT_MAX; a >= TILT_HOME; a--) { moveTo(curPan, a); if (stopRequested()) return; }
  Serial.println("sweep done");
}

// stream sensor readings until stopped
void sensorTest() {
  Serial.println("ms,tl,tr,bl,br,x_diff,y_diff,pan,tilt,bright (any key stops)");
  float x, y;
  while (!stopRequested()) {
    readSensors(x, y);
    delay(100);
  }
}

// servo step in degrees for one axis: proportional to the error, at least 1 degree
// when the error is outside FOUND_THRESHOLD (so small errors don't round to 0), at most MAX_DEGREE_JUMP
int trackStep(float diff) {
  if (fabs(diff) < FOUND_THRESHOLD) return 0;
  int step = round(diff * TRACK_GAIN);
  if (step == 0) step = (diff > 0) ? 1 : -1;
  return constrain(step, -MAX_DEGREE_JUMP, MAX_DEGREE_JUMP);
}

// run the tracking loop with no laser and no motor
void trackTest() {
  Serial.println("tracking, no laser or motor (any key stops)");
  Serial.println("ms,tl,tr,bl,br,x_diff,y_diff,pan,tilt,bright");
  float x, y;
  while (!stopRequested()) {
    if (readSensors(x, y)) {
      // y > 0 means the LED is above, so tilt up (higher tilt)
      moveTo(curPan - trackStep(x), curTilt + trackStep(y));
    }
    else {
      delay(SETTLE_MS);
    }
  }
}

// slam both servos between extremes to check for brownouts
void stressTest() {
  Serial.println("servo stress, 20 cycles. If === BOOT === appears, the Arduino reset (brownout).");
  for (int i = 1; i <= 20; i++) {
    panServo.write(PAN_MIN + 20);
    writeTilt(TILT_MIN);
    delay(400);
    panServo.write(PAN_MAX - 20);
    writeTilt(TILT_MAX);
    delay(400);
    Serial.print("cycle ");
    Serial.println(i);
    if (stopRequested()) break;
  }
  moveTo(PAN_HOME, TILT_HOME);
  Serial.println("stress done, no reset");
}

// run the wheel motor at one speed for a few seconds
void motorRun(int speed) {
  if (!shieldFound) { Serial.println("motor shield not found"); return; }
  speed = constrain(speed, 0, 255);
  Serial.print("motor at ");
  Serial.print(speed);
  Serial.println(" for 3 s (any key stops)");
  bugMotor->setSpeed(speed);
  bugMotor->run(FORWARD);
  unsigned long start = millis();
  while (millis() - start < 3000) {
    if (stopRequested()) break;
  }
  bugMotor->run(RELEASE);
}

// ramp the wheel motor up to find the lowest speed that turns it
void motorRamp() {
  if (!shieldFound) { Serial.println("motor shield not found"); return; }
  Serial.println("ramping motor, 2 s per step. Note the first speed that turns the wheel reliably.");
  bugMotor->run(FORWARD);
  for (int speed = 20; speed <= 255; speed += 10) {
    bugMotor->setSpeed(speed);
    Serial.print("speed ");
    Serial.println(speed);
    unsigned long start = millis();
    while (millis() - start < 2000) {
      if (stopRequested()) { bugMotor->run(RELEASE); return; }
    }
  }
  bugMotor->run(RELEASE);
  Serial.println("ramp done");
}

void laserTest() {
  Serial.println("laser on for 1 s");
  digitalWrite(laser_pin, HIGH);
  delay(1000);
  digitalWrite(laser_pin, LOW);
}

void printHelp() {
  Serial.println();
  Serial.println("commands:");
  Serial.println("  h          home (pan 90, tilt 90)");
  Serial.println("  p<deg>     set pan, e.g. p120");
  Serial.println("  t<deg>     set tilt, e.g. t60");
  Serial.println("  a / d      pan -1 / +1 degree");
  Serial.println("  w / x      tilt up / down 1 degree");
  Serial.println("  s          sweep full servo range");
  Serial.println("  r          stream sensor readings");
  Serial.println("  k          track a handheld LED (no laser, no motor)");
  Serial.println("  b          servo stress / brownout test");
  Serial.println("  m<speed>   run wheel motor at 0-255 for 3 s, e.g. m80");
  Serial.println("  n          ramp wheel motor to find minimum speed");
  Serial.println("  l          laser on for 1 s");
  Serial.println("  ?          this list");
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  Serial.println();
  Serial.println("=== BOOT ===");

  // initialize pin modes
  pinMode(tl_sensor, INPUT);
  pinMode(tr_sensor, INPUT);
  pinMode(bl_sensor, INPUT);
  pinMode(br_sensor, INPUT);
  pinMode(laser_pin, OUTPUT);
  digitalWrite(laser_pin, LOW);

  // initialize motor shield, but keep going without it so servo tests still work
  shieldFound = AFMS.begin();
  if (shieldFound) bugMotor->run(RELEASE);
  else Serial.println("motor shield not found, motor tests disabled");

  // initialize servos
  panServo.attach(pan_pin);
  tiltServo.attach(tilt_pin);
  moveTo(PAN_HOME, TILT_HOME);

  printHelp();
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);
  int value = line.substring(1).toInt();

  switch (cmd) {
    case 'h': moveTo(PAN_HOME, TILT_HOME); printPosition(); break;
    case 'p': moveTo(value, curTilt); printPosition(); break;
    case 't': moveTo(curPan, value); printPosition(); break;
    case 'a': moveTo(curPan - 1, curTilt); printPosition(); break;
    case 'd': moveTo(curPan + 1, curTilt); printPosition(); break;
    case 'w': moveTo(curPan, curTilt + 1); printPosition(); break;   // up
    case 'x': moveTo(curPan, curTilt - 1); printPosition(); break;   // down
    case 's': sweepTest(); break;
    case 'r': sensorTest(); break;
    case 'k': trackTest(); break;
    case 'b': stressTest(); break;
    case 'm': motorRun(value); break;
    case 'n': motorRamp(); break;
    case 'l': laserTest(); break;
    case '?': printHelp(); break;
    default: Serial.println("unknown command, type ? for help");
  }
}
