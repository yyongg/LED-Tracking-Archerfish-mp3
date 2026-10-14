#include <Servo.h>
#include <Adafruit_MotorShield.h>

// per-sensor calibration (OFF_*, GAIN_*) and MIN_BRIGHTNESS, calibrated = (raw - OFF) * GAIN
// generated from the latest calibration run by calibration/apply.py, do not edit by hand
#include "calibration_values.h"

// constants
const int PAN_HOME = 90;                    // points straight ahead
const int TILT_HOME = 90;                   // points level
// tilt angles in this code: higher = up, lower = down (90 = level)
const bool PAN_REVERSED = true;            // true = pan servo turns the opposite way, so write 180 - pan to it
const bool TILT_REVERSED = true;            // servo is mounted flipped, so write 180 - tilt to it
const int TILT_MIN = 30, TILT_MAX = 150;    // TILT_MIN = farthest down, TILT_MAX = farthest up; keep the sensor from hitting the mount
const int SETTLE_MS = 60;                  // delay after each servo step
const int BIG_MOVE_MS = 500;                // delay after a big jump (homing)
const int MAX_DEGREE_JUMP = 5;              // maximum number of degrees the servos can jump at once
const float TRACK_GAIN = 30;                // degrees of servo step per unit of x_diff / y_diff (tune: too low = sluggish, too high = overshoot)
const float FOUND_THRESHOLD = 0.02;         // Error threshold for marking LED as found (placeholder, set from characterization)
const int BUG_MOTOR_SPEED = 170;            // wheel motor speed (0-255)
const int LASER_TILT_OFFSET = 10;            // degrees to tilt up before firing, because the laser hits below the LED
                                            // (~2 deg corrects 1 cm at ~30 cm; raise it if still low, lower it if high)

// initialize sensor pins (2x2 square, as seen from behind the head looking at the LED)
const int tl_sensor = A0;   // top left
const int tr_sensor = A1;   // top right
const int bl_sensor = A2;   // bottom left
const int br_sensor = A3;   // bottom right

// servos and laser pin numbers
const int pan_pin = 9;
const int tilt_pin = 10;
const int laser_pin = 8;

// boolean for if LED is found (tracking successful)
bool LED_found = false;

// initialize servos
Servo panServo, tiltServo;

// initialize motor shield and bug wheel motor (port M1)
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *bugMotor = AFMS.getMotor(4);

// initialize current pan and tilt positions
int curPan = PAN_HOME, curTilt = TILT_HOME;

// initialize structure for differential sensing values
struct Differences {
  float x_diff;
  float y_diff;
};

// move pan and tilt servos, then wait for them to stop moving
void moveTo(int pan, int tilt) {
  // ensure pan and tilt values are within range
  pan = constrain(pan, 0, 180);
  tilt = constrain(tilt, TILT_MIN, TILT_MAX);

  // move to pan and tilt values
  panServo.write(PAN_REVERSED ? 180 - pan : pan);
  tiltServo.write(TILT_REVERSED ? 180 - tilt : tilt);

  // set current pan and tilt values to new values
  curPan = pan;
  curTilt = tilt;

  delay(SETTLE_MS);
}

// servo step in degrees for one axis: proportional to the error, at least 1 degree
// when the error is outside FOUND_THRESHOLD (so small errors don't round to 0), at most MAX_DEGREE_JUMP
int trackStep(float diff) {
  if (fabs(diff) < FOUND_THRESHOLD) return 0;
  int step = round(diff * TRACK_GAIN);
  if (step == 0) step = (diff > 0) ? 1 : -1;
  return constrain(step, -MAX_DEGREE_JUMP, MAX_DEGREE_JUMP);
}

void bugMove(unsigned long duration) {
  // turn on motor attached to bug for duration of time
  bugMotor->setSpeed(BUG_MOTOR_SPEED);
  bugMotor->run(FORWARD);
  delay(duration);
  bugMotor->run(RELEASE);
}

// apply calibration to a raw reading, never below 0
float calibrate(int raw, float off, float gain) {
  return max(0.0, (raw - off) * gain);
}

Differences diffSenseNorm(float tl, float tr, float bl, float br) {
  // calculate normalized difference for a 2x2 sensor square of calibrated readings
  // x_diff = (left column - right column) / total
  // y_diff = (top row - bottom row) / total
  // x_diff > 0: left column brighter, LED is to the left
  // y_diff > 0: top row brighter, LED is above
  // loop() adds y_diff to tilt (higher = up) and subtracts x_diff from pan;
  // if pan turns away from the LED, flip PAN_REVERSED
  // callers must check brightness first so the denominator is never 0

  float total = tl + tr + bl + br;
  Differences diffs;
  diffs.x_diff = ((tl + bl) - (tr + br)) / total;
  diffs.y_diff = ((tl + tr) - (bl + br)) / total;
  return diffs;
}

void setup() {
  Serial.begin(115200);

  // initialize pin modes
  pinMode(tl_sensor, INPUT);
  pinMode(tr_sensor, INPUT);
  pinMode(bl_sensor, INPUT);
  pinMode(br_sensor, INPUT);
  pinMode(laser_pin, OUTPUT);
  digitalWrite(laser_pin, LOW);

  // initialize motor shield
  if (!AFMS.begin()) {
    Serial.println("Motor shield not found");
    while (1);
  }
  bugMotor->run(RELEASE);

  // initialize servos
  panServo.attach(pan_pin);
  tiltServo.attach(tilt_pin);
  moveTo(PAN_HOME, TILT_HOME);
  delay(BIG_MOVE_MS);

  // CSV header for serial logging
  Serial.println("ms,tl,tr,bl,br,x_diff,y_diff,pan,tilt,bright,found");
}

void loop() {
  // read sensor values
  int tl = analogRead(tl_sensor);
  int tr = analogRead(tr_sensor);
  int bl = analogRead(bl_sensor);
  int br = analogRead(br_sensor);

  // calibrate so every sensor reads the same for the same light
  float c_tl = calibrate(tl, OFF_TL, GAIN_TL);
  float c_tr = calibrate(tr, OFF_TR, GAIN_TR);
  float c_bl = calibrate(bl, OFF_BL, GAIN_BL);
  float c_br = calibrate(br, OFF_BR, GAIN_BR);

  // only track if the sensors see enough light (also prevents divide by zero)
  bool bright = (c_tl + c_tr + c_bl + c_br >= MIN_BRIGHTNESS);

  Differences diffs = {0, 0};
  if (bright) {
    diffs = diffSenseNorm(c_tl, c_tr, c_bl, c_br);

    // check if servos pointing at LED (both axes inside FOUND_THRESHOLD, the same test trackStep
    // uses to stop moving), otherwise move a small step towards the LED
    if (trackStep(diffs.x_diff) == 0 && trackStep(diffs.y_diff) == 0) {
      LED_found = true;
    }
    else {
      // y_diff > 0 means the LED is above, so tilt up (higher tilt)
      int lastPan = curPan, lastTilt = curTilt;
      moveTo(curPan - trackStep(diffs.x_diff), curTilt + trackStep(diffs.y_diff));

      // if the servo limits stopped every axis that still needed to move, the LED is out of
      // range; this is as close as the head can get, so fire from here anyway
      if (curPan == lastPan && curTilt == lastTilt) {
        Serial.println("LED out of range, at servo limit, firing anyway");
        LED_found = true;
      }
    }
  }

  // log raw readings and tracking state
  Serial.print(millis());       Serial.print(',');
  Serial.print(tl);             Serial.print(',');
  Serial.print(tr);             Serial.print(',');
  Serial.print(bl);             Serial.print(',');
  Serial.print(br);             Serial.print(',');
  Serial.print(diffs.x_diff, 4); Serial.print(',');
  Serial.print(diffs.y_diff, 4); Serial.print(',');
  Serial.print(curPan);         Serial.print(',');
  Serial.print(curTilt);        Serial.print(',');
  Serial.print(bright);         Serial.print(',');
  Serial.println(LED_found);

  // if found
  if (LED_found == true) {
    // tilt up to correct the laser hitting below the LED, fire, then return to the tracked position
    int aimedTilt = curTilt;
    moveTo(curPan, curTilt + LASER_TILT_OFFSET);
    digitalWrite(laser_pin, HIGH); // activate laser diode
    delay(1000);
    digitalWrite(laser_pin, LOW);
    moveTo(curPan, aimedTilt);
    bugMove(random(1, 2001)); // turn on motor for random time between 1 ms - 2000 ms
    LED_found = false;
    waitForStart();
  }
}

// pause until the user sends anything over Serial, so each tracking cycle is started by hand
void waitForStart() {
  Serial.println("cycle done, press Enter in the Serial Monitor to start the next one");
  while (Serial.available()) Serial.read();   // drop anything typed during the cycle
  while (!Serial.available());                // wait for input
  delay(10);                                  // let the rest of the line arrive
  while (Serial.available()) Serial.read();
  Serial.println("ms,tl,tr,bl,br,x_diff,y_diff,pan,tilt,bright,found");
}
