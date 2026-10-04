#include <Servo.h>

const int PAN_HOME = 90;                    // points straight ahead
const int TILT_HOME = 90;                   // points level
const int SETTLE_MS = 60;                   // delay after a 1 degree step
const int BIG_MOVE_MS = 500;                // delay after a big jump
const int LUMINOSITY_PERCENT_DIFF = 0.03    // allowable percent difference of sensor readings before balanced

// initialize servos
Servo panServo, tiltServo;

// initialize current pan and tilt positions
int curPan = PAN_HOME, curTilt = TILT_HOME;

// initialize structure for differential sensing values
struct Differences {
  float x_diff
  float y_diff
}

// move pan and tilt servos, then wait for them to stop moving
void moveTo(int pan, int tilt) {
  // ensure pan and tilt values are within range
  pan = constrain(pan, 0, 180);
  tilt = constrain(tilt, 30, 150);  // keep the sensor from hitting the mount

  // check if a big move was made (5+ degrees)
  bool bigMove = abs(pan - curPan) > 5 || abs(tilt - curTilt) > 5;

  // move to pan and tilt values
  panServo.write(pan);
  tiltServo.write(tilt);

  // set current pan and tilt values to new values
  curPan = pan;
  curTilt = tilt;

  // add a bigger delay if a big move was made
  delay(bigMove ? BIG_MOVE_MS : SETTLE_MS);
}

Differences float diffSenseNorm(int left, int right, int up, int down) {
  // calculate normalized difference between sensor readings(a-b)/(a+b)
  float x_diff_norm = static_cast<float>(right-left) / (right+left)
  float y_diff_norm = static_cast<float>(up-down) / (up+down)

  return stru
}

void setup() {
  panServo.attach(9);
  tiltServo.attach(10);
  moveTo(PAN_HOME, TILT_HOME);
  delay(BIG_MOVE_MS);
}


void loop() {
}
