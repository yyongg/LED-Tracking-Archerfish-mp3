#include <Servo.h>

// constants
const int PAN_HOME = 90;                    // points straight ahead
const int TILT_HOME = 90;                   // points level
const int SETTLE_MS = 60;                   // delay after a 1 degree step
const int BIG_MOVE_MS = 500;                // delay after a big jump
const int MAX_DEGREE_JUMP = 5;              // maximum number of degrees the servos can jump at once
const float FOUND_THRESHOLD = 0.02;         // Error thereshold for marking LED as found

// initialize sensor pins
const int x_l_sensor = A0;
const int x_r_sensor = A1;
const int y_d_sensor = A2;
const int y_u_sensor = A3;

// servos and laser pin numbers
const int pan_pin = 9;
const int tilt_pin = 10;
const int laser_pin = 8;
const int motor_pin = 7;

// boolean for if LED is found (tracking successful)
bool LED_found = LOW;

// initialize servos
Servo panServo, tiltServo;

// initialize current pan and tilt positions
int curPan = PAN_HOME, curTilt = TILT_HOME;

// initialize structure for differential sensing values
struct Differences {
  float x_diff;
  float y_diff;
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

void bugMove(unsigned long duration) {
  // turn on motor attached to bug for duration of time
  digitalWrite(motor_pin, HIGH);
  delay(duration);
  digitalWrite(motor_pin, LOW);
}

Differences diffSenseNorm(int left, int right, int down, int up) {
  // calculate normalized difference between sensor readings(a-b)/(a+b)
  // x_diff > 0 : move right, - angle
  // y_diff > 0: move down, - angle

  Differences diffs;
  diffs.x_diff = static_cast<float>(left-right) / (right+left);
  diffs.y_diff = static_cast<float>(up-down) / (up+down);
  return diffs;
}

void setup() {
  // initialize pin modes
  pinMode(x_l_sensor, INPUT);
  pinMode(x_r_sensor, INPUT);
  pinMode(y_d_sensor, INPUT);
  pinMode(y_u_sensor, INPUT);
  pinMode(laser_pin, OUTPUT);
  pinMode(motor_pin, OUTPUT);
  
  // initialize servos
  panServo.attach(9);
  tiltServo.attach(10);
  moveTo(PAN_HOME, TILT_HOME);
  delay(BIG_MOVE_MS);

  // initialize sensor values
  int x_l, x_r, y_d, y_u;
  x_l = x_r = y_d = y_u = 0;
}

void loop() {
  x_l = analogRead(x_l_sensor);
  x_r = analogRead(x_r_sensor);
  y_d = analogRead(y_d_sensor);
  y_u = analogRead(y_u_sensor);

  diffs = diffSenseNorm(x_l,x_r,y_d,y_u);

  // check if servos pointing at LED, otherwise move a small step towards the LED
  if (sqrt(pow(diffs.x_diff,2) + pow(diffs.y_diff,2)) < FOUND_THRESHOLD) {
    LED_found = true;
  }
  else {
    moveTo(curPan - diffs.x_diff * MAX_DEGREE_JUMP, curTilt - diffs.y_diff * MAX_DEGREE_JUMP);
  }

  // if found
  if (LED_found == true) {
    digitalWrite(laser_pin, HIGH); // activate laser diode
    delay(1000);
    digitalWrite(laser_pin, LOW);
    bugMove(random(1,2001)); // turn on motor for random time between 1 ms - 2000 ms
    LED_found = false;
  }
}
