#include <Servo.h>

// Calibration sketch: holds the head at home and streams raw sensor readings.
// Uses only the sensors and servos. The laser and bug wheel motor are not used;
// calibrate with a handheld LED.
// Upload this, then run calibrate.py on the computer.
// Pins must match tracking_mechanism.ino.

// constants
const int PAN_HOME = 90;                    // points straight ahead
const int TILT_HOME = 90;                   // points level (same servo angle whether or not the tilt servo is flipped)
const int SAMPLE_MS = 20;                   // time between samples
const bool USE_SERVOS = false;               // false = never send servo pulses for noise 

// initialize sensor pins (2x2 square, as seen from behind the head looking at the LED)
const int tl_sensor = A0;   // top left
const int tr_sensor = A1;   // top right
const int bl_sensor = A2;   // bottom left
const int br_sensor = A3;   // bottom right

// servo pin numbers
const int pan_pin = 9;
const int tilt_pin = 10;

// initialize servos
Servo panServo, tiltServo;

void setup() {
  Serial.begin(115200);

  // initialize pin modes
  pinMode(tl_sensor, INPUT);
  pinMode(tr_sensor, INPUT);
  pinMode(bl_sensor, INPUT);
  pinMode(br_sensor, INPUT);

  // hold the head straight ahead
  if (USE_SERVOS) {
    panServo.attach(pan_pin);
    tiltServo.attach(tilt_pin);
    panServo.write(PAN_HOME);
    tiltServo.write(TILT_HOME);
    delay(500);
  }

  // CSV header for serial logging
  Serial.println("ms,tl,tr,bl,br");
}

void loop() {
  // log raw readings
  Serial.print(millis());                 Serial.print(',');
  Serial.print(analogRead(tl_sensor));    Serial.print(',');
  Serial.print(analogRead(tr_sensor));    Serial.print(',');
  Serial.print(analogRead(bl_sensor));    Serial.print(',');
  Serial.println(analogRead(br_sensor));

  delay(SAMPLE_MS);
}
