/* -----------------------------------------------------------------------------
 * DCMotorPIDTorqueControl.ino
 * -----------------------------------------------------------------------------
 * Firmware for the Arduino that controls the DC motor torque of a haptic 
 * feed back system meant for VR applications. The controller used is a standard
 * PID controller using derivative on measurement to limit the derivative kick.
 */

#include <PID_v1.h>
#include <stdint.h>

/*
 * DEFINITIONS
 */

// Motor direction, 0 or 1
#define MOTOR_DIR 0

// Mechanical specifications
#define GEAR_RATIO 51.45 // 51.45:1 (geared down)
#define CPR 12 // Counts per revolution of the quadrature encoder
#define PPR CPR/4 // for quad encoder reading only rising edge on one channel

// Motor encoder channel pins A and B (note these are the Arduino digital pin
// numbers)
#define CHANNEL_PIN_A 3 // Also INT0, SCL
#define CHANNEL_PIN_B 2 // Also INT1, SDA

// Motor driver pins (Arduino digital pin numbers)
#define PH_PIN 12
#define PWM_PIN 11

// Timing constants
#define TS_ms 1 // Encoder sample period in ms

// Communication
#define SERIAL_BAUD 115200
#define BUFFER_LEN 2

// Sensing
#define CS_ADC A0

// PID controller constants (determined through simulation)
#define KP 5023.5276
#define KI 666884.475
#define KD 2.5993

// Motor driver
#define PWM_MAX 255 // Maximum PWM value
#define PWM_MIN 13 // Minimum PWM value, min voltage = (PWM_MIN/255)*Driving voltage

// Communication codes
#define KILL_SWITCH 31337 // Stop powering the motor
#define SYNC 31338 // Reset the tick count

/*
 * VARIABLES
 */

// Volatile variables for the ISR
volatile int portState = 0;
volatile long encoderTicks = 0;
volatile bool changedTick = 0;

// Communication
int16_t serialBuffer;

// Timing
unsigned long curTime = 0;
unsigned long lastTime = 0;

// PID
double setPoint, input, output;
PID myPID( &input, &output, &setPoint, KP, KI, KD, DIRECT );

void setup() {

  Serial.begin( SERIAL_BAUD );

  // Motor driver
  // Set the pwm and phase pins as outputs
  pinMode(PH_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  // Start both pins at 0 just in case
  analogWrite(PH_PIN, 0);
  analogWrite(PWM_PIN, 0);

  // Quadrature encoder
  // Set pins as inputs
  pinMode(CHANNEL_PIN_A, INPUT);
  pinMode(CHANNEL_PIN_B, INPUT);
  // Enable internal pull up resistors
  digitalWrite(CHANNEL_PIN_A, LOW);
  digitalWrite(CHANNEL_PIN_B, LOW);
  // Attach an interrupt to channel A
  attachInterrupt(digitalPinToInterrupt(CHANNEL_PIN_A), 
    encoderInterruptA, CHANGE);
  // Attach an interrupt to channel B
  attachInterrupt(digitalPinToInterrupt(CHANNEL_PIN_B),
    encoderInterruptB, CHANGE);

  // PID Controller
  setPoint = KILL_SWITCH;
  myPID.SetSampleTime(TS_ms);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetMode(MANUAL);

}

void loop() {

  // Determine whether to drive the motor
  if (setPoint == KILL_SWITCH) {
    myPID.SetMode(MANUAL);
  } else if (setPoint == SYNC) {
    myPID.SetMode(MANUAL);
    encoderTicks = 0;
  } else {
    myPID.SetMode(AUTOMATIC);
  }
  
  // Send the tick count if it has changed
  if (changedTick) {
    serialBuffer = encoderTicks;
    Serial.write((char*) serialBuffer, BUFFER_LEN);
    changedTick = 0;
  }

  // Read the current sense value, in the future this will be filtered.
  input = analogRead(CS_ADC);

  myPID.compute();
  
  // Drive the motor in the correct direction
  if (output < 0) {
    analogWrite(PH_PIN, 1);
    analogWrite(PWM_PIN, -output);
  } else {
    analogWrite(PH_PIN, 0);
    analogWrite(PWM_PIN, output);
  }

}

void encoderInterruptA() {
  portState = PIND & B11;
  switch ( portState ) {
    case B00:
    case B11:
      #if MOTOR_DIR
        encoderTicks += 1;
      #else
        encoderTicks -= 1;
      #endif
      break;
    case B01:
    case B10:
      #if MOTOR_DIR
        encoderTicks -= 1;
      #else
        encoderTicks += 1;
      #endif
      break;
  }
  changedTick = 1;
}

void encoderInterruptB() {
  portState = PIND & B11;
  switch ( portState ) {
    case B00:
    case B11:
      #if MOTOR_DIR
        encoderTicks -= 1;
      #else
        encoderTicks += 1;
      #endif
      break;
    case B01:
    case B10:
      #if MOTOR_DIR
        encoderTicks += 1;
      #else
        encoderTicks -= 1;
      #endif
      break;
  }
  changedTick = 1;
}
