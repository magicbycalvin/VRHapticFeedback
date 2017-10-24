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

#define DEBUG
#ifdef DEBUG
  #define DBprint( x ) Serial.print( x )
  #define DBprintln( x ) Serial.println( x )
#else
  #define DBprint( x )
  #define DBprintln( x )
#endif

// Motor direction, 0 or 1
#define MOTOR_DIR 1

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

// Low Pass Butterworth Filter
#define AX2 2.08056713549229E-3
#define AX1 4.16113427098458E-3
#define AX0 2.08056713549229E-3
#define BY1 -1.86689227971171
#define BY0 875.214548253684E-3

// PID controller constants (determined through simulation)
#define KP 25 // 5023.5276 // 20
#define KI 50 // 666884.475 // 0
#define KD 0 // 2.5993 // 0

// Motor driver
#define PWM_MAX 255 // Maximum PWM value
//#define PWM_MIN -255 // Minimum PWM value, min voltage = (PWM_MIN/255)*Driving voltage
#define PWM_MIN 0

// Communication codes
#define KILL_SWITCH 31337 // Stop powering the motor
#define SYNC 31338 // Reset the tick count

// Number of analog readings to take for determining the current bias
#define I_BIAS_SAMPLES 1000

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

// Filter
double xv[3] = {0};
double yv[2] = {0};

// Removing input bias from current sense
int currentSum = 0;
double currentBias = 0;

// PID
double setPoint, input, output;
PID myPID( &input, &output, &setPoint, KP, KI, KD, P_ON_E, DIRECT );

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
  myPID.SetOutputLimits(PWM_MIN, PWM_MAX);
  myPID.SetMode(MANUAL);

  // Remove any bias present in the current reading
  for (int i = 1; i < I_BIAS_SAMPLES; i++) {
    currentSum += analogRead( CS_ADC );
  }
  currentBias = (double) currentSum / I_BIAS_SAMPLES;

}

void loop() {

  // Read the input (either in DEBUG or normal mode)
  #ifdef DEBUG
    while (Serial.available() > 0) {
      setPoint = (double) Serial.parseInt();
    }
  #else
    if (Serial.available() > 1) {
      serialBuffer = Serial.read() << 8;
      serialBuffer |= Serial.read();
      setPoint = (double) serialBuffer;
      // Clear the buffer
      while (Serial.available() > 0) Serial.read();
    }
  #endif

  // Determine whether to drive the motor
  if (setPoint == KILL_SWITCH) {
    myPID.SetMode(MANUAL);
    output = 0;
  } else if (setPoint == SYNC) {
    myPID.SetMode(MANUAL);
    output = 0;
    encoderTicks = 0;
  } else {
    myPID.SetMode(AUTOMATIC);
    if (setPoint < 0) {
      setPoint = -setPoint;
      digitalWrite(PH_PIN, 1);
    } else {
      digitalWrite(PH_PIN, 0);
    }
  }

  // Filter and compute at a consistent frequency
  curTime = millis();
  if (curTime - lastTime >= TS_ms) {
    input = analogRead(CS_ADC) - currentBias;
    lowPassFilter( &input, xv, yv );
    myPID.Compute();
    analogWrite(PWM_PIN, output);
    lastTime = curTime;
    
    #ifdef DEBUG
      //Serial.print(setPoint);
      //Serial.print('\t');
      Serial.print(input);
      Serial.print('\t');
      Serial.println(output);
      //Serial.print('\t');
      //Serial.println(encoderTicks);
    #endif
  }

  // Send the tick count if it has changed
  if (changedTick) {
    #ifndef DEBUG
      serialBuffer = encoderTicks;
      Serial.write((char*) serialBuffer, BUFFER_LEN);
    #endif
    changedTick = 0;
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

void lowPassFilter( double* data, double xv[3], double yv[3] ) {
  xv[2] = xv[1];
  xv[1] = xv[0];

  xv[0] = *data;
  //yv[2] = yv[1];
  yv[1] = yv[0];

  yv[0] = (AX2 * xv[0] + AX1 * xv[1] + AX0 * xv[2]
             - BY1 * yv[0]
             - BY0 * yv[1]);

  *data = yv[0];
}

