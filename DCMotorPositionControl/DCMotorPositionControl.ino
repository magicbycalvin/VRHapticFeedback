/*
 * DEBUG STATEMENTS
 * 
 * Uncomment DEBUG in order to see debug statements printed to serial.
 */
 //#define DEBUG
 #ifdef DEBUG
  #define DBprint(x) Serial.print(x)
  #define DBprintln(x) Serial.println(x)
 #else
  #define DBprint(x)
  #define DBprintln(x)
 #endif

/*
 * INCLUDES
 */

/*
 * DEFINITIONS
 */

// Motor direction (0 or 1), if the motor is spinning CW when it should spin
// CCW, just change this 0 to a 1
#define MOTOR_DIR 0

// Mechanical specifications
#define GEAR_RATIO 4.995 // 4.995:1 (geared down)
#define CPR 12 // Counts per revolution of the quadrature encoder
#define PPR CPR/4 // for quad encoder reading only rising edge on one channel

// Motor encoder channel pins A and B (note these are the Arduino digital pin
// numbers)
#define CHANNEL_PIN_A 3 // Also INT0, SCL
#define CHANNEL_PIN_B 2 // Also INT1, SDA

// Motor driver pins (Arduino digital pin numbers)
#define PH_PIN 7
#define PWM_PIN 5

// Timing constants
#define ENC_SAMPLE_PER 1000 // Encoder sample period in us

// Baud rate for serial communication
#define SERIAL_BAUD 115200

// PID controller constants
#define KP 60
#define KI 0
#define KD 20
#define INTEGRAL_MAX 16000

// Motor driver
#define PWM_MAX 255 // Maximum PWM value
#define PWM_MIN 60 // Minimum PWM value, min voltage = (PWM_MIN/255)*Driving voltage

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

// Timing
unsigned long curTime = 0;
unsigned long lastTime = 0;

// PID
double integral = 0;
double lastError = 0;
double motorPWM = 0;
double setPoint = KILL_SWITCH;

/*
 * SETUP
 * 
 * Initializes the motor driver, quadrature encoder, and PID controller. If
 * DEBUG is set to 1, it will also begin serial communication.
 */

void setup() {

  Serial.begin(SERIAL_BAUD);

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

}

void loop() {
  
  curTime = micros();
  if (curTime-lastTime >= ENC_SAMPLE_PER) {

    #ifdef DEBUG
      while (Serial.available() > 0) {
        setPoint = Serial.parseInt();
        if (setPoint == 1337) while (1);
        if (Serial.read() == '\n') break;
      }
    #else 
      if (changedTick) {
        Serial.print(encoderTicks);
        changedTick = 0;
      }
      while (Serial.available() > 0) {
        setPoint = Serial.parseInt();
        Serial.read(); // Flush the buffer
      }
    #endif
  
    motorPWM = PID( setPoint, encoderTicks, integral, lastError );
    if (setPoint == KILL_SWITCH) {
      motorPWM = 0;
    } else if (setPoint == SYNC) {
      encoderTicks = 0;
      setPoint = KILL_SWITCH;
      motorPWM = 0;
    }
  
    driveDCMotor(motorPWM);
  
    DBprint(",\tSP: ");
    DBprint(setPoint);
    DBprint(",\tEncTick: ");
    DBprintln(encoderTicks);

    lastTime = curTime;

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

double PID( const double setPoint, const long input, double &integral, double &lastError) {

  double error = setPoint - input;

  /* Integral tends to overflow, we will make sure it doesn't here
  if (integral > INTEGRAL_MAX) {
    integral = INTEGRAL_MAX;
  } else if (integral < -INTEGRAL_MAX) {
    integral = -INTEGRAL_MAX;
  }*/

  double proportional = KP*error;
  integral += KI*error;
  double derivative = KD*(error - lastError);

  lastError = error;

  DBprint("P: ");
  DBprint(proportional);
  DBprint(",\t\tI: ");
  DBprint(integral);
  DBprint(",\t\tD: ");
  DBprint(derivative);
  DBprint(",\t\tErr: ");
  DBprint(error);

  return (proportional + integral + derivative);
  
}

void driveDCMotor( double input ) {
  
  int motorDir = 0;
  
  if (input < 0) {
    motorDir = 1;
    input = -input;
  }

  // Don't go over the maximum
  if (input > PWM_MAX) {
    input = PWM_MAX;
  // Don't go under the minimum
  } /* else if (input < PWM_MIN) {
    input = 0;
  }*/

  DBprint(",\t\tDir: ");
  DBprint(motorDir);
  DBprint(",\tPWM: ");
  DBprint(input);

  digitalWrite(PH_PIN, motorDir);
  analogWrite(PWM_PIN, (int)input);

  return;
  
}

