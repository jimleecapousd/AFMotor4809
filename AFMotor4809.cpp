/*
  Patched Adafruit Motor Shield (AFMotor) Library – ATmega4809 compatible
  -----------------------------------------------------------------------
  This file is a drop‑in replacement for the original AFMotor.cpp that ships
  with the Adafruit Motor Shield V1 library.  It retains all functionality
  on classic 8‑bit AVRs, ATmega1280/2560, and PIC32 while adding seamless
  support for the newer megaAVR‑0 family (ATmega4809) used on the Arduino
  Uno WiFi Rev 2 and Nano Every.

  Key idea:  The megaAVR‑0 timers are totally different, so we simply let the
  Arduino core handle PWM via analogWrite() instead of poking timer registers.
  That requires three small changes:
    1.  Detect ATmega4809 and define AFMOTOR_NO_DIRECT_PWM.
    2.  In every initPWMx()/setPWMx() pair, add a fast analogWrite() branch
        that is compiled when the guard is present.
    3.  Ensure no #error triggers on the new MCU.

  No changes were needed elsewhere – the higher‑level AF_DCMotor and
  AF_Stepper classes continue to work unmodified.
*/

#if (ARDUINO >= 100)
  #include "Arduino.h"
#else
  #if defined(__AVR__)
    #include <avr/io.h>
  #endif
  #include "WProgram.h"
#endif

#include "AFMotor4809.h"

/* -------------------------------------------------------------------------
 *  ATmega4809 compatibility shim
 * -------------------------------------------------------------------------*/
#if defined(__AVR_ATmega4809__)
  // Skip all legacy timer register hacks; we will rely purely on analogWrite()
  #define AFMOTOR_NO_DIRECT_PWM
#endif

static uint8_t latch_state;

#if (MICROSTEPS == 8)
uint8_t microstepcurve[] = {0, 50, 98, 142, 180, 212, 236, 250, 255};
#elif (MICROSTEPS == 16)
uint8_t microstepcurve[] = {0, 25, 50, 74, 98, 120, 141, 162, 180, 197, 212, 225, 236, 244, 250, 253, 255};
#endif

AFMotorController::AFMotorController(void) {
    TimerInitalized = false;
}

void AFMotorController::enable(void) {
  // Pin directions
  pinMode(MOTORLATCH,   OUTPUT);
  pinMode(MOTORENABLE,  OUTPUT);
  pinMode(MOTORDATA,    OUTPUT);
  pinMode(MOTORCLK,     OUTPUT);

  latch_state = 0;
  latch_tx();  // reset all outputs

  // Enable 74HC595 output
  digitalWrite(MOTORENABLE, LOW);
}

void AFMotorController::latch_tx(void) {
  uint8_t i;

  digitalWrite(MOTORLATCH, LOW);
  digitalWrite(MOTORDATA,  LOW);

  for (i = 0; i < 8; i++) {
    digitalWrite(MOTORCLK, LOW);
    digitalWrite(MOTORDATA, (latch_state & _BV(7 - i)) ? HIGH : LOW);
    digitalWrite(MOTORCLK, HIGH);
  }
  digitalWrite(MOTORLATCH, HIGH);
}

static AFMotorController MC;

/******************************************
               DC MOTORS
******************************************/

/* ---------------- PWM channel 1  (motor ENA, Arduino pin 11) -------------- */
inline void initPWM1(uint8_t freq) {
#if defined(AFMOTOR_NO_DIRECT_PWM)
    // ATmega4809: core handles PWM
    pinMode(11, OUTPUT);
    analogWrite(11, 0);
#elif defined(__AVR_ATmega8__)   || defined(__AVR_ATmega48__)  || \
      defined(__AVR_ATmega88__)  || defined(__AVR_ATmega168__) || \
      defined(__AVR_ATmega328P__)
    TCCR2A |= _BV(COM2A1) | _BV(WGM20) | _BV(WGM21);
    TCCR2B  = freq & 0x7;
    OCR2A   = 0;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    TCCR1A |= _BV(COM1A1) | _BV(WGM10);
    TCCR1B  = (freq & 0x7) | _BV(WGM12);
    OCR1A   = 0;
#elif defined(__PIC32MX__)
    /* PIC32 branches unchanged */
    #if defined(PIC32_USE_PIN9_FOR_M1_PWM)
        pinMode(9, OUTPUT);
        pinMode(11, INPUT);
        if (!MC.TimerInitalized) {
            T2CON = 0x8000 | ((freq & 0x07) << 4);
            TMR2 = 0x0000;
            PR2  = 0x0100;
            MC.TimerInitalized = true;
        }
        OC4CON = 0x8006;
        OC4RS  = 0;
        OC4R   = 0;
    #elif defined(PIC32_USE_PIN10_FOR_M1_PWM)
        pinMode(10, OUTPUT);
        pinMode(11, INPUT);
        if (!MC.TimerInitalized) {
            T2CON = 0x8000 | ((freq & 0x07) << 4);
            TMR2 = 0x0000;
            PR2  = 0x0100;
            MC.TimerInitalized = true;
        }
        OC5CON = 0x8006;
        OC5RS  = 0;
        OC5R   = 0;
    #else
        digitalWrite(11, LOW);
    #endif
#else
    #error "This chip is not supported!"
#endif
#if !defined(AFMOTOR_NO_DIRECT_PWM) && !defined(PIC32_USE_PIN9_FOR_M1_PWM) && !defined(PIC32_USE_PIN10_FOR_M1_PWM)
    pinMode(11, OUTPUT);
#endif
}

inline void setPWM1(uint8_t s) {
#if defined(AFMOTOR_NO_DIRECT_PWM)
    analogWrite(11, s);
#elif defined(__AVR_ATmega8__)   || defined(__AVR_ATmega48__)  || \
      defined(__AVR_ATmega88__)  || defined(__AVR_ATmega168__) || \
      defined(__AVR_ATmega328P__)
    OCR2A = s;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    OCR1A = s;
#elif defined(__PIC32MX__)
    #if defined(PIC32_USE_PIN9_FOR_M1_PWM)
        OC4RS = s;
    #elif defined(PIC32_USE_PIN10_FOR_M1_PWM)
        OC5RS = s;
    #else
        digitalWrite(11, (s > 127));
    #endif
#else
    #error "This chip is not supported!"
#endif
}

/* ---------------- PWM channel 2  (motor ENB, Arduino pin 3) --------------- */
inline void initPWM2(uint8_t freq) {
#if defined(AFMOTOR_NO_DIRECT_PWM)
    pinMode(3, OUTPUT);
    analogWrite(3, 0);
#elif defined(__AVR_ATmega8__)   || defined(__AVR_ATmega48__)  || \
      defined(__AVR_ATmega88__)  || defined(__AVR_ATmega168__) || \
      defined(__AVR_ATmega328P__)
    TCCR2A |= _BV(COM2B1) | _BV(WGM20) | _BV(WGM21);
    TCCR2B  = freq & 0x7;
    OCR2B   = 0;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    TCCR3A |= _BV(COM1C1) | _BV(WGM10);
    TCCR3B  = (freq & 0x7) | _BV(WGM12);
    OCR3C   = 0;
#elif defined(__PIC32MX__)
    if (!MC.TimerInitalized) {
        T2CON = 0x8000 | ((freq & 0x07) << 4);
        TMR2 = 0x0000;
        PR2  = 0x0100;
        MC.TimerInitalized = true;
    }
    OC1CON = 0x8006;
    OC1RS  = 0;
    OC1R   = 0;
#else
    #error "This chip is not supported!"
#endif
#if !defined(AFMOTOR_NO_DIRECT_PWM)
    pinMode(3, OUTPUT);
#endif
}

inline void setPWM2(uint8_t s) {
#if defined(AFMOTOR_NO_DIRECT_PWM)
    analogWrite(3, s);
#elif defined(__AVR_ATmega8__)   || defined(__AVR_ATmega48__)  || \
      defined(__AVR_ATmega88__)  || defined(__AVR_ATmega168__) || \
      defined(__AVR_ATmega328P__)
    OCR2B = s;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    OCR3C = s;
#elif defined(__PIC32MX__)
    OC1RS = s;
#else
    #error "This chip is not supported!"
#endif
}

/* ---------------- PWM channel 3  (motor ENC, Arduino pin 6) --------------- */
inline void initPWM3(uint8_t freq) {
#if defined(AFMOTOR_NO_DIRECT_PWM)
    pinMode(6, OUTPUT);
    analogWrite(6, 0);
#elif defined(__AVR_ATmega8__)   || defined(__AVR_ATmega48__)  || \
      defined(__AVR_ATmega88__)  || defined(__AVR_ATmega168__) || \
      defined(__AVR_ATmega328P__)
    TCCR0A |= _BV(COM0A1) | _BV(WGM00) | _BV(WGM01);
    OCR0A   = 0;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    TCCR4A |= _BV(COM1A1) | _BV(WGM10);
    TCCR4B  = (freq & 0x7) | _BV(WGM12);
    OCR4A   = 0;
#elif defined(__PIC32MX__)
    if (!MC.TimerInitalized) {
        T2CON = 0x8000 | ((freq & 0x07) << 4);
        TMR2 = 0x0000;
        PR2  = 0x0100;
        MC.TimerInitalized = true;
    }
    OC3CON = 0x8006;
    OC3RS  = 0;
    OC3R   = 0;
#else
    #error "This chip is not supported!"
#endif
#if !defined(AFMOTOR_NO_DIRECT_PWM)
    pinMode(6, OUTPUT);
#endif
}

inline void setPWM3(uint8_t s) {
#if defined(AFMOTOR_NO_DIRECT_PWM)
    analogWrite(6, s);
#elif defined(__AVR_ATmega8__)   || defined(__AVR_ATmega48__)  || \
      defined(__AVR_ATmega88__)  || defined(__AVR_ATmega168__) || \
      defined(__AVR_ATmega328P__)
    OCR0A = s;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    OCR4A = s;
#elif defined(__PIC32MX__)
    OC3RS = s;
#else
    #error "This chip is not supported!"
#endif
}

/* ---------------- PWM channel 4  (motor END, Arduino pin 5) --------------- */
inline void initPWM4(uint8_t freq) {
#if defined(AFMOTOR_NO_DIRECT_PWM)
    pinMode(5, OUTPUT);
    analogWrite(5, 0);
#elif defined(__AVR_ATmega8__)   || defined(__AVR_ATmega48__)  || \
      defined(__AVR_ATmega88__)  || defined(__AVR_ATmega168__) || \
      defined(__AVR_ATmega328P__)
    TCCR0A |= _BV(COM0B1) | _BV(WGM00) | _BV(WGM01);
    OCR0B   = 0;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    TCCR3A |= _BV(COM1A1) | _BV(WGM10);
    TCCR3B  = (freq & 0x7) | _BV(WGM12);
    OCR3A   = 0;
#elif defined(__PIC32MX__)
    if (!MC.TimerInitalized) {
        T2CON = 0x8000 | ((freq & 0x07) << 4);
        TMR2 = 0x0000;
        PR2  = 0x0100;
        MC.TimerInitalized = true;
    }
    OC2CON = 0x8006;
    OC2RS  = 0;
    OC2R   = 0;
#else
    #error "This chip is not supported!"
#endif
#if !defined(AFMOTOR_NO_DIRECT_PWM)
    pinMode(5, OUTPUT);
#endif
}

inline void setPWM4(uint8_t s) {
#if defined(AFMOTOR_NO_DIRECT_PWM)
    analogWrite(5, s);
#elif defined(__AVR_ATmega8__)   || defined(__AVR_ATmega48__)  || \
      defined(__AVR_ATmega88__)  || defined(__AVR_ATmega168__) || \
      defined(__AVR_ATmega328P__)
    OCR0B = s;
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    OCR3A = s;
#elif defined(__PIC32MX__)
    OC2RS = s;
#else
    #error "This chip is not supported!"
#endif
}

/* -------------------------------------------------------------------------
 *  The remainder of the original AFMotor.cpp is **unchanged**.
 *  It covers AF_DCMotor, AF_Stepper, and helper logic, and compiles on all
 *  architectures because the low‑level PWM helpers above absorbed the 4809
 *  differences.  To keep this patch readable, scroll down if you need to
  *  inspect that part – everything is exactly as the user provided.
 * -------------------------------------------------------------------------*/

/* -----  ORIGINAL CODE CONTINUES BELOW (identical to user submission) ----- */

AF_DCMotor::AF_DCMotor(uint8_t num, uint8_t freq) {
  motornum = num;
  pwmfreq = freq;

  MC.enable();

  switch (num) {
  case 1:
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM1(freq);
    break;
  case 2:
    latch_state &= ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM2(freq);
    break;
  case 3:
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM3(freq);
    break;
  case 4:
    latch_state &= ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // set both motor pins to 0
    MC.latch_tx();
    initPWM4(freq);
    break;
  }
}

void AF_DCMotor::run(uint8_t cmd) {
  uint8_t a, b;
  switch (motornum) {
  case 1:
    a = MOTOR1_A; b = MOTOR1_B; break;
  case 2:
    a = MOTOR2_A; b = MOTOR2_B; break;
  case 3:
    a = MOTOR3_A; b = MOTOR3_B; break;
  case 4:
    a = MOTOR4_A; b = MOTOR4_B; break;
  default:
    return;
  }
  
  switch (cmd) {
  case FORWARD:
    latch_state |= _BV(a);
    latch_state &= ~_BV(b); 
    MC.latch_tx();
    break;
  case BACKWARD:
    latch_state &= ~_BV(a);
    latch_state |= _BV(b); 
    MC.latch_tx();
    break;
  case RELEASE:
    latch_state &= ~_BV(a);     // A and B both low
    latch_state &= ~_BV(b); 
    MC.latch_tx();
    break;
  }
}

void AF_DCMotor::setSpeed(uint8_t speed) {
  switch (motornum) {
  case 1:
    setPWM1(speed); break;
  case 2:
    setPWM2(speed); break;
  case 3:
    setPWM3(speed); break;
  case 4:
    setPWM4(speed); break;
  }
}

/******************************************
               STEPPERS
******************************************/

AF_Stepper::AF_Stepper(uint16_t steps, uint8_t num) {
  MC.enable();

  revsteps = steps;
  steppernum = num;
  currentstep = 0;

  if (steppernum == 1) {
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B) &
      ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // all motor pins to 0
    MC.latch_tx();
    
    // enable both H bridges
    pinMode(11, OUTPUT);
    pinMode(3, OUTPUT);
    digitalWrite(11, HIGH);
    digitalWrite(3, HIGH);

    // use PWM for microstepping support
    initPWM1(STEPPER1_PWM_RATE);
    initPWM2(STEPPER1_PWM_RATE);
    setPWM1(255);
    setPWM2(255);

  } else if (steppernum == 2) {
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B) &
      ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // all motor pins to 0
    MC.latch_tx();

    // enable both H bridges
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);

    // use PWM for microstepping support
    // use PWM for microstepping support
    initPWM3(STEPPER2_PWM_RATE);
    initPWM4(STEPPER2_PWM_RATE);
    setPWM3(255);
    setPWM4(255);
  }
}

void AF_Stepper::setSpeed(uint16_t rpm) {
  usperstep = 60000000 / ((uint32_t)revsteps * (uint32_t)rpm);
  steppingcounter = 0;
}

void AF_Stepper::release(void) {
  if (steppernum == 1) {
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B) &
      ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // all motor pins to 0
    MC.latch_tx();
  } else if (steppernum == 2) {
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B) &
      ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // all motor pins to 0
    MC.latch_tx();
  }
}

void AF_Stepper::step(uint16_t steps, uint8_t dir,  uint8_t style) {
  uint32_t uspers = usperstep;
  uint8_t ret = 0;

  if (style == INTERLEAVE) {
    uspers /= 2;
  }
 else if (style == MICROSTEP) {
    uspers /= MICROSTEPS;
    steps *= MICROSTEPS;
#ifdef MOTORDEBUG
    Serial.print("steps = "); Serial.println(steps, DEC);
#endif
  }

  while (steps--) {
    ret = onestep(dir, style);
    delay(uspers/1000); // in ms
    steppingcounter += (uspers % 1000);
    if (steppingcounter >= 1000) {
      delay(1);
      steppingcounter -= 1000;
    }
  }
  if (style == MICROSTEP) {
    while ((ret != 0) && (ret != MICROSTEPS)) {
      ret = onestep(dir, style);
      delay(uspers/1000); // in ms
      steppingcounter += (uspers % 1000);
      if (steppingcounter >= 1000) {
	delay(1);
	steppingcounter -= 1000;
      } 
    }
  }
}

uint8_t AF_Stepper::onestep(uint8_t dir, uint8_t style) {
  uint8_t a, b, c, d;
  uint8_t ocrb, ocra;

  ocra = ocrb = 255;

  if (steppernum == 1) {
    a = _BV(MOTOR1_A);
    b = _BV(MOTOR2_A);
    c = _BV(MOTOR1_B);
    d = _BV(MOTOR2_B);
  } else if (steppernum == 2) {
    a = _BV(MOTOR3_A);
    b = _BV(MOTOR4_A);
    c = _BV(MOTOR3_B);
    d = _BV(MOTOR4_B);
  } else {
    return 0;
  }

  // next determine what sort of stepping procedure we're up to
  if (style == SINGLE) {
    if ((currentstep/(MICROSTEPS/2)) % 2) { // we're at an odd step, weird
      if (dir == FORWARD) {
	currentstep += MICROSTEPS/2;
      }
      else {
	currentstep -= MICROSTEPS/2;
      }
    } else {           // go to the next even step
      if (dir == FORWARD) {
	currentstep += MICROSTEPS;
      }
      else {
	currentstep -= MICROSTEPS;
      }
    }
  } else if (style == DOUBLE) {
    if (! (currentstep/(MICROSTEPS/2) % 2)) { // we're at an even step, weird
      if (dir == FORWARD) {
	currentstep += MICROSTEPS/2;
      } else {
	currentstep -= MICROSTEPS/2;
      }
    } else {           // go to the next odd step
      if (dir == FORWARD) {
	currentstep += MICROSTEPS;
      } else {
	currentstep -= MICROSTEPS;
      }
    }
  } else if (style == INTERLEAVE) {
    if (dir == FORWARD) {
       currentstep += MICROSTEPS/2;
    } else {
       currentstep -= MICROSTEPS/2;
    }
  } 

  if (style == MICROSTEP) {
    if (dir == FORWARD) {
      currentstep++;
    } else {
      // BACKWARDS
      currentstep--;
    }

    currentstep += MICROSTEPS*4;
    currentstep %= MICROSTEPS*4;

    ocra = ocrb = 0;
    if ( (currentstep >= 0) && (currentstep < MICROSTEPS)) {
      ocra = microstepcurve[MICROSTEPS - currentstep];
      ocrb = microstepcurve[currentstep];
    } else if  ( (currentstep >= MICROSTEPS) && (currentstep < MICROSTEPS*2)) {
      ocra = microstepcurve[currentstep - MICROSTEPS];
      ocrb = microstepcurve[MICROSTEPS*2 - currentstep];
    } else if  ( (currentstep >= MICROSTEPS*2) && (currentstep < MICROSTEPS*3)) {
      ocra = microstepcurve[MICROSTEPS*3 - currentstep];
      ocrb = microstepcurve[currentstep - MICROSTEPS*2];
    } else if  ( (currentstep >= MICROSTEPS*3) && (currentstep < MICROSTEPS*4)) {
      ocra = microstepcurve[currentstep - MICROSTEPS*3];
      ocrb = microstepcurve[MICROSTEPS*4 - currentstep];
    }
  }

  currentstep += MICROSTEPS*4;
  currentstep %= MICROSTEPS*4;

#ifdef MOTORDEBUG
  Serial.print("current step: "); Serial.println(currentstep, DEC);
  Serial.print(" pwmA = "); Serial.print(ocra, DEC); 
  Serial.print(" pwmB = "); Serial.println(ocrb, DEC); 
#endif

  if (steppernum == 1) {
    setPWM1(ocra);
    setPWM2(ocrb);
  } else if (steppernum == 2) {
    setPWM3(ocra);
    setPWM4(ocrb);
  }


  // release all
  latch_state &= ~a & ~b & ~c & ~d; // all motor pins to 0

  //Serial.println(step, DEC);
  if (style == MICROSTEP) {
    if ((currentstep >= 0) && (currentstep < MICROSTEPS))
      latch_state |= a | b;
    if ((currentstep >= MICROSTEPS) && (currentstep < MICROSTEPS*2))
      latch_state |= b | c;
    if ((currentstep >= MICROSTEPS*2) && (currentstep < MICROSTEPS*3))
      latch_state |= c | d;
    if ((currentstep >= MICROSTEPS*3) && (currentstep < MICROSTEPS*4))
      latch_state |= d | a;
  } else {
    switch (currentstep/(MICROSTEPS/2)) {
    case 0:
      latch_state |= a; // energize coil 1 only
      break;
    case 1:
      latch_state |= a | b; // energize coil 1+2
      break;
    case 2:
      latch_state |= b; // energize coil 2 only
      break;
    case 3:
      latch_state |= b | c; // energize coil 2+3
      break;
    case 4:
      latch_state |= c; // energize coil 3 only
      break; 
    case 5:
      latch_state |= c | d; // energize coil 3+4
      break;
    case 6:
      latch_state |= d; // energize coil 4 only
      break;
    case 7:
      latch_state |= d | a; // energize coil 1+4
      break;
    }
  }

 
  MC.latch_tx();
  return currentstep;
}

