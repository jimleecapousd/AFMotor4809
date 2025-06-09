/*
  Patched Adafruit Motor Shield Library – Header (ATmega4809 compatible)
  --------------------------------------------------------------------
  This file replaces the original AFMotor.h so the library builds on the
  Arduino Uno WiFi Rev 2 / Nano Every (ATmega4809, megaAVR‑0 core).

  Changes made:
  * Add a dedicated block for __AVR_ATmega4809__, defining:
      – AFMOTOR_NO_DIRECT_PWM (tells the .cpp to skip low‑level timer hacks)
      – dummy prescale constants so existing code compiles even though they
        are never used on this MCU (we rely on analogWrite()).
  * Guard the legacy AVR register‑specific constants so they are *not*
    evaluated when compiling for the 4809 – otherwise names like CS20 are
    undefined and the pre‑processor fails.

  No functional changes for classic AVRs or PIC32.
*/

#ifndef _AFMotor_h_
#define _AFMotor_h_

#include <inttypes.h>

/* -------------------------------------------------------------------------
 *  megaAVR‑0 family (ATmega4809) – new block
 * -------------------------------------------------------------------------*/
#if defined(__AVR_ATmega4809__)

  // Notify the implementation to fall back to analogWrite()
  #define AFMOTOR_NO_DIRECT_PWM

  // Library features
  #define MICROSTEPS 16   // keep full micro‑stepping support

  /*
     The original library tweaks timer prescalers via constants such as
     MOTOR12_8KHZ that expand to _BV(CS21).  Those bit names do not exist on
     the 4809.  We still need symbolic values so the higher‑level code that
     stores them in variables compiles, even though they will be ignored.
  */
  #define MOTOR12_64KHZ 0
  #define MOTOR12_8KHZ  1
  #define MOTOR12_2KHZ  2
  #define MOTOR12_1KHZ  3

  #define MOTOR34_64KHZ 0
  #define MOTOR34_8KHZ  1
  #define MOTOR34_1KHZ  2

  // Pick sensible defaults (they do not affect PWM on this MCU)
  #define DC_MOTOR_PWM_RATE   MOTOR34_8KHZ
  #define STEPPER1_PWM_RATE   MOTOR12_64KHZ
  #define STEPPER2_PWM_RATE   MOTOR34_64KHZ

/* -------------------------------------------------------------------------
 *  Classic 8‑bit AVRs – unchanged
 * -------------------------------------------------------------------------*/
#elif defined(__AVR__)

  #include <avr/io.h>
  //#define MOTORDEBUG 1

  #define MICROSTEPS 16                       // 8 or 16

  #define MOTOR12_64KHZ _BV(CS20)             // no prescale
  #define MOTOR12_8KHZ  _BV(CS21)             // divide by 8
  #define MOTOR12_2KHZ (_BV(CS21) | _BV(CS20))  // divide by 32
  #define MOTOR12_1KHZ _BV(CS22)              // divide by 64

  #define MOTOR34_64KHZ _BV(CS00)             // no prescale
  #define MOTOR34_8KHZ  _BV(CS01)             // divide by 8
  #define MOTOR34_1KHZ (_BV(CS01) | _BV(CS00))  // divide by 64

  #define DC_MOTOR_PWM_RATE   MOTOR34_8KHZ    // PWM rate for DC motors
  #define STEPPER1_PWM_RATE   MOTOR12_64KHZ   // PWM rate for stepper 1
  #define STEPPER2_PWM_RATE   MOTOR34_64KHZ   // PWM rate for stepper 2

/* -------------------------------------------------------------------------
 *  PIC32 branch – original
 * -------------------------------------------------------------------------*/
#elif defined(__PIC32MX__)

  //#define MOTORDEBUG 1

  // Uncomment if you jumper pin 9→11 or 10→11 for PWM on M1
  //#define PIC32_USE_PIN9_FOR_M1_PWM
  //#define PIC32_USE_PIN10_FOR_M1_PWM

  #define MICROSTEPS 16

  // Timer‑2 prescaler settings by target PWM frequency
  #define MOTOR12_312KHZ  0
  #define MOTOR12_156KHZ  1
  #define MOTOR12_64KHZ   2
  #define MOTOR12_39KHZ   3
  #define MOTOR12_19KHZ   4
  #define MOTOR12_8KHZ    5
  #define MOTOR12_4_8KHZ  6
  #define MOTOR12_2KHZ    7
  #define MOTOR12_1KHZ    7

  #define MOTOR34_312KHZ  0
  #define MOTOR34_156KHZ  1
  #define MOTOR34_64KHZ   2
  #define MOTOR34_39KHZ   3
  #define MOTOR34_19KHZ   4
  #define MOTOR34_8KHZ    5
  #define MOTOR34_4_8KHZ  6
  #define MOTOR34_2KHZ    7
  #define MOTOR34_1KHZ    7

  #define DC_MOTOR_PWM_RATE   MOTOR34_39KHZ
  #define STEPPER1_PWM_RATE   MOTOR12_39KHZ   // same for PIC32
  #define STEPPER2_PWM_RATE   MOTOR34_39KHZ

#endif  // architecture selection

/* -------------------------------------------------------------------------
 *  Register map (shared by all MCUs)
 * -------------------------------------------------------------------------*/

// Bit positions in the 74HC595 output
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR3_A 5
#define MOTOR3_B 7

// Motion directions
#define FORWARD  1
#define BACKWARD 2
#define BRAKE    3
#define RELEASE  4

// Stepper styles
#define SINGLE      1
#define DOUBLE      2
#define INTERLEAVE  3
#define MICROSTEP   4

// Arduino pins wired to the shield’s 74HC595
#define MOTORLATCH  12
#define MOTORCLK    4
#define MOTORENABLE 7
#define MOTORDATA   8

/* -------------------------------------------------------------------------
 *  Class declarations (unchanged)
 * -------------------------------------------------------------------------*/
class AFMotorController {
  public:
    AFMotorController(void);
    void enable(void);
    friend class AF_DCMotor;
    void latch_tx(void);
    uint8_t TimerInitalized;
};

class AF_DCMotor {
  public:
    AF_DCMotor(uint8_t motornum, uint8_t freq = DC_MOTOR_PWM_RATE);
    void run(uint8_t);
    void setSpeed(uint8_t);
  private:
    uint8_t motornum, pwmfreq;
};

class AF_Stepper {
  public:
    AF_Stepper(uint16_t, uint8_t);
    void step(uint16_t steps, uint8_t dir, uint8_t style = SINGLE);
    void setSpeed(uint16_t);
    uint8_t onestep(uint8_t dir, uint8_t style);
    void release(void);
    uint16_t revsteps;  // steps per revolution
    uint8_t steppernum;
    uint32_t usperstep, steppingcounter;
  private:
    uint8_t currentstep;
};

// Helper exposed by original library
uint8_t getlatchstate(void);

#endif  // _AFMotor_h_
