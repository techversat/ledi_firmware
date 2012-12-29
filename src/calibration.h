#ifndef _H_CALIBRATION
#define _H_CALIBRATION


#define CALIBRATION_METHOD_SIMPLE          1

#define ASYNC_TIMER                        AS2
#define NO_PRESCALING                      CS20
#define ASYNC_TIMER_CONTROL_REGISTER       TCCR2B
#define ASYNC_TIMER_CONTROL_UPDATE_BUSY    TCR2AUB
#define OUTPUT_COMPARE_UPDATE_BUSY         OCR2AUB
#define TIMER_UPDATE_BUSY                  TCN2UB
#define TIMER                              TCNT2
#define OSCCAL_RESOLUTION                  7
#define LOOP_CYCLES                        7
#define TWO_RANGES

#ifdef F_CPU
#define CALIBRATION_FREQUENCY F_CPU
#else
#define CALIBRATION_FREQUENCY 8000000
#endif

/*! Frequency of the external oscillator. A 32kHz crystal is recommended
 */
#define XTAL_FREQUENCY 32768
#define EXTERNAL_TICKS 100               // ticks on XTAL. Modify to increase/decrease accuracy

/*! \brief Fixed calibration values and macros
 *
 * These values are fixed and used by all calibration methods. Not to be modified.
 *
 */
#define FALSE 0
#define TRUE 1
#define RUNNING 0
#define FINISHED 1
#define DEFAULT_OSCCAL_MASK        0x00  // Lower half and
#define DEFAULT_OSCCAL_MASK_HIGH   0x80  // upper half for devices with splitted OSCCAL register

#define DEFAULT_OSCCAL_HIGH ((1 << (OSCCAL_RESOLUTION - 1)) | DEFAULT_OSCCAL_MASK_HIGH)
#define INITIAL_STEP         (1 << (OSCCAL_RESOLUTION - 2))
#define DEFAULT_OSCCAL      ((1 << (OSCCAL_RESOLUTION - 1)) | DEFAULT_OSCCAL_MASK)

// **** Functions implemented as macros to avoid function calls
#define PREPARE_CALIBRATION() \
calStep = INITIAL_STEP; \
calibration = RUNNING;

#define COMPUTE_COUNT_VALUE() \
countVal = ((EXTERNAL_TICKS*CALIBRATION_FREQUENCY)/(XTAL_FREQUENCY*LOOP_CYCLES));

// Set up timer to be ASYNCHRONOUS from the CPU clock with a second EXTERNAL 32,768kHz CRYSTAL driving it. No prescaling on asynchronous timer.
#define SETUP_ASYNC_TIMER() \
ASSR |= (1<<ASYNC_TIMER); \
ASYNC_TIMER_CONTROL_REGISTER = (1<<NO_PRESCALING);

#ifndef NOP()
#define NOP() __asm__ __volatile__ ("nop")
#endif

// #define NOP() __no_operation()
// Absolute value macro.
#define ABS(var) (((var) < 0) ? -(var) : (var));

unsigned char do_calibration(void);

#endif
