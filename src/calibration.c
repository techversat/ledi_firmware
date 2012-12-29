#include "calibration.h"
#include <avr/io.h>
#include <avr/interrupt.h>

//! Holds the number of neighbors searched
unsigned char neighborsSearched;
//! The binary search step size
unsigned char calStep;
//! The lowest difference between desired and measured counter value
unsigned char bestCountDiff = 0xFF;
//! Stores the lowest difference between desired and measured counter value for the first search
unsigned char bestCountDiff_first;
//! The OSCCAL value corresponding to the bestCountDiff
unsigned char bestOSCCAL;
//! Stores the OSCCAL value corresponding to the bestCountDiff for the first search
unsigned char bestOSCCAL_first;
//! The desired counter value
unsigned int countVal;
//! Calibration status
unsigned int calibration;
//! Stores the direction of the binary step (-1 or 1)
signed char sign;

//Functions used
void CalibrationInit(void);
void CalibrateInternalRc(void);
unsigned int Counter(void);
void BinarySearch(unsigned int ct);
void NeighborSearch(void);

/*! \brief Program entry point.
*
* Main initializes all subsystems, prepares and starts calibration
* according to the calibration method chosen, and the device specific
* oscillator characteristics. Ends in an eternal loop.
*
*/

uint8_t do_calibration(void)
{
  CalibrationInit();                         // Initiates calibration
  PREPARE_CALIBRATION();                     // Sets initial stepsize and sets calibration state to "running"
  CalibrateInternalRc();                     // Calibrates to selected frequency

#ifndef CALIBRATION_METHOD_SIMPLE            // If simple search method is chosen, there is no need to do two calibrations.
#ifdef TWO_RANGES                            // For devices with splitted OSCCAL register.
  if (bestCountDiff != 0x00)                 // Do not do a second search if perfect match
  {
    OSCCAL = DEFAULT_OSCCAL_HIGH;            // Sets search range to upper part of OSCCAL
    NOP();
    bestOSCCAL_first = bestOSCCAL;           // Save OSCCAL value and count difference achieved in first calibration
    bestCountDiff_first = bestCountDiff;
    PREPARE_CALIBRATION();                   // Search performed in lower OSCCAL range, perform search in upper OSCCAl range
    CalibrateInternalRc();                   // Perform a second search in upper part of OSCCAL

    if (bestCountDiff > bestCountDiff_first) // Check which search gave the best calibration
    {
      OSCCAL = bestOSCCAL_first;             // First calibration is more accurate and OSCCAL is written accordingly
      NOP();
    }
  }
#endif
#endif

  return OSCCAL;
}


/*! \brief Initializes the calibration.
*
* Computes the count value needed to compare the desired internal oscillator
* speed with the external watch crystal, and sets up the asynchronous timer.
*
*/
void CalibrationInit(void){

  COMPUTE_COUNT_VALUE();                     // Computes countVal for use in the calibration
  OSCCAL = DEFAULT_OSCCAL;
  NOP(); // NOP(); NOP(); NOP(); NOP();
  SETUP_ASYNC_TIMER();                       // Asynchronous timer setup
}

/*! \brief Calibration function
*
* Performs the calibration according to calibration method chosen.
* Compares different calibration results in order to achieve optimal results.
*
*/
void CalibrateInternalRc(void)
{
  unsigned int count;

#ifdef CALIBRATION_METHOD_SIMPLE                    // Simple search method
  unsigned char cycles = 0x80;

  do{
    count = Counter();
    if (count > countVal)
      OSCCAL--;                                     // If count is more than count value corresponding to the given frequency:
    NOP();                                          // - decrease speed
    if (count < countVal)
      OSCCAL++;
    NOP();                                          // If count is less: - increase speed
    if (count == countVal)
      cycles=1;			
  } while(--cycles);                                // Calibrate using 128(0x80) calibration cycles

#else                                               // Binary search with or without neighbor search
  unsigned char countDiff;
  unsigned char neighborSearchStatus = FINISHED;

  while(calibration == RUNNING){
    count = Counter();                              // Counter returns the count value after external ticks on XTAL
    if (calStep != 0)
    {
      BinarySearch(count);                          // Do binary search until stepsize is zero
    }
    else
    {
      if(neighborSearchStatus == RUNNING)
      {
        countDiff = ABS((signed int)count-(signed int)countVal);
        if (countDiff < bestCountDiff)                          // Store OSCCAL if higher accuracy is achieved
        {
          bestCountDiff = countDiff;
          bestOSCCAL = OSCCAL;
        }
        NeighborSearch();                                       // Do neighbor search
      }
      else                                                      // Prepare and start neighbor search
      {
#ifdef CALIBRATION_METHOD_BINARY_WITHOUT_NEIGHBOR               // No neighbor search if deselected
        calibration = FINISHED;
        countDiff = ABS((signed int)count-(signed int)countVal);
        bestCountDiff = countDiff;
        bestOSCCAL = OSCCAL;
#else
        neighborSearchStatus = RUNNING;                         // Do neighbor search by default
        neighborsSearched = 0;
        countDiff = ABS((signed int)count-(signed int)countVal);
        bestCountDiff = countDiff;
        bestOSCCAL = OSCCAL;
#endif
      }
    }
  }
#endif
}

/*! \brief The Counter function
*
* This function increments a counter for a given ammount of ticks on
* on the external watch crystal.
*
*/
unsigned int Counter(void)
{
  unsigned int cnt;
  cnt = 0;                                       // Reset counter
  TIMER = 0x00;                                  // Reset async timer/counter
  while(ASSR & (
      (1<<OUTPUT_COMPARE_UPDATE_BUSY)|
      (1<<TIMER_UPDATE_BUSY)|
      (1<<ASYNC_TIMER_CONTROL_UPDATE_BUSY)));    // Wait until async timer is updated  (Async Status reg. busy flags).
  do{          // cnt++: Increment counter - the add immediate to word (ADIW) takes 2 cycles of code.
    cnt++;     // Devices with async TCNT in I/0 space use 1 cycle reading, 2 for devices with async TCNT in extended I/O space
  } while (TIMER < EXTERNAL_TICKS);   // CPI takes 1 cycle, BRCS takes 2 cycles, resulting in: 2+1(or 2)+1+2=6(or 7) CPU cycles
  return cnt;                                    // NB! Different compilers may give different CPU cycles!
}                                                // Until 32.7KHz (XTAL FREQUENCY) * EXTERNAL TICKS

/*! \brief The binary search method
*
* This function uses the binary search method to find the
* correct OSSCAL value.
*
*/
void BinarySearch(unsigned int ct)
{
  if (ct > countVal)           // Check if count is larger than desired value
  {
    sign = -1;                 // Saves the direction
    OSCCAL -= calStep;         // Decrease OSCCAL if count is too high
    NOP();
  }
  else if (ct < countVal)      // Opposite procedure for lower value
  {
    sign = 1;
    OSCCAL += calStep;
    NOP();
  }
  else                         // Perfect match, OSCCAL stays unchanged
  {
    calibration = FINISHED;
  }
  calStep >>= 1;
}

/*! \brief The neighbor search method
*
* This function uses the neighbo search method to improve
* binary search result. Will always be called with a binary search
* prior to it.
*
*/
void NeighborSearch(void)
{
  neighborsSearched++;
  if (neighborsSearched == 4)     // Finish if 3 neighbors searched
  {
    OSCCAL = bestOSCCAL;
    calibration = FINISHED;
  }
  else
  {
    OSCCAL+=sign;
    NOP();
  }
}

