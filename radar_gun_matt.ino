// Matthew Miller
// Dec 2025
// Based on project from Kevin Darrah - http://www.kevindarrah.com/wiki/index.php?title=Arduino_Radar_Gun
// NOTE - SET BOARD TYPE "Arduino Micro" for Leonardo Micro board or TX/RX LEDs will be stuck on!

/*
 * Initial version
 *   Created initial version
 * v2
 *   Optimized readability
 *   Added serial command processing
 *   Optimized LCD digit decoding (NOTE - this version may have decoding bug in parameter order)
 *   Improved preprocessor directives for debugging
 * v3
 *   LCD digit decoding (NOTE - this version may have decoding bug in parameter order)
 *   Improved initialization
 *   Improved preprocessor directives for debugging
 *   Changed serial print strings to F() for lower RAM use
 *   Improved main processing loop
 * v4
 *   LCD digit decoding (NOTE - this version may have decoding bug in parameter order)
 *   Added debugging for sleep and RFI diagnostics
 *   Fixed main processing loop bug which could cause radar to keep radiating when scan time set to zero
 *   Improved main processing efficiency
 *   Added offset angle correction processing for actual vs measured speed
 *   Improved power on button press timing 
 *   Added init/reboot option for serial commands
 * v5
 *   Corrected error in LCD digit decoding (parameter swapped)
 *   Added additional debugging options
 *   Added main loop timing information display options
 *   Added ability to change units between MPH/KPH
 *   Improved power button press logic
 *   Fixed bug in serial read logic due to misplaced breaks in case statments
 *   Fixed bug in sDelay causing unexpected results if a long operation resulted in negative delay
 * v6
 *   Improved readability
 *   Added dynamic timing to main loop to more accurately process scan/off time values
 *   Added option to always print first scan speed reading even if it is a duplicate of previous reading
 *   Added option to supress printing zero-speed values
 *   Optimized MPH/KPH unit selection function calls 
 *   Optimized some serial command processing
 * v7 - Jan 2026
 *   Added option to verify-reading to perform second radar scan (to help combat interference)
 *   Added debugging for LCD decoding processing
 *   Added +/- sign to verified speed % to help indicate which direction the error is believed to be
 *   Adjusted defaults for verifying speed
 *   Added check to attempt recovery during power-on failure loop
 *   Improved power-on self-diagnostics and warnings
 *   Adjusted LCD scan to reduce chances of validation errors
 * v8
 *   Fixed error in debug preprocessor directives
 *   Added delay option before verification-reading scans
 *   Enhanced verify-reading to allow multiple verification scans, identifying worst % error
 *   Re-ordered some things for readability and consistency
 *   Added some ASCII-Art to source to help explain some options
 *   Added printing ETime for scan and verify vs main loop
 *   Added warnings if timing looks bad (verify too long to fit in off-time)
 *   Added warnings if duty cycle is >50% with projected verify timings
 *   Added warnings if main loop is repeatedly over-running timing
 *   Fixed alignment of print statments for readability
 */



// OUTPUT AND PROCESSING DEBUG OPTIONS
//WARNING - enabling extra debugging may affect timings
#define INFO_ON                // Print informational outputs (e.g. sent power commands, low battery)
//#define DEBUG_ON               // Print debug statments
//#define DEBUG_INVALID_LCD      // Prints information if LCD decode was invalid
//#define DEBUG_LCD_DECODE       // Prints information about LCD decoding
//#define DEBUG_CONTROLS_ON      // Run test of button actions during startup
//#define DEBUG_TRIGGER_ON       // Print information when pulling/releasing trigger
//#define DEBUG_PBUTTON_ON       // Print information when pulling/releasing power button
//#define DEBUG_MAIN_LOOP_TIME   // Print elapsed time metrics for main processing loop
//#define DEBUG_SDELAY           // Print debug statments for sDelay calls
//#define DEBUG_SCAN_OFF_DELAY   // Print debug statments for scan and off timing
//#define DEBUG_LED_PIN 13       // Flash LED pin 13 to debug USB signal detect and sleep
//#define DEBUG_SLEEP_FOREVER    // After setup, sleeps forever.  Used for debugging RFI issues when running.
//#define DEBUG_NEVER_SLEEP      // Ignore sleep commands.  Used for debugging without pins connected.

// INPUT DEBUG OPTIONS
#define INFO_SERIAL_INPUT_ON   // Print informational outputs (e.g. echo back commands after set)
//#define DEBUG_SERIAL_INPUT_ON  // Print debug statments related to serial input received

// DEFAULT VARIABLE CONFIGURATION OPTIONS
//      RADIATE suggested times with 0 or 1 verify scan - 125/375 (half-second loop) or 250/775 (one-second loop)
//      RADIATE suggested times with 2 or 3 verify scans - 125/875 (one-second loop) or 250/2250 (2.5 second loop)
//                                NOTE minimum time must be greater than (LCD_SCAN_DELAY + LCD_SCAN_TIME)
#define RADIATE_SCAN_TIME 125   // mS duration of radar active scan; (0)=never radiate; (-1)=always on
#define RADIATE_OFF_TIME  875   // mS delay idle between pricessing loop iterations
#define VERIFY_SPEED      2     // >= 1 - Performs "n" additional scans, runs during 'off' time, may affect duty cycle
#define VERIFY_WAIT       125   // mS delay before additional scans.  If VERIFY_SPEED = 0 this has no effect.
#define AUTO_RUN_RADAR    true  // true - start/stop radar automatically; false - control only by serial
#define PRINT_FIRST_SPEED true  // true - print first scan after trigger pulled even if duplicate
#define PRINT_ZERO_SPEED  false // true - print speed values of zero; false - print only values >0

/*
 * Example timings:
 * S = Scan
 * o = Off
 * V = Verify Scanning
 * w = Verify Wait
 * 
 * RADIATE_SCAN_TIME 125
 * RADIATE_OFF_TIME  875
 * VERIFY_SPEED      0
 * VERIFY_WAIT       125
 * NOTE - basic scan loop
 * Time-graph in 1/8 second (125mS) increments:
 * SoooooooSoooooooSooooooo
 * \__1s__/\__1s__/\__1s__/
 * 
 * VERIFY_SPEED      2
 * NOTE - runs during "off" affects duty cycle
 * Time-graph in 1/8 second (125mS) increments:
 * SwVwVoooSwVwVoooSwVwVooo
 * \__1s__/\__1s__/\__1s__/ 
 * 
 * VERIFY_SPEED      3
 * NOTE - runs during "off" affects duty cycle
 * Time-graph in 1/8 second (125mS) increments:
 * SwVwVwVoSwVwVwVoSwVwVwVo
 * \__1s__/\__1s__/\__1s__/
 * 
 * VERIFY_SPEED      5
 * NOTE - pushes too long alters timing
 * Time-graph in 1/8 second (125mS) increments:
 * SwVwVwVwVwVSwVwVwVwVwVSwVwVwVwVwV
 * \__1s__/xxx\__1s__/xxx\__1s__/
 * Notice 'x' runs past 1 second interval
 */

// CONTROL OPTIONS
#define LCD_SCAN_DELAY 10   // mS delay for LCD to stabilize before reading
#define LCD_SCAN_TIME  20   // mS duration to keep scanning LCD for active segments

// Control pins
// Pull output and pull low to activate
// Set input and low (no pull-up) to deactivate
#define TRIGGER_PIN   2  // Digital Pin
#define POWER_ON_PIN  3  // Digital Pin
#define POWER_OFF_PIN 12 // Digital Pin
#define POWER_OFF_SENSE_PIN 11 // Analog Pin
// Note - D12 is also an analog input so we can "wait a bit" after power-down for the voltage to bleed off or it gets stuck

// Battery vs USB detection pins
#define USB_POWER_PIN 7 // Digital Pin, should also be interrupt

/*
 * The available pins for attachInterupt() on the Leonardo are 0, 1, 2, 3, and 7.
 * INT0 on Pin 3 Interrupt 0
 * INT1 on Pin 2 Interrupt 1
 * INT2 on Pin 0 Interrupt 2
 * INT3 on Pin 1 Interrupt 3
 * INT6 on Pin 7 Interrupt 4
 */

 
// LCD PINs
/* These are sequential because we want to use them in loops 
 *  A0  = 0
 *  A1  = 1
 *  A2  = 3
 *  A3  = 3
 *  A4  = 4
 *  A5  = 5
 *  A6  = 6
 *  A7  = 7
 *  A8  = 8
 *  A9  = 9
 *  A10 = 10
 *  
 *  NOTE - for Leonardo, the pin numbering for additional analog pins:
 *  The correspondence of each analog with their digital counterparts is as follows:
 *  A0 – D18
 *  A1 – D19
 *  A2 – D20
 *  A3 – D21
 *  A4 – D22
 *  A5 – D23
 *  A6 – D4
 *  A7 – D6
 *  A8 – D8
 *  A9 – D9
 *  A10 – D10
 *  A11 – D12
 */



// **** end of configuration settings  ****
// **** program logic below this point ****

#include<avr/sleep.h>
#include<avr/wdt.h>
#include<math.h>

// ADC configuration optins
// https://r6500.blogspot.com/2015/01/fast-adc-on-arduino-leonardo.html
#define ADC_TIME_104  ADCSRA=(ADCSRA&0xF80)|0x07   
#define ADC_TIME_52   ADCSRA=(ADCSRA&0xF80)|0x06   
#define ADC_TIME_26   ADCSRA=(ADCSRA&0xF80)|0x05   
#define ADC_TIME_13   ADCSRA=(ADCSRA&0xF80)|0x04   
// The different settings set the ADC clock frequency, the conversion time
// and the equivalent number of bits
// ADC_TIME_104    f=125kHz   Tconv=104us    ENOB > 9 
// ADC_TIME_52     f=250kHz   Tconv=52us     ENOB > 9
// ADC_TIME_26     f=500kHz   Tconv=26us     ENOB > 9
// ADC_TIME_13     f=1MHz     Tconv=13us     ENOB > 8

int segment[4][7]; // LCD segment data stored here

int measuredSpeed=0, oldSpeed=0;     // converted data to speed
int actualSpeed=0,  offsetAngle = 0; // Speed after correction for cosine
boolean measuredSpeedValid = false;  // stores whether the speed data is valid
int verifiedMeasuredSpeedPercent = -999; // Used to compare if verifying the measured speed; -1 if invalid/unverified
boolean firstMeasurement = false;    // stores whether this is the first measurement of pulling trigger
int failedDecodeCount = 0;           // count of invalid readings since startup
unsigned long eTimeStartMainLoop = 0,
              eTimeEndVerifyLoop = 0,
              eTimeEndMainLoop = 0;  // used to compute main loop elapsed time
unsigned long radiateScanTimeElapsed = 0;     // Used to compensate for misc processing time in main loop
unsigned long radiateOffTimeElapsedEarly = 0; // Used to compensate for misc processing time in main loop
unsigned long radiateOffTimeElapsedLate = 0;  // Used to compensate for misc processing time in main loop
int powerOnFailCount = 0;            // Used to tell if we are in a power-on loop
int mainLoopOverrunCount = 0;        // Used to tell if the main loop is frequently over-running time

//Run control options (could be adjusted later, defaults set here)
long radiateOffTime = RADIATE_OFF_TIME;
long radiateScanTime = RADIATE_SCAN_TIME;
boolean autoRunRadar = AUTO_RUN_RADAR;
boolean printZero = PRINT_ZERO_SPEED;
boolean printFirst = PRINT_FIRST_SPEED;
int verifySpeed = VERIFY_SPEED;
long verifyWait = VERIFY_WAIT;



void setup()
{
  Serial.begin(115200);
  
  unsigned long watchdog = millis();
  //wait up to 5 sec for serial to initialize
  while(((millis()-watchdog) < 5000) && !Serial && isUsbPower());

  if(Serial)
  {
        Serial.println();
        Serial.println(F("Sketch:   " __FILE__));
        Serial.println(F("Compiled: " __DATE__ " " __TIME__));
        Serial.println(F("GCC:      " __VERSION__ ));
        Serial.println(F("Code Designed & Written by: Matthew Miller"));
        Serial.println(F("Based on project from Kevin Darrah\n\n"));
        Serial.println();
        Serial.println(F("Starting up..."));
  }

  // Initialize pins and such
  #ifdef DEBUG_LED_PIN
  pinMode(DEBUG_LED_PIN,OUTPUT);
  #endif
  
  // configure USB power detect pin - internal pullup on for transistor to pull down
  pinMode(USB_POWER_PIN,INPUT);
  digitalWrite(USB_POWER_PIN,HIGH);
  
  // configure sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  
  // adjust ADC to get faster readings (we can afford less precision)
  ADC_TIME_13;


  //show mode for debugging
  #ifdef DEBUG_ON
  if(Serial) 
  {
    
    Serial.println(F("----------------------------------------"));
    Serial.print(F("Power source: "));
    Serial.println( isUsbPower() ? F("USB") : F("Battery"));
    Serial.println(F("Setup complete, starting processing loop."));
    Serial.println();
    Serial.print(F("Trigger Button Level: "));
    Serial.println(digitalRead(TRIGGER_PIN));
    Serial.print(F("Power On Button Level: "));
    Serial.println(digitalRead(POWER_ON_PIN));
    Serial.print(F("Power Off Button Level: "));
    Serial.print(digitalRead(POWER_OFF_PIN));
    Serial.print(F(" / "));
    Serial.println(analogRead(POWER_OFF_SENSE_PIN));
    int x=micros();
    analogRead(POWER_OFF_SENSE_PIN);
    x = micros() - x;
    Serial.print(F("analogRead time micros = "));
    Serial.println(x);
    Serial.println(F("----------------------------------------"));
  }
  #endif
  
  #ifdef DEBUG_CONTROLS_ON
  buttonLcdSelfTest();
  #endif

  // Scan the LCD once so we know what state we are in going into the loop
  scanLcd();

  if(Serial) Serial.println(F("Ready."));

  #ifdef DEBUG_SLEEP_FOREVER
  if(Serial)
  {
    Serial.println(F("Sleeping forever in 10 seconds."));
    Serial.println(F("This should only be used for hardware debug."));
    Serial.println(F("Hardware reset or power cycle required to wake up."));
    Serial.flush();
  }
  delay(10000);   // Makes it easier to re-flashing
  sleepForever(); // For debugging the Arduino in sleep-mode
  #endif
  
}// end setup()



void loop()
{
  eTimeStartMainLoop = millis();
  
  #ifdef DEBUG_LED_PIN
  if(isUsbPower())
    flashDebugLED(200,3);
  #endif
  
  // If we are on battery power
  if(!isUsbPower())
  {
    if(isPowerOn())
    {
      // Turn off the radar
      powerOff();
    }
    
    // Put to deep sleep
    goToSleep();
  } // end if: battery power

  // Check if any new commands have been sent
  readSerialCommands();

  // Decide if we are running the radar or not
  if(autoRunRadar)
  {
    if(!isPowerOn())
    {
      powerOn();
    }

    if(eTimeEndMainLoop > getTargetLoopTime())
    {
      mainLoopOverrunCount++;
      
      #ifdef INFO_ON
      if(Serial && (mainLoopOverrunCount == 3 || (mainLoopOverrunCount % 10) == 0) && mainLoopOverrunCount <= 100)
      {
        Serial.print(F("WARNING: Main loop overrun "));
        Serial.print(mainLoopOverrunCount);
        Serial.print(F(" times.  Last iteration over by "));
        Serial.print(eTimeEndMainLoop-getTargetLoopTime());
        Serial.print(F("mS"));
        
        if(mainLoopOverrunCount == 100)
          Serial.println(F("  *** LAST WARNING! ***"));
        else
          Serial.println();
      }
      #endif
    }
    else
    {
      mainLoopOverrunCount=0;
    }

    // Pull the trigger to search for a target
    if(radiateScanTime > 0)
    {
      // Track non-radiating time before trigger
      radiateOffTimeElapsedEarly = millis() - radiateOffTimeElapsedEarly;
    
      radiateScanTimeElapsed = millis();  // Track radiating time after trigger
      holdTrigger();
      radiateScanTimeElapsed = millis()-radiateScanTimeElapsed;

          
      #ifdef DEBUG_SCAN_OFF_DELAY
      if(Serial) Serial.print(F("radiateScanTime = "));
      if(Serial) Serial.print(radiateScanTime);
      if(Serial) Serial.print(F("; radiateScanTimeElapsed = "));
      if(Serial) Serial.println(radiateScanTimeElapsed);
      #endif
      sDelay(radiateScanTime-radiateScanTimeElapsed);
      
      radiateOffTimeElapsedLate = millis();  // Track non-radiating time after trigger
      releaseTrigger();
    }
    else if(radiateScanTime == 0 && isRadiating())
    {
      releaseTrigger();
    }
    else if(radiateScanTime == -1 && !isRadiating())
    {
      holdTrigger();
    }
  
    // The releaseTrigger already scans the LCD, no need to do it again
    // but if the radiateScanTime is 0 or -1 special cases we need to
    if(radiateScanTime < 1)
    {
      scanLcd();
    }
  
    #ifdef INFO_ON
    if(isBatteryLow())
      if(Serial) Serial.println(F("Low Battery"));
    #endif
  
    // Decode the speed
    decodeLcdSpeed();
    
    // Perform additional pass if verify speed is enabled
    for(int x=0; x < verifySpeed && radiateScanTime > 0 && measuredSpeed > 0; x++)
    {
      #ifdef DEBUG_ON
      if(Serial) 
      {
        Serial.print(F("Peforming additional verification pass "));
        Serial.print(x+1);
        Serial.print(F(" of "));
        Serial.println(verifySpeed);
      }
      #endif

      // Pre-verification delay
      sDelay(verifyWait);
      
      radiateScanTimeElapsed = millis();  // Track radiating time after trigger
      holdTrigger();
      radiateScanTimeElapsed = millis()-radiateScanTimeElapsed;
          
      #ifdef DEBUG_SCAN_OFF_DELAY
      if(Serial) Serial.print(F("(verifySpeed) radiateScanTime = "));
      if(Serial) Serial.print(radiateScanTime);
      if(Serial) Serial.print(F("; radiateScanTimeElapsed = "));
      if(Serial) Serial.println(radiateScanTimeElapsed);
      #endif
      sDelay(radiateScanTime-radiateScanTimeElapsed);
      
      releaseTrigger();

      decodeLcdVerifySpeed();
    }
    eTimeEndVerifyLoop = millis()-eTimeStartMainLoop;
    
    #ifdef DEBUG_MAIN_LOOP_TIME
    if(Serial)
    {
      Serial.print(F("Scan + Verify eTime: "));
      Serial.print(eTimeEndVerifyLoop);
      Serial.println(F(" mS"));
    }
    #endif

    //print the speed
    printSpeed();
    
    // Wait before looping for next reading
    // Makes no sense if radar is manual on/off to wait between scans - so skip in those cases
    if(radiateScanTime > 0)
    {
      // Track non-radiating time after trigger
      radiateOffTimeElapsedLate = millis() - radiateOffTimeElapsedLate;
    
      #ifdef DEBUG_SCAN_OFF_DELAY
      if(Serial) Serial.print(F("radiateOffTime = "));
      if(Serial) Serial.print(radiateOffTime);
      if(Serial) Serial.print(F("; radiateOffTimeElapsedEarly = "));
      if(Serial) Serial.print(radiateOffTimeElapsedEarly);
      if(Serial) Serial.print(F("; radiateOffTimeElapsedLate  = "));
      if(Serial) Serial.println(radiateOffTimeElapsedLate);
      #endif
      
      // Calculate time we have spend doing non-radiating computation
      sDelay(radiateOffTime - radiateOffTimeElapsedEarly - radiateOffTimeElapsedLate);

      //Track non-radiating time before next trigger push
      radiateOffTimeElapsedEarly = millis();
    }
    
  } // end if: run radar
  else // else radar is not running
  {
    // Keep refreshing the in-memory state so
    // its up to date when interacted with
    scanLcd();
    decodeLcdSpeed();
  } // end if-else: if radar is not running

  #ifdef DEBUG_MAIN_LOOP_TIME
  if(Serial)
  {
    Serial.print(F("Main Loop eTime: "));
    Serial.print(millis()-eTimeStartMainLoop);
    Serial.println(F(" mS"));
  }
  #endif
  eTimeEndMainLoop = millis()-eTimeStartMainLoop;
} //end loop()



void buttonLcdSelfTest()
{
  // Leaving the if(Serial) on each print becasue we may want headless tests to run on start
  if(Serial) Serial.println(F("----------------------------------------"));
  if(Serial) Serial.println(F("Running control signal tests . . ."));
  if(isPowerOn())
  {
    if(Serial) Serial.println(F("WARNING: Power is already on, may cause unexpected results!"));
  }
  boolean passing = true;
  delay(5000);
  if(Serial) Serial.println(F("Testing power-on signal . . ."));
  powerOn();
  if(Serial) Serial.println(F("Done."));
  passing = passing && isPowerOn;
  delay(5000);
  if(Serial) Serial.println(F("Testing hold-trigger signal . . ."));
  holdTrigger();
  if(Serial) Serial.println(F("Done."));
  passing = passing && isRadiating();
  delay(5000);
  if(Serial) Serial.println(F("Testing release-trigger signal . . ."));
  releaseTrigger();
  if(Serial) Serial.println(F("Done."));
  passing = passing && !isRadiating();
  delay(5000);
  if(Serial) Serial.println(F("Testing power-off signal . . ."));
  powerOff();
  if(Serial) Serial.println(F("Done."));
  passing = passing && !isPowerOn();

  if(Serial)
  {
    Serial.println();
    if(passing)
    {
      Serial.println(F("All tests passed."));
    }
    else
    {
      Serial.println(F("One or more tests FAILED!"));
    }
    delay(5000);
    Serial.println(F("----------------------------------------"));
  }
}



// Scans the LCD segments and updates stored matrix states
void scanLcd()
{  
  clearLcd();

  // Pause a moment in case something was just busy
  delay(LCD_SCAN_DELAY);
  
  unsigned long scanTimeStart = millis(); // timeout for scanning LCD

  while (millis() - scanTimeStart < LCD_SCAN_TIME) // jump in and scan the LCD!
  {
    for (int i = 7; i < 11; i++)  // sweep the common pins
    {
      if (analogRead(i) < 10) // LOW
      {
        for (int j = 0; j < 7; j++) // sweep the segments after we find an enabled common
        {
          if (analogRead(j) > 600) // HIGH
          {
            segment[i - 7][j] = 1; // set the segment to a 1
          } // end if: HIGH
        } // end for: sweep the segments after we find an enabled common
      } // end if: LOW
    } // end for: sweep the common pins
  } // end while: jump in and scan the LCD!
}



// Decodes an LCD segment digit, returning the value
// The "valid" parameter should be set to true at start of first digit and passed unchanged to subsequent decodes
// Updates "valid" parameter to false and returns 0 on invalid data
// Blank is assumed to be a valid numeric zero
int decodeLcdDigit(boolean &valid, int topLeft, int topCenter, int topRight, int middleCenter, int bottomLeft, int bottomCenter, int bottomRight)
{
  if (topLeft == 0 && topCenter == 0 && topRight == 0 && middleCenter == 0 && bottomLeft == 0 && bottomCenter == 0 && bottomRight == 0)
  {
    return 0; //blank digit is 0
  }
  else if (topLeft == 1 && topCenter == 1 && topRight == 1 && middleCenter == 0 && bottomLeft == 1 && bottomCenter == 1 && bottomRight == 1)
  {
    return 0;
  }
  else if (topLeft == 0 && topCenter == 0 && topRight == 1 && middleCenter == 0 && bottomLeft == 0 && bottomCenter == 0 && bottomRight == 1)
  {
    return 1;
  }
  else if (topLeft == 0 && topCenter == 1 && topRight == 1 && middleCenter == 1 && bottomLeft == 1 && bottomCenter == 1 && bottomRight == 0)
  {
    return 2;
  }
  else if (topLeft == 0 && topCenter == 1 && topRight == 1 && middleCenter == 1 && bottomLeft == 0 && bottomCenter == 1 && bottomRight == 1)
  {
    return 3;
  }
  else if (topLeft == 1 && topCenter == 0 && topRight == 1 && middleCenter == 1 && bottomLeft == 0 && bottomCenter == 0 && bottomRight == 1)
  {
    return 4;
  }
  else if (topLeft == 1 && topCenter == 1 && topRight == 0 && middleCenter == 1 && bottomLeft == 0 && bottomCenter == 1 && bottomRight == 1)
  {
    return 5;
  }
  else if (topLeft == 1 && topCenter == 1 && topRight == 0 && middleCenter == 1 && bottomLeft == 1 && bottomCenter == 1 && bottomRight == 1)
  {
    return 6;
  }
  else if (topLeft == 0 && topCenter == 1 && topRight == 1 && middleCenter == 0 && bottomLeft == 0 && bottomCenter == 0 && bottomRight == 1)
  {
    return 7;
  }
  else if (topLeft == 1 && topCenter == 1 && topRight == 1 && middleCenter == 1 && bottomLeft == 1 && bottomCenter == 1 && bottomRight == 1)
  {
    return 8;
  }
  else if (topLeft == 1 && topCenter == 1 && topRight == 1 && middleCenter == 1 && bottomLeft == 0 && bottomCenter == 1 && bottomRight == 1)
  {
    return 9;
  }
  else   // Should never happen - means an invalid combination of segments were turned on
  {
    valid = false;
    return 0;
  }
}



// Decode the LCD matrix into speed integer
void decodeLcdSpeed()
{
  decodeLcdSpeed(false);
}

// Decode the LCD matrix into speed integer
void decodeLcdVerifySpeed()
{
  decodeLcdSpeed(true);
}

// Decode the LCD matrix into speed integer
// Parameter sets whether we are storing in verified speed or calculating initial speed
void decodeLcdSpeed(boolean isVerifying)
{ 
  int decodedSpeed = 0; // clear the speed, and we'll set it now based on the segment data
  boolean decodedSpeedValid = true; // assume we will be successful

  // ONES POSITION
  //     3,5
  // 3,4     2,5
  //     2,4
  // 1,4     1,5
  //     0,5
  //int decodeLcdDigit(          boolean &valid    , int topLeft  , int topCenter, int topRight , int middleCenter, int bottomLeft, int bottomCenter, int bottomRight)
  decodedSpeed = decodeLcdDigit(decodedSpeedValid, segment[3][4], segment[3][5], segment[2][5], segment[2][4]   , segment[1][4] , segment[0][5]   ,  segment[1][5]);

  // TENS POSITION
  //     3,3
  // 3,2     2,3
  //     2,2
  // 1,2     1,3
  //     0,3
  //int decodeLcdDigit(                 boolean &valid    , int topLeft  , int topCenter, int topRight , int middleCenter, int bottomLeft, int bottomCenter, int bottomRight)
  decodedSpeed += (10 * decodeLcdDigit(decodedSpeedValid, segment[3][2], segment[3][3], segment[2][3], segment[2][2]   , segment[1][2] , segment[0][3]   , segment[1][3]) );

  // HUNDREDS
  //     3,1
  // 3,0     2,1
  //     2,0
  // 1,0     1,1
  //     0,1
  //int decodeLcdDigit(                  boolean &valid    , int topLeft  , int topCenter, int topRight , int middleCenter, int bottomLeft, int bottomCenter, int bottomRight)
  decodedSpeed += (100 * decodeLcdDigit(decodedSpeedValid, segment[3][0], segment[3][1], segment[2][1], segment[2][0]   , segment[1][0] , segment[0][1]   ,  segment[1][1]) );

  if(isMph() == isKph()) // Must be one or the other; if neither or both we have bad data
  {
    decodedSpeedValid = false;
  }

  if(isVerifying)
  {
    // Mostly for readability
    int verifyingMeasuredSpeed = decodedSpeed;      // Store decoded speed
    boolean verifyingSpeedValid = decodedSpeedValid; // Store if decode is valid
    
    #ifdef DEBUG_LCD_DECODE
    if(Serial)
    {
      Serial.print(F("decodeLcdSpeed - verifying: decodedSpeedValid (verifyingSpeedValid)="));
      Serial.print(verifyingSpeedValid);
      Serial.print(F("; measuredSpeedValid="));
      Serial.print(measuredSpeedValid);
      Serial.print(F("; measuredSpeed="));
      Serial.print(measuredSpeed);
      Serial.print(F("; decodedSpeed (verifyingMeasuredSpeed) ="));
      Serial.println(verifyingMeasuredSpeed);
    }
    #endif
    
    if(verifyingSpeedValid && measuredSpeedValid && measuredSpeed > 0)
    {
      int newVerifiedMeasuredSpeedPercent = round(  100.0 * (1.0-( ((double)abs(measuredSpeed-verifyingMeasuredSpeed))/((double)measuredSpeed) ))  );

      // Rare case second speed is crazy higher than first, math falls apart.  Cap at 0% confidence
      if(newVerifiedMeasuredSpeedPercent < 0)
        newVerifiedMeasuredSpeedPercent = 0;

      // If the % verified is between 0 and 100; and also previously measured speed is higher than new decoded (verifying) speed, make negative so we can tell
      if(newVerifiedMeasuredSpeedPercent > 0 && newVerifiedMeasuredSpeedPercent < 100 && measuredSpeed > verifyingMeasuredSpeed)
        newVerifiedMeasuredSpeedPercent*= -1;

      //If its the first verification reading, or worse error than existing, store it
      if(verifiedMeasuredSpeedPercent == -999 || (abs(newVerifiedMeasuredSpeedPercent) < abs(verifiedMeasuredSpeedPercent)) )
        verifiedMeasuredSpeedPercent = newVerifiedMeasuredSpeedPercent;
    }
    else
    {
      verifiedMeasuredSpeedPercent = -999;
    }
    
    #ifdef DEBUG_LCD_DECODE
    if(Serial)
    {
      Serial.print(F("decodeLcdSpeed - verifying computed: verifiedMeasuredSpeedPercent="));
      Serial.println(verifiedMeasuredSpeedPercent);
    }
    #endif
  }
  else //not verifying, do main processing
  {
    #ifdef DEBUG_LCD_DECODE
    if(Serial) Serial.println(F("decodeLcdSpeed - processing measurement"));
    #endif
  
    measuredSpeed = decodedSpeed;           // Store decoded speed
    measuredSpeedValid = decodedSpeedValid; // Store if decode is valid
    verifiedMeasuredSpeedPercent = -999;    // Reset verified to invalid (we are not verifying this pass)
  
    // Should never happen - means an invalid combination of segments were turned on
    if(!measuredSpeedValid)
    {
      failedDecodeCount++;
    }
  
    if(offsetAngle > 0 && measuredSpeedValid)
    {
      #ifdef DEBUG_LCD_DECODE
      if(Serial) Serial.println(F("decodeLcdSpeed - processing offset angle"));
      #endif
    
      //Convert degree to rad
      double offsetAngleRad = (offsetAngle * 1000.0) / 57296.0;
      actualSpeed=round(  ((double)measuredSpeed)/cos(offsetAngleRad)  );
    }
    else
    {
      actualSpeed=measuredSpeed;
    }
  } // end else
}



// Print speed if different from previously stored
void printSpeed()
{
  if(measuredSpeedValid) // only consider print speed if valid
  {    
    if (  ( (oldSpeed != measuredSpeed)        ||  // only print if its changed since last print OR
            (printFirst && firstMeasurement) ) &&  // first measurement pulling trigger
                                                   // AND
            (printZero || actualSpeed > 0)     )   // we either want printing of zeros or its non-zero
    {
      if(Serial)
      {
        Serial.print(F("Speed "));
        Serial.print(actualSpeed);
        Serial.print( isMph() ? F(" MPH") : (isKph() ? F(" KPH") : F(" ???")));

        // If angle was compensating, print raw speed too
        if(offsetAngle > 0)
        {
          Serial.print(F(" (Measured "));
          Serial.print(measuredSpeed);
          Serial.print(F(" @ "));
          Serial.print(offsetAngle);
          Serial.print(F(" degrees offset)"));
        }

        if(verifiedMeasuredSpeedPercent > -999)
        {
          Serial.print(F(" verified at "));
          if(verifiedMeasuredSpeedPercent > 0 && verifiedMeasuredSpeedPercent < 100)
            Serial.print("+");
          Serial.print(verifiedMeasuredSpeedPercent);
          Serial.print(F(" %"));
        }

      // Always print newline after readings
      Serial.println();
      
      } // end if: serial
      
      // Set flag it has been printed
      firstMeasurement = false;
      
    } // end if: should be printed
    
    oldSpeed = measuredSpeed;

  } // end if: valid
  
  #if defined DEBUG_ON || defined DEBUG_INVALID_LCD
  else
  {
    if(Serial) Serial.println(F("Invalid measurement decode"));
    dumpLcd();
    if(Serial)
    {
      Serial.print(F("Total failed decodes: "));
      Serial.println(failedDecodeCount);
    }
  }
  #endif
}



// Clear the LCD matrix
void clearLcd()
{
  // clearing segment data
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 7; j++)
    {
      segment[i][j] = 0;
    }
  }
}



// Debugging - print out the detected LCD segments raw
void dumpLcd()
{
  #ifdef DEBUG_ON
  for (int i = 0; i < 4; i++)
  {
    if(Serial) Serial.print(i);
    if(Serial) Serial.print(F(": "));
    for (int j = 0; j < 7; j++)
    {
      if(Serial) Serial.print(segment[i][j]);
      if(j < 6)
      {
        if(Serial) Serial.print(F(","));
      }
    }
    if(Serial) Serial.println();
  }
  #endif

/*                /*              /*
 * HUNDREDS        * TENS POSITION *  ONES POSITION
 *                 *               */
  //     3,1      //     3,3      //     3,5
  // 3,0     2,1  // 3,2     2,3  // 3,4     2,5
  //     2,0      //     2,2      //     2,4
  // 1,0     1,1  // 1,2     1,3  // 1,4     1,5
  //     0,1      //     0,3      //     0,5
  if(Serial)
  {
    Serial.println();
  
    Serial.print(F(" "));Serial.print( segment[3][1] );Serial.print(F(" "));           Serial.print(F("   "));   Serial.print(F(" "));Serial.print( segment[3][3] );Serial.print(F(" "));            Serial.print(F("   "));  Serial.print(F(" "));Serial.print( segment[3][5] );Serial.println(F(" "));
    
    Serial.print( segment[3][0] );Serial.print(F(" "));Serial.print( segment[2][1] );  Serial.print(F("   "));   Serial.print( segment[3][2] );Serial.print(F(" "));Serial.print( segment[2][3] );   Serial.print(F("   "));  Serial.print( segment[3][4] );Serial.print(F(" "));Serial.println( segment[2][5] );
    
    Serial.print(F(" "));Serial.print( segment[2][0] );Serial.print(F(" "));           Serial.print(F("   "));   Serial.print(F(" "));Serial.print( segment[2][2] );Serial.print(F(" "));            Serial.print(F("   "));  Serial.print(F(" "));Serial.print( segment[2][4] );Serial.println(F(" "));
    
    Serial.print( segment[1][0] );Serial.print(F(" "));Serial.print( segment[1][1] );  Serial.print(F("   "));   Serial.print( segment[1][2] );Serial.print(F(" "));Serial.print( segment[1][3] );   Serial.print(F("   "));  Serial.print( segment[1][4] );Serial.print(F(" "));Serial.println( segment[1][5] ); 
    
    Serial.print(F(" "));Serial.print( segment[0][1] );Serial.print(F(" "));           Serial.print(F(" . "));   Serial.print(F(" "));Serial.print( segment[0][3] );Serial.print(F(" "));            Serial.print(F(" . "));  Serial.print(F(" "));Serial.print( segment[0][5] );Serial.println(F(" "));
  
    Serial.println();
    Serial.print(F("MPH  = "));Serial.println(segment[2][6]);
    Serial.print(F("KPH  = "));Serial.println(segment[1][6]);
    Serial.print(F("BATT = "));Serial.println(segment[0][6]);
    Serial.print(F("RDR  = "));Serial.println(segment[3][6]);
    Serial.println();
  }
}



// Returns whether the LCD test passed (all segments on)
boolean verifyLcdTest()
{
  int sum=0;
  for(int x=0; x < 4; x++)
  {
    for(int y=0; y < 7; y++)
    {
      sum+=segment[x][y];
    }
  }

  #ifdef DEBUG_ON
  if(Serial)
  {
    Serial.print(F("LCD Self Test Count = "));
    Serial.println(sum);
  }
  #endif

  return sum == 25; // Should be all 25 segments "on"
}



// LCD Data
boolean isBatteryLow()
{
  return segment[0][6];
}

// LCD Data
boolean isKph()
{
  return segment[1][6];
}

// LCD Data
boolean isMph()
{
  return segment[2][6];
}

// LCD Data
boolean isRadiating()
{
  return segment[3][6];
}

// Is the power source USB
boolean isUsbPower()
{
  return !digitalRead(USB_POWER_PIN);
}



// Is the radar on (based on LCD Data)
boolean isPowerOn()
{
  scanLcd();
  // Check the LCD - if its on one of these should be illuminated
  return isKph() || isMph();
}



// Send command to radar (internal use only)
void toggleMphKph()
{
  holdTrigger();
  delay(100);
  holdPowerButton();
  delay(100);
  releasePowerButton();
  delay(100);
  releaseTrigger();
  delay(100);
}

// Send command to radar
// parameter is global defined MPH or KPH
//Units references
#define MPH  1
#define KPH  2
void changeUnits(int units)
{
  if(isPowerOn())
  {

    if( (isMph() && units == MPH) || (isKph() && units == KPH) )
    {
      #ifdef INFO_ON
      if(Serial) Serial.println(F("Skipping: changeUnits - Units are already correct"));
      #endif
    }
    else
    {
      #ifdef INFO_ON
      if(Serial) Serial.println(F("Sending: toggleMphKph"));
      #endif
      
      toggleMphKph();
    }
  }
  
  scanLcd();

  // Verify units
  #ifdef INFO_ON
  if(Serial) Serial.println( (isMph() && units == MPH) || (isKph() && units == KPH) ? F("changeUnits: OK") : F("changeToMph: FAIL") );
  #endif
}



// Send command to radar (internal use only)
void holdPowerButton()
{
  #ifdef DEBUG_PBUTTON_ON
  if(Serial) Serial.println(F("Sending: holdPowerButton"));
  #endif

  // Set it as an output to pull it hard low
  // Real life these push together under one rubber cap
  pinMode(POWER_ON_PIN,OUTPUT);
  pinMode(POWER_OFF_PIN,OUTPUT);

  #ifdef DEBUG_PBUTTON_ON
  if(Serial) Serial.println(F("Done: holdPowerButton"));
  #endif
}

// Send command to radar (internal use only)
void releasePowerButton()
{
  #ifdef DEBUG_PBUTTON_ON
  if(Serial) Serial.println(F("Sending: releasePowerButton"));
  #endif
  
  // Set it as an input to let it float
  // Real life these push together under one rubber cap
  pinMode(POWER_ON_PIN,INPUT);
  pinMode(POWER_OFF_PIN,INPUT);

  #ifdef DEBUG_PBUTTON_ON
  if(Serial) Serial.println(F("Done: releasePowerButton"));
  #endif
}

// Send command to radar
void powerOn()
{
  // Check if we have a repeated power-on failure, attempt recovery
  if(powerOnFailCount >= 3)
  {
    #ifdef INFO_ON
    if(Serial) Serial.println(F("ERROR: Repeated powerOn failure detected!  Attempting recovery..."));
    #endif
    
    //Try to turn off
    powerOff();

    //Check power button value and wait with timeout
    int desiredWait = 10 * powerOnFailCount;

    while(analogRead(POWER_OFF_SENSE_PIN) > 100 && desiredWait > 0)
    {
      unsigned long sTime = millis();
      #ifdef INFO_ON
      if(Serial)
      {
        Serial.print(F("Recovering: Waiting up to "));
        Serial.print(desiredWait);
        Serial.println(F(" seconds..."));
      }
      #endif

      sDelay(10000);
      desiredWait-=10;
    }
  }
  
  #ifdef INFO_ON
  if(Serial) Serial.println(F("Sending: powerOn"));
  #endif

  releaseTrigger();
  delay(100);
  
  holdPowerButton();
  delay(100);
  releasePowerButton();
  delay(500);
  scanLcd();

  boolean lcdTestResultPass = verifyLcdTest();
  #ifdef DEBUG_ON
  // Verify display self-test
  if(Serial) Serial.println( lcdTestResultPass ? F("lcdTest: OK") : F("lcdTest: FAIL"));
  #elif defined INFO_ON
  if(Serial && !lcdTestResultPass) Serial.println(F("WARNING: LCD Power-On test FAIL!"));
  #endif
  

  // Wait for self test to clear
  delay(1000 + (100*powerOnFailCount));
  
  scanLcd();

  // Track if we have failed to power on
  if(lcdTestResultPass && isPowerOn())
  {
    powerOnFailCount=0;
  }
  else
  {
    powerOnFailCount++;
  }

  // Verify power
  #ifdef INFO_ON
  if(Serial)
  {
    Serial.println( isPowerOn() ? F("powerOn: OK") : F("powerOn: FAIL"));
    if(powerOnFailCount > 0)
    {
      Serial.print(F("WARNING: Power on check failed "));
      Serial.print(powerOnFailCount);
      Serial.println(F(" times."));
    }
  }
  #endif
}

// Send command to radar
void powerOff()
{
  #ifdef INFO_ON
  if(Serial) Serial.println(F("Sending: powerOff"));
  #endif

  releaseTrigger();
  
  // Set it as an output to pull it hard low
  holdPowerButton();
  delay(3100); //wait 3 sec countdown plus a bit
  // Set it as an input to let it float
  releasePowerButton();
  //Delay for voltage to stabilize
  delay(500);

  // Maybe check power off pin voltage and wait a bit
  // Unsure why this sometimes happens
  // For now this is part of the "recovery" logic when powering on
  
  scanLcd();
  
  // Verify power
  #ifdef INFO_ON
  if(Serial) Serial.println( isPowerOn() ? F("powerOff: FAIL") : F("powerOff: OK"));
  #endif
}



// Send command to radar
void holdTrigger()
{
  #ifdef DEBUG_TRIGGER_ON
  if(Serial) Serial.println(F("Sending: holdTrigger"));
  #endif
  
  // Set it as an output to pull it hard low
  pinMode(TRIGGER_PIN,OUTPUT);

  scanLcd();

  firstMeasurement = true;

  // Verify radiating
  #ifdef DEBUG_TRIGGER_ON
  if(Serial) Serial.println( isRadiating() ? F("holdTrigger: OK") : F("holdTrigger: FAIL"));
  #endif
}

// Send command to radar
void releaseTrigger()
{
  #ifdef DEBUG_TRIGGER_ON
  if(Serial) Serial.println(F("Sending: releaseTrigger"));
  #endif
  
  // Set it as an output to pull it hard low
  pinMode(TRIGGER_PIN,INPUT);
  
  scanLcd();

  // Verify not radiating
  #ifdef DEBUG_TRIGGER_ON
  if(Serial) Serial.println( isRadiating() ? F("releaseTrigger: FAIL") : F("releaseTrigger: OK"));
  #endif
}



// put Arduino board to sleep mode
void goToSleep()
{
  #ifdef DEBUG_LED_PIN
  flashDebugLED(1000,5);
  #endif

  #ifndef DEBUG_NEVER_SLEEP
  
  enableInterrupt();       // enable the interrupt pin so it can wake back up
  
  sleep_enable();          // enables the sleep bit in the mcucr register
                           // so sleep is possible. just a safety pin

  sleep_mode();            // here the device is actually put to sleep!!
                           // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

  sleep_disable();         // first thing after waking from sleep:
                           // disable sleep...

  disableInterrupt();      // disable the interrupt because it interferes with USB

  #endif

  #ifdef DEBUG_LED_PIN
  flashDebugLED(100,50);
  #endif
}

#ifdef DEBUG_SLEEP_FOREVER
// Allows forcing to sleep with no interrupt to wake - for testing only
void sleepForever()
{
  sleep_enable();          // enables the sleep bit in the mcucr register
                           // so sleep is possible. just a safety pin

  sleep_mode();            // here the device is actually put to sleep!!
                           // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
}
#endif

// Code run when the interrupt fires
void doInterrupt()
{
  // We don't actually need to do anything here, its only used to wake from sleep
}

// disable interrupt so we don't get stuck
boolean interruptEnabled = false;
void disableInterrupt()
{
  if(interruptEnabled)
  {
    detachInterrupt(digitalPinToInterrupt(USB_POWER_PIN));
    interruptEnabled=false;
  }
}

// enable interrupt so USB Connect is picked up
void enableInterrupt()
{
  if(!interruptEnabled)
  {
    // use interrupt and run function
    attachInterrupt(digitalPinToInterrupt(USB_POWER_PIN), doInterrupt, LOW);
    interruptEnabled=true;
  }
}



// Returns the ideal loop time (if no over-run)
long getTargetLoopTime()
{
  return radiateScanTime + radiateOffTime;
}

// Returns the projected loop time (with over-run)
long getProjectedLoopTime()
{
  if(getScanPlusVerifyTime() > getTargetLoopTime())
    return getScanPlusVerifyTime();
  else
    return getTargetLoopTime();
}

// Returns the projected scan+verify time
long getScanPlusVerifyTime()
{
  // Some of these values are guesswork for "extra processing" in the code combined with how many times the LCD scan code is called
  return (radiateScanTime + (2*(LCD_SCAN_DELAY + LCD_SCAN_TIME))) + (verifySpeed * (verifyWait + radiateScanTime + (1*(LCD_SCAN_DELAY + LCD_SCAN_TIME))));
}

// Returns the projected worst case duty cycle
int getProjectedDutyCycle()
{
  long totalRadiateTime = radiateScanTime + (verifySpeed * radiateScanTime);
  
  return (totalRadiateTime*100)/getProjectedLoopTime();
}



// print expected serial command syntax
void printSerialCommands()
{
  if(Serial)
  {
    //                    NOTE - First letter of command must be unique
    //                           because it is used for switch statment
    //                    NOTE - Sort order groups functionally, not alphabetical!
    Serial.println(F("##############################################################"));
    Serial.println(F("#                                                            #"));
    Serial.println(F("#         [CMD]    ARG Action                                #"));
    Serial.println(F("#         -------- --- ------------------------------------  #"));
    Serial.println(F("#  Info                                                      #"));
    Serial.println(F("#         [H]ELP      : Help Menu (this menu)                #"));
    Serial.println(F("#         [E]TIME     : eTime Metrics For Main Loop (msec)   #"));
    Serial.println(F("#         [D]IAG      : Diagnostic self-test Buttons/LCD     #"));
    Serial.println(F("#  Action                                                    #"));
    Serial.println(F("#         [T]RIG    # : Ctrl Trigger       1=Hold 0=Release  #"));
    Serial.println(F("#         [P]OWER   # : Ctrl Power Button  1=On   0=Off      #"));
    Serial.println(F("#         [M]EASURE   : Poll Speed Measurement (if changed)  #"));
    Serial.println(F("#         [U]NITS   # : Set Units (1)=MPH; (2)=KPH           #"));
    Serial.println(F("#                       (cycle power to save persistant)     #"));
    Serial.println(F("#         [R]EFRESH   : Poll LCD Scan and print LCD data     #"));
    Serial.println(F("#                       Note: This prints in code style      #"));
    Serial.println(F("#                       Example: (-1 speed = invalid)        #"));
    Serial.println(F("#                         isPowerOn=1                        #"));
    Serial.println(F("#                         isBatteryLow=0                     #"));
    Serial.println(F("#                         isMph=1                            #"));
    Serial.println(F("#                         isKph=0                            #"));
    Serial.println(F("#                         isRadiating=0                      #"));
    Serial.println(F("#                         measuredSpeed=0                    #"));
    Serial.println(F("#  Config                                                    #"));
    Serial.println(F("#         [G]ET       : Print current config values          #"));
    Serial.println(F("#         [S]CAN    # : Set Radiating Scan Time (long msec)  #"));
    Serial.println(F("#                       (0)=never radiate; (-1)=always on    #"));
    Serial.println(F("#         [O]FF     # : Set Radiating Off Time  (long msec)  #"));
    Serial.println(F("#                       NOTE: Ignored if SCAN = 1 or 0       #"));
    Serial.println(F("#         [C]ORR    # : Correction for offset angle (0-89)   #"));
    Serial.println(F("#                       NOTE: Init causes USB re-detect      #"));
    Serial.println(F("#         [V]ERIFY  # : Verify measured speed (int x) 0=Off  #"));
    Serial.println(F("#                       Perform # add'l scans, runs during   #"));
    Serial.println(F("#                       'off' time, may affect duty cycle    #"));
    Serial.println(F("#         [W]AIT    # : Pre-Verify Wait time (long msec)     #"));
    Serial.println(F("#         [A]UTO    # : Radar auto-processing   1=On 0=Off   #"));
    Serial.println(F("#         [F]IRST   # : Print 1st reading always 1=On 0=Off  #"));
    Serial.println(F("#         [Z]ERO    # : Print zero-speed scans   1=On 0=Off  #"));
    Serial.println(F("#         [I]NIT   -9 : Init & Reboot (arg = -9 to confirm)  #"));
    Serial.println(F("#                                                            #"));
    Serial.println(F("##############################################################"));
  }
}



// read and process incoming serial commands
void readSerialCommands()
{
  if(Serial.available())
  {
    // Read until newline
    String input = Serial.readStringUntil('\n');

    // Convert to upper-case (we don't want case-sensitive)
    input.toUpperCase();

    #ifdef DEBUG_SERIAL_INPUT_ON
    if(Serial)
    {
      Serial.print(F("Input: \""));
      Serial.print(input);
      Serial.println(F("\""));
    }
    #endif

    // If we got non-zero input, process it
    if(input.length() > 0)
    {
      // Extract first characater (command key)
      char serialCommand = input[0];

      // Storage for numeric argument (-999 is invalid)
      long serialCommandArg = -999;

      // Search for numeric arguments
      int searchIndex = 0;
      int scalor = 1; // positive/negative scalor
      while(searchIndex < input.length() && (input[searchIndex] < '0' || input[searchIndex] > '9'))
      {
        #ifdef DEBUG_SERIAL_INPUT_ON
        if(Serial)
        {
            Serial.print(F("L1 searchIndex="));
            Serial.println(searchIndex);
        }
        #endif
        
        searchIndex++;
      }

      // Check if its negative
      if(searchIndex > 0 && input[searchIndex-1] == '-')
      {
        scalor = -1;

        #ifdef DEBUG_SERIAL_INPUT_ON
        if(Serial)
        {
            Serial.print(F("Found negative '-' char "));
            Serial.println(searchIndex);
        }
        #endif
      }

      // Extract numeric value
      while(searchIndex < input.length() && (input[searchIndex] >= '0' && input[searchIndex] <= '9'))
      {
        if(serialCommandArg == -999)
        {
          serialCommandArg=0;            // Set to zero if first valid value
        }

        serialCommandArg*=10;             // Shift place value
        serialCommandArg+=(input[searchIndex]-'0'); //Add digit

        #ifdef DEBUG_SERIAL_INPUT_ON
        if(Serial)
        {
            Serial.print(F("L2 searchIndex="));
            Serial.println(searchIndex);
        }
        #endif

        searchIndex++;
      }

      // Make negative if applicable
      serialCommandArg*=scalor;

      #ifdef DEBUG_SERIAL_INPUT_ON
      if(Serial)
      {
          Serial.print(F("Decoded serialCommandArg="));
          Serial.println(serialCommandArg);
      }
      #endif

      switch (serialCommand)
      {
        case 'H': // HELP - Help Menu
          printSerialCommands();
          break;

        case 'E': // ETIME - Show eTime Metrics For Main Loop
          if(Serial)
          {
            Serial.print(F("INFO ETIME : Scan + Verifying = "));
            Serial.print(eTimeEndVerifyLoop);
            Serial.print(F("mS  (expected "));
            Serial.print(getScanPlusVerifyTime());
            Serial.println(F("mS)"));
            
            Serial.print(F("INFO ETIME : Total Loop Time  = "));
            Serial.print(eTimeEndMainLoop);
            Serial.print(F("mS  (expected "));
            Serial.print(getProjectedLoopTime());
            Serial.println(F("mS)"));

            Serial.print(F("INFO ETIME : Projected Duty Cycle  = "));
            Serial.print(getProjectedDutyCycle());
            Serial.println(F("%"));
          }
          break;

        case 'D': // DIAG - Diagnostic self-test Buttons/LCD
          if(Serial) Serial.println(F("INFO DIAG - Preparing to perform self-tests..."));
          powerOff();
          if(Serial) Serial.println(F("INFO DIAG - Starting self-tests..."));
          buttonLcdSelfTest();
          if(Serial) Serial.println(F("INFO DIAG - Completed self-tests, resuming normal run."));
          break;
          
        case 'T': // TRIG # - Ctrl Trigger 1=Hold 0=Release
          if(serialCommandArg == 1)
          {
            holdTrigger();
          }
          else if(serialCommandArg == 0)
          {
            releaseTrigger();
          }
          else
          {
            if(Serial) Serial.println(F("ERROR: Invalid command.  Try \"HELP\""));
          }

          #ifdef INFO_SERIAL_INPUT_ON
          if(Serial)
          {
            Serial.print(F("ACTION TRIG = "));
            Serial.println(isRadiating());
          }
          #endif
          break;
          
        case 'P': // POWER # - Ctrl Power Button 1=On   0=Off
          if(serialCommandArg == 1)
          {
            if(!isPowerOn())
              powerOn();
          }
          else if(serialCommandArg == 0)
          {
            if(isPowerOn())
              powerOff();
          }
          else
          {
            if(Serial) Serial.println(F("ERROR: Invalid command.  Try \"HELP\""));
          }

          #ifdef INFO_SERIAL_INPUT_ON
          Serial.print(F("ACTION POWER = "));
          Serial.println(isPowerOn());
          #endif
          break;
          
        case 'M': // Measure - Ctrl Poll Speed Measurement (if changed)
          scanLcd();
          decodeLcdSpeed();
          printSpeed();  
          break;
          
        case 'U': // Units - Set Units (1)=MPH; (2)=KPH
          if(serialCommandArg == 1 || serialCommandArg == 2)
          {
            changeUnits(serialCommandArg);
          }
          else
          {
            if(Serial) Serial.println(F("ERROR: Invalid command.  Try \"HELP\""));
          }

          #ifdef INFO_SERIAL_INPUT_ON
          Serial.print(F("ACTION UNITS = "));
          Serial.println(isMph() ? F(" MPH") : (isKph() ? F(" KPH") : F(" ???")));
          #endif
          break;
          
        case 'R': // REFRESH - Poll LCD Scan and print LCD data
          decodeLcdSpeed();

          if(Serial)
          {
            Serial.print(F("isPowerOn="));
            Serial.println(isPowerOn());
            Serial.print(F("isBatteryLow="));
            Serial.println(isBatteryLow());
            Serial.print(F("isMph="));
            Serial.println(isMph());
            Serial.print(F("isKph="));
            Serial.println(isKph());
            Serial.print(F("isRadiating="));
            Serial.println(isRadiating());
            Serial.print(F("measuredSpeed="));
            Serial.println(measuredSpeedValid ? measuredSpeed : -1);
          }
          break;
          
        case 'G': // GET - Print current config values
          if(Serial)
          {
            
            Serial.print(F("CONFIG SCAN   = "));
            Serial.println(radiateScanTime);
            
            Serial.print(F("CONFIG OFF    = "));
            Serial.println(radiateOffTime);

            Serial.print(F("CONFIG CORR   = "));
            Serial.println(offsetAngle);
            
            Serial.print(F("CONFIG VERIFY = "));
            Serial.println(verifySpeed);

            Serial.print(F("CONFIG WAIT   = "));
            Serial.println(verifyWait);

            Serial.print(F("CONFIG AUTO   = "));
            Serial.println(autoRunRadar);
            
            Serial.print(F("CONFIG FIRST  = "));
            Serial.println(printFirst);
            
            Serial.print(F("CONFIG ZERO   = "));
            Serial.println(printZero);
          }
          break;
          
        case 'S': // SCAN - Set Radiating Scan Time (long msec) (0)=never radiate; (-1)=always on
          if(serialCommandArg >= -1)
          {
            radiateScanTime=serialCommandArg;
          }
          else
          {
            if(Serial) Serial.println(F("ERROR: Invalid command.  Try \"HELP\""));
          }

          #ifdef INFO_SERIAL_INPUT_ON
          if(Serial)
          {
            Serial.print(F("CONFIG SCAN   = "));
            Serial.println(radiateScanTime);
          }
          #endif
          break;
          
        case 'O': // OFF - Set Radiating Off Time  (long msec)
          if(serialCommandArg >= 0)
          {
            radiateOffTime=serialCommandArg;
          }
          else
          {
            if(Serial) Serial.println(F("ERROR: Invalid command.  Try \"HELP\""));
          }

          #ifdef INFO_SERIAL_INPUT_ON
          if(Serial)
          {
            Serial.print(F("CONFIG OFF    = "));
            Serial.println(radiateOffTime);
          }
          #endif
          break;

        case 'C': // CORR - Set Correction for offset angle (0-89)
        if(serialCommandArg >= 0 && serialCommandArg < 90)
          {
            offsetAngle=serialCommandArg;
          }
          else
          {
            if(Serial) Serial.println(F("ERROR: Invalid command.  Try \"HELP\""));
          }

          #ifdef INFO_SERIAL_INPUT_ON
          if(Serial)
          {
            Serial.print(F("CONFIG CORR   = "));
            Serial.println(offsetAngle);
          }
          #endif
          break;
                  
        case 'V': // Verify measured speed (int x) 0=Off
          if(serialCommandArg >= 0)
          {
            verifySpeed=serialCommandArg;
          }
          else
          {
            if(Serial)
            {
              Serial.println(F("ERROR: Invalid command.  Try \"HELP\""));
              Serial.println(F("NOTE:  Don't be absurd doing too many scans!"));
              Serial.println(F("NOTE:  (max 1500 feet range) / (min 10 miles per hour) = 102.272727 seconds in range"));
            }
          }

          #ifdef INFO_SERIAL_INPUT_ON
          if(Serial)
          {
            Serial.print(F("CONFIG VERIFY = "));
            Serial.println(verifySpeed);
          }
          #endif
          break;
          
        case 'W': // Pre-Verify Delay time (long msec)
          if(serialCommandArg >= 0)
          {
            verifyWait=serialCommandArg;
          }
          else
          {
            if(Serial) Serial.println(F("ERROR: Invalid command.  Try \"HELP\""));
          }

          #ifdef INFO_SERIAL_INPUT_ON
          if(Serial)
          {
            Serial.print(F("CONFIG WAIT   = "));
            Serial.println(verifyWait);
          }
          #endif
          break;
          
        case 'A': // AUTO - Radar auto-processing 1=On 0=Off
          if(serialCommandArg == 0 || serialCommandArg == 1)
          {
            autoRunRadar=serialCommandArg;
          }
          else
          {
            if(Serial) Serial.println(F("ERROR: Invalid command.  Try \"HELP\""));
          }

          #ifdef INFO_SERIAL_INPUT_ON
          if(Serial)
          {
            Serial.print(F("CONFIG AUTO   = "));
            Serial.println(autoRunRadar);
          }
          #endif
          break;

        case 'F': // FIRST - Print 1st reading always 1=On 0=Off
          if(serialCommandArg == 0 || serialCommandArg == 1)
          {
            printFirst=serialCommandArg;
          }
          else
          {
            if(Serial) Serial.println(F("ERROR: Invalid command.  Try \"HELP\""));
          }

          #ifdef INFO_SERIAL_INPUT_ON
          if(Serial)
          {
            Serial.print(F("CONFIG FIRST  = "));
            Serial.println(printFirst);
          }
          #endif
          break;
          
        case 'Z': // ZERO - Print zero-speed scans 1=On 0=Off
          if(serialCommandArg == 0 || serialCommandArg == 1)
          {
            printZero=serialCommandArg;
          }
          else
          {
            if(Serial) Serial.println(F("ERROR: Invalid command.  Try \"HELP\""));
          }

          #ifdef INFO_SERIAL_INPUT_ON
          if(Serial)
          {
            Serial.print(F("CONFIG ZERO   = "));
            Serial.println(printZero);
          }
          #endif
          break;

        case 'I': // Init - Init & Reboot (arg = -9 to confirm)
          if(serialCommandArg >= -9)
          {
            if(Serial) Serial.println(F("CONFIG INIT - Initializing and rebooting, please wait . . ."));
            reboot();
          }
          else
          {
            if(Serial)
            {
              Serial.print(F("CONFIG INIT - Ignore.  Confirmation got "));
              Serial.print(serialCommandArg);
              Serial.println(F(" but expecting -9."));
              Serial.println(F("ERROR: Invalid command.  Try \"HELP\""));
            }
          }
          break;
          
        default:
          if(Serial) Serial.println(F("ERROR: Invalid command.  Try \"HELP\""));
          break;
          
      } // end switch/case

      //Print warning if timings look bad
      long remainderTime = getTargetLoopTime() - getProjectedLoopTime();
      if(remainderTime < 0)
      {
        if(Serial)
        {
          Serial.print(F("WARNING: Loop timing looks bad!  Projected extra time in 'Off' loop is "));
          Serial.print(remainderTime);
          Serial.println(F(" mS"));
        }
      }
      
      //Print warning if duty cycle is high
      int dutyCycle = getProjectedDutyCycle();
      if(dutyCycle > 50)
      {
        if(Serial)
        {
          Serial.print(F("WARNING: Duty cycle looks high at "));
          Serial.print(dutyCycle);
          Serial.println(F("%"));
        }
      }
      
    } // end if: input length >1
  }// end if: serial
}// end readSerialCommands()

// serial input friendly delay...monitors for serial commands faster
#define SDELAY_MS 250
void sDelay(long ms)
{
  #ifdef DEBUG_SDELAY
  if(Serial)
  {
    Serial.print(F("sDelay("));
    Serial.print(ms);
    Serial.println(F(")"));
  }
  unsigned long eTimeDebug = millis();
  #endif
  
  // do tiny delays of SDELAY_MS ms until we are waiting less than that
  while(ms > SDELAY_MS)
  {
    // Time and run readSerialCommands
    unsigned long eTime=millis();
    readSerialCommands();
    eTime = millis()-eTime;

    // Wait remainder of SDELAY_MS    
    if(eTime < SDELAY_MS)
    {
      delay(SDELAY_MS-eTime);
      ms-=SDELAY_MS;
    }
    else
    {
      ms-=eTime;
    }
  }
  
  // if there was some small <SDELAY_MS ms left, wait that amount now.
  if(ms > 0)
    delay(ms);

  #ifdef DEBUG_SDELAY
  eTimeDebug = millis()-eTimeDebug;
  if(Serial)
  {
    Serial.print(F("Actual sDelay was "));
    Serial.println(eTimeDebug);
  }
  #endif
}



// Reboot to reinitialize defaults
void reboot() {
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}

// debug flashing LED pin
#ifdef DEBUG_LED_PIN
void flashDebugLED(int rate, int count)
{
  for(int x=0; x < count; x++)
  {
    digitalWrite(DEBUG_LED_PIN,HIGH);
    delay(rate);
    digitalWrite(DEBUG_LED_PIN,LOW);
    delay(rate);
  }
}
#endif
