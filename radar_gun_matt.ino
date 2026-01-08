//Matthew Miller
//Dec 2025
//Based on project from Kevin Darrah - http://www.kevindarrah.com/wiki/index.php?title=Arduino_Radar_Gun
//NOTE - SET BOARD TYPE "Arduino Micro" for Leonardo Micro board or TX/RX LEDs will be stuck on!

/*
 * Initial version
 *   Created initial version
 * v2
 *   Optimized readability
 *   Added serial command processing
 *   Optimized LCD digit decoding (NOTE - this version may have decoding bug in parameter order
 *   Improved preprocessor directives for debugging
 */



#define INFO_ON                //Print informational outputs (e.g. sent power commands, low battery)
//#define DEBUG_ON               //Print debug statments
//#define DEBUG_CONTROLS_ON      //Run test of button actions during startup
//#define DEBUG_LED_PIN 13       //Flash LED pin 13 to debug USB signal detect and sleep
//#define DEBUG_TRIGGER_ON       //Print information when pulling/releasing trigger
#define INFO_SERIAL_INPUT_ON   //Print informational outputs (e.g. echo back commands after set)
//#define DEBUG_SERIAL_INPUT_ON  //Print debug statments related to serial input received

#define RADIATE_OFF_TIME 5000 //mS delay idle between pricessing loop iterations
#define RADIATE_SCAN_TIME 1000  //mS duration of radar active scan; (0)=never radiate; (-1)=always on
#define LCD_SCAN_DELAY 10   //mS delay for LCD to stabilize before reading
#define LCD_SCAN_TIME 20    //mS duration to keep scanning LCD for active segments

// Control pins
// Pull output and pull low to activate
// Set input and low (no pull-up) to deactivate
#define TRIGGER_PIN   2  //Digital Pin
#define POWER_ON_PIN  3  //Digital Pin
#define POWER_OFF_PIN 12 //Digital Pin
#define POWER_OFF_SENSE_PIN 11 //Analog Pin
//Note - D12 is also an analog input so we can "wait a bit" after power-down for the voltage to bleed off or it gets stuck

//Battery vs USB detection pins
#define USB_POWER_PIN 7 //Digital Pin, should also be interrupt

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



//**** end of configuration settings  ****
//**** program logic below this point ****

#include<avr/sleep.h>

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

int segment[4][7];//LCD segment data stored here

int measuredSpeed, oldSpeed;//converted data to speed
boolean measuredSpeedValid = false; //stores whether the speed data is valid
int failedDecodeCount = 0; //count of invalid readings since startup

//Run control options (could be adjusted later, defaults set here)
long radiateOffTime = RADIATE_OFF_TIME;
long radiateScanTime = RADIATE_SCAN_TIME;
boolean autoRunRadar = true;



void setup()
{
  Serial.begin(115200);
  
  long watchdog = millis();
  //wait up to 5 sec for serial to initialize
  while(((millis()-watchdog) < 5000) && !Serial && isUsbPower());
  
  if(Serial) Serial.println("Starting up...");

  //Initialize pins and such
  #ifdef DEBUG_LED_PIN
  pinMode(DEBUG_LED_PIN,OUTPUT);
  #endif
  //configure USB power detect pin - internal pullup on for transistor to pull down
  pinMode(USB_POWER_PIN,INPUT);
  digitalWrite(USB_POWER_PIN,HIGH);
  //configure sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  //adjust ADC to get faster readings (we can afford less precision)
  ADC_TIME_13;


  //show mode for debugging
  #ifdef DEBUG_ON
  if(Serial) 
  {
    Serial.print("Power source: ");
    Serial.println( isUsbPower() ? "USB" : "Battery");
    Serial.println("Setup complete, starting processing loop.");
    Serial.println();
    Serial.print("Trigger Button Level: ");
    Serial.println(digitalRead(TRIGGER_PIN));
    Serial.print("Power On Button Level: ");
    Serial.println(digitalRead(POWER_ON_PIN));
    Serial.print("Power Off Button Level: ");
    Serial.print(digitalRead(POWER_OFF_PIN));
    Serial.print(" / ");
    Serial.println(analogRead(POWER_OFF_SENSE_PIN));
    int x=micros();
    analogRead(POWER_OFF_SENSE_PIN);
    x = micros() - x;
    Serial.print("analogRead time micros = ");
    Serial.println(x);
    Serial.println("----------------------------------------");
  }
  #endif
  #ifdef DEBUG_CONTROLS_ON
  //Leaving the if(Serial) on each print becasue we may want headless tests to run on start
  if(Serial) Serial.println("Running control signal tests . . .");
  if(isPowerOn())
  {
    if(Serial) Serial.println("WARNING: Power is already on, may cause unexpected results!");
  }
  boolean passing = true;
  delay(5000);
  if(Serial) Serial.println("Testing power-on signal . . .");
  powerOn();
  if(Serial) Serial.println("Done.");
  passing = passing && isPowerOn;
  delay(5000);
  if(Serial) Serial.println("Testing hold-trigger signal . . .");
  holdTrigger();
  if(Serial) Serial.println("Done.");
  passing = passing && isRadiating();
  delay(5000);
  if(Serial) Serial.println("Testing release-trigger signal . . .");
  releaseTrigger();
  if(Serial) Serial.println("Done.");
  passing = passing && !isRadiating();
  delay(5000);
  if(Serial) Serial.println("Testing power-off signal . . .");
  powerOff();
  if(Serial) Serial.println("Done.");
  passing = passing && !isPowerOn();

  if(Serial)
  {
    Serial.println();
    if(passing)
    {
      Serial.println("All tests passed.");
    }
    else
    {
      Serial.println("One or more tests FAILED!");
    }
    delay(5000);
    Serial.println("----------------------------------------");
  }
  #endif

  //Scan the LCD once so we know what state we are in going into the loop
  scanLcd();

  if(Serial) Serial.println("Ready.");
}//setup



void loop()
{
  #ifdef DEBUG_LED_PIN
  if(isUsbPower())
    flashDebugLED(200,3);
  #endif
  
  //If we are on battery power
  if(!isUsbPower())
  {
    if(isPowerOn())
    {
      //Turn off the radar
      powerOff();
    }
    
    //Put to deep sleep
    goToSleep();
  }

  //Check if any new commands have been sent
  readSerialCommands();

  //Decide if we are running the radar or not
  if(autoRunRadar)
  {
    if(!isPowerOn())
    {
      powerOn();
    }
  
    //Pull the trigger to search for a target
    if(radiateScanTime > 0)
    {
      holdTrigger();
      sDelay(radiateScanTime);
      releaseTrigger();
    }
    else if(radiateScanTime == -1 && !isRadiating())
    {
      holdTrigger();
    }
  
    //The releaseTrigger already scans the LCD, no need to do it again
    //but if the radiateScanTime is 0 or -1 special cases we need to
    if(radiateScanTime < 1)
    {
      scanLcd();
    }
  
    #ifdef INFO_ON
    if(isBatteryLow())
      if(Serial) Serial.println("Low Battery");
    #endif
  
    //Decode and print the speed
    decodeLcdSpeed();
    printMeasuredSpeed();
  
    //Wait before looping for next reading
    sDelay(radiateOffTime);
    
  } //if: run radar
  else //if radar is not running
  {
    //TODO: If its not running
  } //if-else: if radar is not running
}



//Scans the LCD segments and updates stored matrix states
void scanLcd()
{  
  clearLcd();

  //Pause a moment in case something was just busy
  delay(LCD_SCAN_DELAY);
  
  unsigned long scanTimeStart = millis();//timeout for scanning LCD

  while (millis() - scanTimeStart < LCD_SCAN_TIME) //jump in and scan the LCD!
  {
    for (int i = 7; i < 11; i++)  //sweep the common pins
    {
      if (analogRead(i) < 10) // LOW
      {
        for (int j = 0; j < 7; j++) //sweep the segments after we find an enabled common
        {
          if (analogRead(j) > 600) //HIGH
          {
            segment[i - 7][j] = 1;//set the segment to a 1
          } //if: HIGH
        } //for: sweep the segments after we find an enabled common
      } // if: LOW
    } //for: sweep the common pins
  } //while: jump in and scan the LCD!
}



//Decodes an LED segment, returning the value
//The "valid" parameter should be set to true at start of first digit and passed unchanged to subsequent decodes
//Updates "valid" parameter to false and returns 0 on invalid data
//Blank is assumed to be a valid numeric zero
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
  else   //Should never happen - means an invalid combination of segments were turned on
  {
    valid = false;
    return 0;
  }
}

//Decode the LCD matrix into speed integer
void decodeLcdSpeed()
{
  measuredSpeed = 0;//clear the speed, and we'll set it now based on the segment data
  measuredSpeedValid = true; //assume we will be successful

  // ONES POSITION
  //     3,5
  // 3,4     2,5
  //     2,4
  // 1,4     1,5
  //     0,5
  measuredSpeed = decodeLcdDigit(measuredSpeedValid,segment[3][4], segment[3][5], segment[2][5], segment[2][4], segment[1][4], segment[0][5],  segment[1][5]);

  // TENS POSITION
  //     3,3
  // 3,2     2,3
  //     2,2
  // 1,2     1,3
  //     0,3
  measuredSpeed += (10 * decodeLcdDigit(measuredSpeedValid, segment[3][2], segment[2][2], segment[2][3], segment[3][3], segment[1][2], segment[0][3], segment[1][3]) );

  // HUNDREDS
  //     3,1
  // 3,0     2,1
  //     2,0
  // 1,0     1,1
  //     0,1
  measuredSpeed += (100 * decodeLcdDigit(measuredSpeedValid, segment[3][1], segment[3][0], segment[2][1], segment[2][0], segment[1][0], segment[0][1],  segment[1][1]) );

  if(isMph() == isKph()) //Must be one or the other; if neither or both we have bad data
  {
    measuredSpeedValid = false;
  }

  //Should never happen - means an invalid combination of segments were turned on
  if(!measuredSpeedValid)
  {
    failedDecodeCount++;
  }
}

//Print speed if different from previously stored
void printMeasuredSpeed()
{
  if(measuredSpeedValid)
  {
    if (oldSpeed != measuredSpeed) //ony print speed if it's new and valid
    {
      if(Serial)
      {
        Serial.print("Speed ");
        Serial.print(measuredSpeed);
        Serial.println( isMph() ? " mph" : (isKph() ? " kph" : " ???"));
      }
    }
    oldSpeed = measuredSpeed;
  }
  #ifdef DEBUG_ON
  else
  {
    if(Serial) Serial.println("Invalid measurement decode");
    dumpLcd();
    if(Serial)
    {
      Serial.print("Total failed decodes: ");
      Serial.println(failedDecodeCount);
    }
  }
  #endif
}

//Clear the LCD matrix
void clearLcd()
{
  for (int i = 0; i < 4; i++) {//for degugging and clearing segment data
    //Serial.print(i);
    //Serial.print(": ");
    for (int j = 0; j < 7; j++) {
      //Serial.print(segment[i][j]);
      //Serial.print(",");
      segment[i][j] = 0;
    }
    //Serial.println("  ");
  }
}

//Debugging - print out the detected LCD segments raw
void dumpLcd()
{
  #ifdef DEBUG_ON
  for (int i = 0; i < 4; i++)
  {
    if(Serial) Serial.print(i);
    if(Serial) Serial.print(": ");
    for (int j = 0; j < 7; j++)
    {
      if(Serial) Serial.print(segment[i][j]);
      if(j < 6)
      {
        if(Serial) Serial.print(",");
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
  
    Serial.print(" ");Serial.print( segment[3][1] );Serial.print(" ");              Serial.print("   ");   Serial.print(" ");Serial.print( segment[3][3] );Serial.print(" ");               Serial.print("   ");  Serial.print(" ");Serial.print( segment[3][5] );Serial.println(" ");
    
    Serial.print( segment[3][0] );Serial.print(" ");Serial.print( segment[2][1] );  Serial.print("   ");   Serial.print( segment[3][2] );Serial.print(" ");Serial.print( segment[2][3] );   Serial.print("   ");  Serial.print( segment[3][4] );Serial.print(" ");Serial.println( segment[2][5] );
    
    Serial.print(" ");Serial.print( segment[2][0] );Serial.print(" ");              Serial.print("   ");   Serial.print(" ");Serial.print( segment[2][2] );Serial.print(" ");               Serial.print("   ");  Serial.print(" ");Serial.print( segment[2][4] );Serial.println(" ");
    
    Serial.print( segment[1][0] );Serial.print(" ");Serial.print( segment[1][1] );  Serial.print("   ");   Serial.print( segment[1][2] );Serial.print(" ");Serial.print( segment[1][3] );   Serial.print("   ");  Serial.print( segment[1][4] );Serial.print(" ");Serial.println( segment[1][5] ); 
    
    Serial.print(" ");Serial.print( segment[0][1] );Serial.print(" ");              Serial.print(" . ");   Serial.print(" ");Serial.print( segment[0][3] );Serial.print(" ");               Serial.print(" . ");  Serial.print(" ");Serial.print( segment[0][5] );Serial.println(" ");
  
    Serial.println();
    Serial.print("mph  = ");Serial.println(segment[2][6]);
    Serial.print("kph  = ");Serial.println(segment[1][6]);
    Serial.print("batt = ");Serial.println(segment[0][6]);
    Serial.print("rdr  = ");Serial.println(segment[3][6]);
    Serial.println();
  }
}

//Returns whether the LCD test passed (all segments on)
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
    Serial.print("LCD Self Test Count = ");
    Serial.println(sum);
  }
  #endif

  return sum == 25; //Should be all 25 segments "on"
}

//LCD Data
boolean isBatteryLow()
{
  return segment[0][6];
}

//LCD Data
boolean isKph()
{
  return segment[1][6];
}

//LCD Data
boolean isMph()
{
  return segment[2][6];
}

//LCD Data
boolean isRadiating()
{
  return segment[3][6];
}

//Is the power source USB
boolean isUsbPower()
{
  return !digitalRead(USB_POWER_PIN);
}

//Is the radar on (based on LCD Data)
boolean isPowerOn()
{
  scanLcd();
  //Check the LCD - if its on one of these should be illuminated
  return isKph() || isMph();
}

//Send command to radar
void powerOn()
{
  #ifdef INFO_ON
  if(Serial) Serial.println("Sending: powerOn");
  #endif

  releaseTrigger();
  
  //Set it as an output to pull it hard low
  pinMode(POWER_ON_PIN,OUTPUT);
  delay(100);
  //Set it as an input to let it float
  pinMode(POWER_ON_PIN,INPUT);

  scanLcd();

  #ifdef DEBUG_ON
  //Verify display self-test
    if(Serial) Serial.println( verifyLcdTest() ? "lcdTest: OK" : "lcdTest: FAIL");
  #endif
  

  //Wait for self test to clear
  delay(1100);
  
  scanLcd();

  //Verify power
  #ifdef INFO_ON
  if(Serial) Serial.println( isPowerOn() ? "powerOn: OK" : "powerOn: FAIL");
  #endif
}

//Send command to radar
void powerOff()
{
  #ifdef INFO_ON
  if(Serial) Serial.println("Sending: powerOff");
  #endif

  releaseTrigger();
  
  //Set it as an output to pull it hard low
  pinMode(POWER_OFF_PIN,OUTPUT);
  delay(3100); //wait 3 sec countdown plus a bit
  //Set it as an input to let it float
  pinMode(POWER_OFF_PIN,INPUT);
  //Delay for voltage to stabilize
  delay(10);

  //TODO: Check power off pin voltage and wait a bit
  //Unsure why this sometimes happens
  
  scanLcd();
  
  //Verify power
  #ifdef INFO_ON
  if(Serial) Serial.println( isPowerOn() ? "powerOff: FAIL" : "powerOff: OK");
  #endif
}

//Send command to radar
void holdTrigger()
{
  #ifdef DEBUG_TRIGGER_ON
  if(Serial) Serial.println("Sending: holdTrigger");
  #endif
  
  //Set it as an output to pull it hard low
  pinMode(TRIGGER_PIN,OUTPUT);

  scanLcd();

  //Verify radiating
  #ifdef DEBUG_TRIGGER_ON
  if(Serial) Serial.println( isRadiating() ? "holdTrigger: OK" : "holdTrigger: FAIL");
  #endif
}

//Send command to radar
void releaseTrigger()
{
  #ifdef DEBUG_TRIGGER_ON
  if(Serial) Serial.println("Sending: releaseTrigger");
  #endif
  
  //Set it as an output to pull it hard low
  pinMode(TRIGGER_PIN,INPUT);

  scanLcd();

  //Verify not radiating
  #ifdef DEBUG_TRIGGER_ON
  if(Serial) Serial.println( isRadiating() ? "releaseTrigger: FAIL" : "releaseTrigger: OK");
  #endif
}

//put Arduino board to sleep mode
void goToSleep()
{
  #ifdef DEBUG_LED_PIN
  flashDebugLED(1000,5);
  #endif
  
  enableInterrupt();       // enable the interrupt pin so it can wake back up
  
  sleep_enable();          // enables the sleep bit in the mcucr register
                           // so sleep is possible. just a safety pin

  sleep_mode();            // here the device is actually put to sleep!!
                           // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP

  sleep_disable();         // first thing after waking from sleep:
                           // disable sleep...

  disableInterrupt();      // disable the interrupt because it interferes with USB

  #ifdef DEBUG_LED_PIN
  flashDebugLED(100,50);
  #endif
}

//Code run when the interrupt fires
void doInterrupt()
{
  //We don't actually need to do anything here, its only used to wake from sleep
}

//disable interrupt so we don't get stuck
boolean interruptEnabled = false;
void disableInterrupt()
{
  if(interruptEnabled)
  {
    detachInterrupt(digitalPinToInterrupt(USB_POWER_PIN));
    interruptEnabled=false;
  }
}

//enable interrupt so USB Connect is picked up
void enableInterrupt()
{
  if(!interruptEnabled)
  {
    // use interrupt and run function
    attachInterrupt(digitalPinToInterrupt(USB_POWER_PIN), doInterrupt, LOW);
    interruptEnabled=true;
  }
}

//print expected serial command syntax
void printSerialCommands()
{
  if(Serial)
  {
    //                    NOTE - First letter of command must be unique
    //                           because it is used for switch statment
    Serial.println(F("##############################################################"));
    Serial.println(F("#                                                            #"));
    Serial.println(F("#         [CMD]    ARG Action                                #"));
    Serial.println(F("#         -------- --- ------------------------------------  #"));
    Serial.println(F("#  Info                                                      #"));
    Serial.println(F("#         [H]ELP      : Help Menu (this menu)                #"));
    Serial.println(F("#  Action                                                    #"));
    Serial.println(F("#         [T]RIG    # : Ctrl Trigger       1=Hold 0=Release  #"));
    Serial.println(F("#         [P]OWER   # : Ctrl Power Button  1=On   0=Off      #"));
    Serial.println(F("#         [M]EASURE   : Poll Speed Measurement (if changed)  #"));
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
    Serial.println(F("#         [A]UTO    # : Radar auto-processing 1=On 0=Off     #"));
    Serial.println(F("#         [S]CAN    # : Set Radiating Scan Time (long msec)  #"));
    Serial.println(F("#                       (0)=never radiate; (-1)=always on    #"));
    Serial.println(F("#         [O]FF     # : Set Radiating Off Time  (long msec)  #"));
    Serial.println(F("#                                                            #"));
    Serial.println(F("##############################################################"));
  }
}

//read and process incoming serial commands
void readSerialCommands()
{
  if(Serial.available())
  {
    // Read until newline
    String input = Serial.readStringUntil('\n');

    //Convert to upper-case (we don't want case-sensitive)
    input.toUpperCase();

    #ifdef DEBUG_SERIAL_INPUT_ON
    if(Serial)
    {
      Serial.print("Input: \"");
      Serial.print(input);
      Serial.println("\"");
    }
    #endif

    //If we got non-zero input, process it
    if(input.length() > 0)
    {
      //Extract first characater (command key)
      char serialCommand = input[0];

      //Storage for numeric argument (-999 is invalid)
      long serialCommandArg = -999;

      //Search for numeric arguments
      int searchIndex = 0;
      int scalor = 1; //positive/negative scalor
      while(searchIndex < input.length() && (input[searchIndex] < '0' || input[searchIndex] > '9'))
      {
        #ifdef DEBUG_SERIAL_INPUT_ON
        if(Serial)
        {
            Serial.print("L1 searchIndex=");
            Serial.println(searchIndex);
        }
        #endif
        
        searchIndex++;
      }

      //Check if its negative
      if(searchIndex > 0 && input[searchIndex-1] == '-')
      {
        scalor = -1;

        #ifdef DEBUG_SERIAL_INPUT_ON
        if(Serial)
        {
            Serial.print("Found negative '-' char ");
            Serial.println(searchIndex);
        }
        #endif
      }

      //Extract numeric value
      while(searchIndex < input.length() && (input[searchIndex] >= '0' && input[searchIndex] <= '9'))
      {
        if(serialCommandArg == -999)
        {
          serialCommandArg=0;            //Set to zero if first valid value
        }

        serialCommandArg*=10;             //Shift place value
        serialCommandArg+=(input[searchIndex]-'0'); //Add digit

        #ifdef DEBUG_SERIAL_INPUT_ON
        if(Serial)
        {
            Serial.print("L2 searchIndex=");
            Serial.println(searchIndex);
        }
        #endif

        searchIndex++;
      }

      //Make negative if applicable
      serialCommandArg*=scalor;

      #ifdef DEBUG_SERIAL_INPUT_ON
      if(Serial)
      {
          Serial.print("Decoded serialCommandArg=");
          Serial.println(serialCommandArg);
      }
      #endif

      switch (serialCommand)
      {
        case 'H': //HELP - Help Menu
          printSerialCommands();
          break;
          
        case 'T': //TRIG # - Ctrl Trigger 1=Hold 0=Release
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
            if(Serial) Serial.println("ERROR: Invalid command.  Try \"HELP\"");
          }

          #ifdef INFO_SERIAL_INPUT_ON
          if(Serial)
          {
            Serial.print("ACTION TRIG = ");
            Serial.println(isRadiating());
          }
          break;
          #endif
          
        case 'P': //POWER # - Ctrl Power Button 1=On   0=Off
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
            if(Serial) Serial.println("ERROR: Invalid command.  Try \"HELP\"");
          }

          #ifdef INFO_SERIAL_INPUT_ON
          Serial.print("ACTION POWER = ");
          Serial.println(isPowerOn());
          break;
          #endif
          
        case 'M': //Measure - Ctrl Poll Speed Measurement (if changed)
          decodeLcdSpeed();
          printMeasuredSpeed();  
          break;
          
        case 'R': //REFRESH - Poll LCD Scan and print LCD data
          decodeLcdSpeed();

          if(Serial)
          {
            Serial.print("isPowerOn=");
            Serial.println(isPowerOn());
            Serial.print("isBatteryLow=");
            Serial.println(isBatteryLow());
            Serial.print("isMph=");
            Serial.println(isMph());
            Serial.print("isKph=");
            Serial.println(isKph());
            Serial.print("isRadiating=");
            Serial.println(isRadiating());
            Serial.print("measuredSpeed=");
            Serial.println(measuredSpeedValid ? measuredSpeed : -1);
          }
          break;
          
        case 'G': //GET - Print current config values
          if(Serial)
          {
            Serial.print("CONFIG AUTO = ");
            Serial.println(autoRunRadar);
            
            Serial.print("CONFIG SCAN = ");
            Serial.println(radiateScanTime);
            
            Serial.print("CONFIG OFF  = ");
            Serial.println(radiateOffTime);
          }
          break;
          
        case 'A': //AUTO - Radar auto-processing 1=On 0=Off
          if(serialCommandArg == 1)
          {
            autoRunRadar=1;
          }
          else if(serialCommandArg == 0)
          {
            autoRunRadar=0;
          }
          else
          {
            if(Serial) Serial.println("ERROR: Invalid command.  Try \"HELP\"");
          }

          #ifdef INFO_SERIAL_INPUT_ON
          if(Serial)
          {
            Serial.print("CONFIG AUTO = ");
            Serial.println(autoRunRadar);
          }
          #endif
          break;
          
        case 'S': //SCAN - Set Radiating Scan Time (long msec) (0)=never radiate; (-1)=always on
          if(serialCommandArg >= -1)
          {
            radiateScanTime=serialCommandArg;
          }
          else
          {
            if(Serial) Serial.println("ERROR: Invalid command.  Try \"HELP\"");
          }

          #ifdef INFO_SERIAL_INPUT_ON
          if(Serial)
          {
            Serial.print("CONFIG SCAN = ");
            Serial.println(radiateScanTime);
          }
          #endif
          break;
          
        case 'O': //OFF - Set Radiating Off Time  (long msec)
          if(serialCommandArg >= 0)
          {
            radiateOffTime=serialCommandArg;
          }
          else
          {
            if(Serial) Serial.println("ERROR: Invalid command.  Try \"HELP\"");
          }

          #ifdef INFO_SERIAL_INPUT_ON
          if(Serial)
          {
            Serial.print("CONFIG OFF  = ");
            Serial.println(radiateOffTime);
          }
          #endif
          break;
          
        default:
          if(Serial) Serial.println("ERROR: Invalid command.  Try \"HELP\"");
          break;
          
      } //switch/case
    } //if: input length >1
  }//if: serial
}

//serial input friendly delay...monitors for serial commands faster
#define SDELAY_MS 250
void sDelay(int ms)
{
  //do tiny delays of 20ms until we are waiting less than that
  while(ms > SDELAY_MS)
  {
    //Time and run readSerialCommands
    long eTime=millis();
    readSerialCommands();
    eTime = millis()-eTime;

    //Wait remainder of SDELAY_MS
    ms-=(SDELAY_MS-eTime);
    delay(SDELAY_MS-eTime);
  }
  //if there was some small <SDELAY_MS ms left, wait that amount now.
  if(ms > 0)
    delay(ms);
}

//debug flashing LED pin
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
