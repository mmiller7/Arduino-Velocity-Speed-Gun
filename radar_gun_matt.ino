//Matthew Miller
//Dec 2025
//Based on project from Kevin Darrah - http://www.kevindarrah.com/wiki/index.php?title=Arduino_Radar_Gun
//NOTE - SET BOARD TYPE "Arduino Micro" for Leonardo Micro board or TX/RX LEDs will be stuck on!

/*
 * Initial version
 *   Created initial version
 */



#define INFO_ON                //Print informational outputs (e.g. sent power commands, low battery)
//#define DEBUG_ON               //Print debug statments
//#define DEBUG_CONTROLS_ON      //Run test of button actions during startup
//#define DEBUG_LED_PIN 13       //Flash LED pin 13 to debug USB signal detect and sleep
//#define DEBUG_TRIGGER_ON       //Print information when pulling/releasing trigger

#define RADIATE_OFF_DELAY 5000 //mS delay idle between pricessing loop iterations
#define RADIATE_DURATION 1000  //mS duration of radar active scan; (0)=never radiate; (-1)=always on
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
boolean measuredSpeedValid = false;
int failedDecodeCount = 0;



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
  
  if(!isPowerOn())
  {
    powerOn();
  }

  //Pull the trigger to search for a target
  if(RADIATE_DURATION > 0)
  {
    holdTrigger();
    delay(RADIATE_DURATION);
    releaseTrigger();
  }
  else if(RADIATE_DURATION == -1 && !isRadiating())
  {
    holdTrigger();
  }

  //The releaseTrigger already scans the LCD, no need to do it again
  //but if the RADIATE_DURATION is 0 or -1 special cases we need to
  if(RADIATE_DURATION < 1)
  {
    scanLcd();
  }

  #ifdef INFO_ON
  if(isBatteryLow())
    if(Serial) Serial.println("Low Battery");
  #endif

  //Decode and print the speed
  decodeLcdSpeed();
  printLcdSpeed();

  //Wait before looping for next reading
  delay(RADIATE_OFF_DELAY);
}



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

void decodeLcdSpeed()
{
  measuredSpeed = 0;//clear the speed, and we'll set it now based on the segment data
  measuredSpeedValid = true; //assume we will be successful

  /*
   *  ONES POSITION
   */

  //     3,5
  // 3,4     2,5
  //     2,4
  // 1,4     1,5
  //     0,5

  //Zero - Ones
  if (segment[3][4] == 1 && segment[3][5] == 1 && segment[2][5] == 1 && segment[2][4] == 0 && segment[1][4] == 1 && segment[1][5] == 1 && segment[0][5] == 1) {
    measuredSpeed = 0;
  }
  //One - Ones
  else if (segment[3][4] == 0 && segment[3][5] == 0 && segment[2][5] == 1 && segment[2][4] == 0 && segment[1][4] == 0 && segment[1][5] == 1 && segment[0][5] == 0) {
    measuredSpeed = 1;
  }
  //Two - Ones
  else if (segment[3][4] == 0 && segment[3][5] == 1 && segment[2][5] == 1 && segment[2][4] == 1 && segment[1][4] == 1 && segment[1][5] == 0 && segment[0][5] == 1) {
    measuredSpeed = 2;
  }
  //Three - Ones
  else if (segment[3][4] == 0 && segment[3][5] == 1 && segment[2][5] == 1 && segment[2][4] == 1 && segment[1][4] == 0 && segment[1][5] == 1 && segment[0][5] == 1) {
    measuredSpeed = 3;
  }
  //Four - Ones
  else if (segment[3][4] == 1 && segment[3][5] == 0 && segment[2][5] == 1 && segment[2][4] == 1 && segment[1][4] == 0 && segment[1][5] == 1 && segment[0][5] == 0) {
    measuredSpeed = 4;
  }
  //Five - Ones
  else if (segment[3][4] == 1 && segment[3][5] == 1 && segment[2][5] == 0 && segment[2][4] == 1 && segment[1][4] == 0 && segment[1][5] == 1 && segment[0][5] == 1) {
    measuredSpeed = 5;
  }
  //Six - Ones
  else if (segment[3][4] == 1 && segment[3][5] == 1 && segment[2][5] == 0 && segment[2][4] == 1 && segment[1][4] == 1 && segment[1][5] == 1 && segment[0][5] == 1) {
    measuredSpeed = 6;
  }
  //Seven - Ones
  else if (segment[3][4] == 0 && segment[3][5] == 1 && segment[2][5] == 1 && segment[2][4] == 0 && segment[1][4] == 0 && segment[1][5] == 1 && segment[0][5] == 0) {
    measuredSpeed = 7;
  }
  //Eight - Ones
  else if (segment[3][4] == 1 && segment[3][5] == 1 && segment[2][5] == 1 && segment[2][4] == 1 && segment[1][4] == 1 && segment[1][5] == 1 && segment[0][5] == 1) {
    measuredSpeed = 8;
  }
  //Nine - Ones
  else if (segment[3][4] == 1 && segment[3][5] == 1 && segment[2][5] == 1 && segment[2][4] == 1 && segment[1][4] == 0 && segment[1][5] == 1 && segment[0][5] == 1) {
    measuredSpeed = 9;
  }
  //Blank - Ones
  else if (segment[3][4] == 0 && segment[3][5] == 0 && segment[2][5] == 0 && segment[2][4] == 0 && segment[1][4] == 0 && segment[1][5] == 0 && segment[0][5] == 0) {
    measuredSpeed = 0;
  }
  //Should never happen - means an invalid combination of segments were turned on
  else
  {
    measuredSpeedValid = false;
    failedDecodeCount++;
    return;
  }


/*
 * TENS POSITION
 */
  //     3,3
  // 3,2     2,3
  //     2,2
  // 1,2     1,3
  //     0,3

  //One - Tens
  if (segment[3][2] == 0 && segment[2][2] == 0 && segment[2][3] == 1 && segment[3][3] == 0 && segment[1][2] == 0 && segment[1][3] == 1 && segment[0][3] == 0) {
    measuredSpeed = measuredSpeed + (1 * 10);
  }
  //Two - Tens
  else if (segment[3][2] == 0 && segment[2][2] == 1 && segment[2][3] == 1 && segment[3][3] == 1 && segment[1][2] == 1 && segment[1][3] == 0 && segment[0][3] == 1) {
    measuredSpeed = measuredSpeed + (2 * 10);
  }
  //Three - Tens
  else if (segment[3][2] == 0 && segment[2][2] == 1 && segment[2][3] == 1 && segment[3][3] == 1 && segment[1][2] == 0 && segment[1][3] == 1 && segment[0][3] == 1) {
    measuredSpeed = measuredSpeed + (3 * 10);
  }
  //Four - Tens
  else if (segment[3][2] == 1 && segment[2][2] == 1 && segment[2][3] == 1 && segment[3][3] == 0 && segment[1][2] == 0 && segment[1][3] == 1 && segment[0][3] == 0) {
    measuredSpeed = measuredSpeed + (4 * 10);
  }
  //Five - Tens
  else if (segment[3][2] == 1 && segment[2][2] == 1 && segment[2][3] == 0 && segment[3][3] == 1 && segment[1][2] == 0 && segment[1][3] == 1 && segment[0][3] == 1) {
    measuredSpeed = measuredSpeed + (5 * 10);
  }
  //Six - Tens
  else if (segment[3][2] == 1 && segment[2][2] == 1 && segment[2][3] == 0 && segment[3][3] == 1 && segment[1][2] == 1 && segment[1][3] == 1 && segment[0][3] == 1) {
    measuredSpeed = measuredSpeed + (6 * 10);
  }
  //Seven - Tens
  else if (segment[3][2] == 0 && segment[2][2] == 0 && segment[2][3] == 1 && segment[3][3] == 1 && segment[1][2] == 0 && segment[1][3] == 1 && segment[0][3] == 0) {
    measuredSpeed = measuredSpeed + (7 * 10);
  }
  //Eight - Tens
  else if (segment[3][2] == 1 && segment[2][2] == 1 && segment[2][3] == 1 && segment[3][3] == 1 && segment[1][2] == 1 && segment[1][3] == 1 && segment[0][3] == 1) {
    measuredSpeed = measuredSpeed + (8 * 10);
  }
  //Nine - Tens
  else if (segment[3][2] == 1 && segment[2][2] == 1 && segment[2][3] == 1 && segment[3][3] == 1 && segment[1][2] == 0 && segment[1][3] == 1 && segment[0][3] == 1) {
    measuredSpeed = measuredSpeed + (9 * 10);
  }
  //Blank - Tens
  else if (segment[3][2] == 0 && segment[2][2] == 0 && segment[2][3] == 0 && segment[3][3] == 0 && segment[1][2] == 0 && segment[1][3] == 0 && segment[0][3] == 0) {
    //no-op
  }
  //Should never happen - means an invalid combination of segments were turned on
  else
  {
    measuredSpeedValid = false;
    failedDecodeCount++;
    return;
  }

/*
 * HUNDREDS
 */
  //     3,1
  // 3,0     2,1
  //     2,0
  // 1,0     1,1
  //     0,1
  //One - Hundreds
  if (segment[3][1] == 0 && segment[3][0] == 0 && segment[2][1] == 1 && segment[2][0] == 0 && segment[1][0] == 0 && segment[1][1] == 1 && segment[0][1] == 0) {
    measuredSpeed = measuredSpeed + (1 * 100);
  }
  else if (segment[3][1] == 1 && segment[3][0] == 0 && segment[2][1] == 1 && segment[2][0] == 1 && segment[1][0] == 1 && segment[1][1] == 0 && segment[0][1] == 1) {
    measuredSpeed = measuredSpeed + (2 * 100);
  }
  //Blank - Hundreds
  else if (segment[3][1] == 0 && segment[3][0] == 0 && segment[2][1] == 0 && segment[2][0] == 0 && segment[1][0] == 0 && segment[1][1] == 0 && segment[0][1] == 0) {
  //no-op
  }
  //Should never happen - means an invalid combination of segments were turned on
  else
  {
    measuredSpeedValid = false;
    failedDecodeCount++;
    return;
  }

  if((!isMph() && !isKph()) || (isMph() && isKph())) //Must be one or the other; if neither or both we have bad data
  {
    measuredSpeedValid = false;
    failedDecodeCount++;
    return;
  }
}

void printLcdSpeed()
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

void dumpLcd()
{
  /*
  for (int i = 0; i < 4; i++) {//for degugging and clearing segment data
    if(Serial) Serial.print(i);
    if(Serial) Serial.print(": ");
    for (int j = 0; j < 7; j++) {
      if(Serial) Serial.print(segment[i][j]);
      if(Serial) Serial.print(",");
    }
    if(Serial) Serial.println("  ");
  }
*/

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

boolean isUsbPower()
{
  return !digitalRead(USB_POWER_PIN);
}

boolean isPowerOn()
{
  scanLcd();
  //Check the LCD - if its on one of these should be illuminated
  return isKph() || isMph();
}

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

//put board to sleep mode
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
