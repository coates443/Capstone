
//https://github.com/coates443/Capstone
//Our github with more information on the project

#include <SPI.h>
#include <IntervalTimer.h>
#include <ILI9341_t3.h>
#include <stdio.h>
#include <time.h>
#include <ADC.h>
#include <ADC_util.h>


// ------------------------
// Button Pins (placeholders)
// ------------------------
//(could not implement buttons on board, so we used the keypad, and column 3 of keypad is pin 21 and column 4 is 22)
// active low buttons
#define BTN_CORRECT   21 //buttons for choosing if the PRF is correct or not
#define BTN_INCORRECT 22 //buttons are also for choosing short/long pulse length for PCR_Trigger

// ------------------------
// Display Pins
// ------------------------
#define TFT_DC    9
#define TFT_CS   10
#define TFT_RST   255
// MOSI 11
// MISO 12
// SCK 13

// ------------------------
// PCR Fixture Pins
// ------------------------
#define Charge_Trigger    0
#define Discharge_Trigger 1
#define PRI0              2
#define PRI1              3
#define PRI2              4
#define PCR_Trigger       5
#define PCR_Reset         6
#define Fault_In          7
#define Current_Monitor   14
#define RF_Start          23

#define Keypad4           18
#define Keypad3           17
#define Keypad2           16
#define Keypad1           15

// ------------------------
// Global Variables
// ------------------------
float         bleed_off_current;
float         bleed_time;
int           charge_flag;
float         charge_trigger_time;
int           charge_trigger_width = 10;
unsigned int  current_monitor_raw;
float         current_monitor_voltage;
int           discharge_flag;
float         discharge_trigger_time;
int           discharge_trigger_width = 10;
float         discharge_trigger_delta;
unsigned long old_start_time;
int           operating_mode;
int           PCR_flag;
float         PCR_trigger_time;
int           PCR_trigger_width = 10;
float         PRF;
float         pulse_interval;            //we didn't use pulse interval, but could be calculated in getPRF() method after PRF value is calculated (1/PRF I think)
unsigned long start_time;
int           pulse_mode;

int           timing;                    //used for calculating PCR trigger time for short or long pulse, basically pulse_interval subtracted by 105 or 210, we did not use pulse_interval but would be useful
volatile int  count = 0;                 //for counting falling edges in RF start
              elapsedMillis t = 0;       //timer for counting falling edges in RF Start for PRF
bool          lastState = HIGH;          // used in falling edge logic for PRF
bool          currentState = HIGH;       // used in falling edge logic for PRF

unsigned long rawSum = 0;                //sum of raw voltage values in Current_Monitor
int           rawSamples = 0;            //number of samples of raw voltage values in Current_Monitor
float         averageVoltageRaw = 0;     //stores raw average voltage through Current_Monitor
bool          measuringCurrentMonitor = false;   //for checking if the Current_Monitor is ready to be read from


//ADC reader
ADC *adc = new ADC();

// ------------------------
// Timers
// ------------------------
IntervalTimer charge_timer;     //same funciton as PCR_timer
IntervalTimer discharge_timer;  //same funciton as PCR_timer
IntervalTimer PCR_timer;        //this determines the width/time the trigger is on, currently 10 microseconds

IntervalTimer PCR_triggertimer; //this determines when the trigger is sent in correlation to the RF pulses

// ------------------------
// Display Object
// ------------------------
ILI9341_t3 tft(TFT_CS, TFT_DC, TFT_RST);



// ------------------------
// Function Prototypes
// ------------------------
void outputPRIs(float PRF);
void triggerTimings(float PRF, int pulse_mode);
void startTest();
void chargeTrigger();
void dischargeTrigger();
void PCRTrigger();
void endChargePulse();
void endDischargePulse();
void endPCRPulse();

void getPRF();
void chooseLongOrShort();

//display functions
void displayPRF(float frequency);
void displayShortOrLongChoice();
void displayCompletion();
void displayCurrentMonitor();
void displayStartProgram();


void setup() {
  pinMode(PRI0, OUTPUT);
  pinMode(PRI1, OUTPUT);
  pinMode(PRI2, OUTPUT);

  pinMode(Charge_Trigger, OUTPUT);
  pinMode(Discharge_Trigger, OUTPUT);
  pinMode(PCR_Trigger, OUTPUT);
  pinMode(PCR_Reset, OUTPUT);

  pinMode(Fault_In, INPUT);
  pinMode(Current_Monitor, INPUT);
  pinMode(RF_Start, INPUT);

  // Buttons use internal pullups
  pinMode(BTN_CORRECT, INPUT_PULLUP);
  pinMode(BTN_INCORRECT, INPUT_PULLUP);

  pinMode(Keypad1, OUTPUT);
  pinMode(Keypad2, OUTPUT);
  pinMode(Keypad3, OUTPUT);
  pinMode(Keypad4, OUTPUT);

  // Default outputs high/inactive
  digitalWriteFast(Charge_Trigger, HIGH);
  digitalWriteFast(Discharge_Trigger, HIGH);
  digitalWriteFast(PCR_Trigger, HIGH);

  digitalWriteFast(Keypad1, LOW);
  digitalWriteFast(Keypad2, LOW);
  digitalWriteFast(Keypad3, LOW);
  digitalWriteFast(Keypad4, LOW);

  //sets ADC to 12 bits
  adc->adc0->setAveraging(8); //number of averages to take from
  adc->adc0->setResolution(12); //number of bits representing the read value
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED); //Sets conversion speed
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED); //Sets sampling speed
  adc->adc0->recalibrate();

  // Example initial values, used by last team 
  PRF = 0;
  //pulse_interval = ((1.0 / PRF) * 1e6);
  pulse_interval = 0;
  pulse_mode = 0;
  operating_mode = 0;
  start_time = 0;
  old_start_time = 0;


  charge_flag = 1;
  discharge_flag = 1;
  PCR_flag = 1;

  // Initialize display
  tft.begin();
  tft.setRotation(1);
}


void getPRF(){
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 20);
  tft.println("Finding PRF");

  //This section calculates the PRF
  t = 0;                    //make t=0 here to reset the timer, it counts up in the background of the mcu
  count = 0;
  currentState = HIGH;
  lastState = HIGH;
    while (t <= 5000) {     //a longer time will allow for a more accurate measurement of PRF, if you change the 2000 here, change what you divide the count a few lines below by the same factor
      currentState = digitalReadFast(RF_Start);
      if (lastState == HIGH && currentState == LOW){
        count++;
      }
    lastState = currentState;
    }
  PRF = count / 5.0;  
}

void chooseLongOrShort(){
  bool confirmed = false;
  while (!confirmed) {
    if (digitalRead(BTN_CORRECT) == LOW) {
      timing = (1/PRF)*1000000-105; //gives a shorter trigger time
      confirmed = true;
    }

    if (digitalRead(BTN_INCORRECT) == LOW) {
      timing = (1/PRF)*1000000-210; //gives a longer trigger time
      confirmed = true;
    }
  }
}



//----------------------------
//output PRI's
//----------------------------
void outputPRIs(float PRF) {
  if (PRF <= 327.09) {
    digitalWriteFast(PRI0, HIGH);
    digitalWriteFast(PRI1, HIGH);
    digitalWriteFast(PRI2, HIGH);
  }
  else if ((372.09 < PRF) && (PRF <= 446.52)) {
    digitalWriteFast(PRI0, LOW);
    digitalWriteFast(PRI1, HIGH);
    digitalWriteFast(PRI2, HIGH);
  }
  else if ((446.52 < PRF) && (PRF <= 536.04)) {
    digitalWriteFast(PRI0, HIGH);
    digitalWriteFast(PRI1, LOW);
    digitalWriteFast(PRI2, HIGH);
  }
  else if ((536.04 < PRF) && (PRF <= 643.29)) {
    digitalWriteFast(PRI0, LOW);
    digitalWriteFast(PRI1, LOW);
    digitalWriteFast(PRI2, HIGH);
  }
  else if ((643.29 < PRF) && (PRF <= 771.90)) {
    digitalWriteFast(PRI0, HIGH);
    digitalWriteFast(PRI1, HIGH);
    digitalWriteFast(PRI2, LOW);
  }
  else if ((771.90 < PRF) && (PRF <= 926.35)) {
    digitalWriteFast(PRI0, LOW);
    digitalWriteFast(PRI1, HIGH);
    digitalWriteFast(PRI2, LOW);
  }
  else if ((926.35 < PRF) && (PRF <= 1111.72)) {
    digitalWriteFast(PRI0, HIGH);
    digitalWriteFast(PRI1, LOW);
    digitalWriteFast(PRI2, LOW);
  }
  else {
    digitalWriteFast(PRI0, LOW);
    digitalWriteFast(PRI1, LOW);
    digitalWriteFast(PRI2, LOW);
  }
}

// ------------------------
// triggerTimings()
// ------------------------
//Our PCR trigger time is calculated in getPRF() and is stored in timing, not PCR_trigger_time. This method was here from last capstone
//and may be beneficial to you. We did not need it or use it for our objectives
void triggerTimings(float PRF, int pulse_mode) {
  if (pulse_mode == 0) {
    charge_trigger_time     = pulse_interval - (740.0 + 3.0);
    PCR_trigger_time        = (pulse_interval - (105.0 + 3.0))*1e-3;
    discharge_trigger_delta = (2.24 + 3.0);
  }
  else {
    charge_trigger_time     = pulse_interval - (1200.0 + 3.0);
    PCR_trigger_time        = pulse_interval - (205.0 + 3.0);
    discharge_trigger_delta = (4.48 + 3.0);
  }

  discharge_trigger_time = pulse_interval - discharge_trigger_delta;
}

// ------------------------
// Trigger Functions
// ------------------------
void endChargePulse() {
  charge_flag = 1;
  digitalWriteFast(Charge_Trigger, HIGH);
  charge_timer.end();
}

void endDischargePulse() {
  discharge_flag = 1;
  digitalWriteFast(Discharge_Trigger, HIGH);
  discharge_timer.end();
}

void endPCRPulse() {
  PCR_flag = 1;
  digitalWriteFast(PCR_Trigger, HIGH);
  PCR_timer.end();
  PCR_triggertimer.end();
}

void chargeTrigger() {
  digitalWriteFast(Charge_Trigger, LOW);
  charge_timer.begin(endChargePulse, charge_trigger_width);
}

void dischargeTrigger() {
  digitalWriteFast(Discharge_Trigger, LOW);
  discharge_timer.begin(endDischargePulse, discharge_trigger_width);
}

void PCRTrigger() {
  digitalWriteFast(PCR_Trigger, LOW);
  measuringCurrentMonitor = true;
  PCR_timer.begin(endPCRPulse, PCR_trigger_width);
}



//--------------------------
// Display functions
//--------------------------
void displayStartProgram(){
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 20);
  tft.println("Press the 3 Button");
  tft.setCursor(20, 35);
  tft.println("when you are ready to ");
  tft.setCursor(20, 50);
  tft.println("read the Pulse");
  tft.setCursor(20, 65);
  tft.println("Repetition Frequency");
}

void displayPRF(float PRF){
  tft.fillScreen(ILI9341_BLACK);

  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 20);
  tft.println("Is this the");
  tft.setCursor(20, 50);
  tft.println("frequency you want?");

  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(3);
  tft.setCursor(20, 110);
  tft.print(PRF, 1);
  tft.println(" Hz");

  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(2);
  tft.setCursor(20, 180);
  tft.println("Correct = 3 Button");

  tft.setTextColor(ILI9341_RED);
  tft.setCursor(20, 210);
  tft.println("Incorrect = A Button");
}

void displayShortOrLongChoice(){
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(20, 20);
  tft.println("Is the RF pulse period");
  tft.setCursor(20, 50);
  tft.println("long or short?");

  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(2);
  tft.setCursor(20, 100);
  tft.println("3 Button for short");

  tft.setTextColor(ILI9341_RED);
  tft.setCursor(20, 150);
  tft.println("A Button for long");
}

void displayCompletion(){
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  //tft.setCursor(20, 20);
  //tft.println("PCR trigger is getting sent out. Connect the red wire to ground to show the current monitor info, or the connect green wire to ground to restart the program and find the PRF.");
  tft.setCursor(20, 20);
  tft.println("PCR trigger is getting ");
  tft.setCursor(20, 35);
  tft.println("sent out. Press A Button");
  tft.setCursor(20, 50);
  tft.println("to show the current moni-");
  tft.setCursor(20, 65);
  tft.println("tor info, or press the 3");
  tft.setCursor(20, 80);
  tft.println("button to restart the");
  tft.setCursor(20, 95);
  tft.println("program and find the PRF.");

}

void displayCurrentMonitor(){
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);

  tft.setCursor(20, 20);
  tft.print("Average Voltage: ");
  tft.print(current_monitor_voltage, 3);
  tft.println("V");

  tft.setCursor(20, 80);
  tft.print("Bleed-off Current: ");
  tft.print(bleed_off_current, 2);
  tft.println("mA");

  tft.setCursor(20, 130);
  tft.println("Click A Button to");
  tft.setCursor(20, 145);
  tft.println("recheck current monitor");

  tft.setCursor(20, 180);
  tft.println("Click 3 Button to");
  tft.setCursor(20, 195);
  tft.println("restart program");

}


void loop() {
  while(1){

    delay(500); // delays the button push at the end of the program

    displayStartProgram();

    //waits for the user to confirm the test fixture is ready to read the PRF
    while(digitalRead(BTN_CORRECT) == HIGH){}

    getPRF();
    displayPRF(PRF);

    //code that waits for user input to confirm or re-read PRF through push buttons
    bool confirmed = false;
    while (!confirmed) {
      if (digitalRead(BTN_CORRECT) == LOW) {
        confirmed = true;
      }

      if (digitalRead(BTN_INCORRECT) == LOW) {
        tft.fillScreen(ILI9341_BLACK);
        getPRF();
        displayPRF(PRF);
      }
    }

    displayShortOrLongChoice();
    delay(1000); //gives the user a second to release the button so they don't accidently choose the wrong triggerTiming

    chooseLongOrShort();

    //outputs our PRI bits
    outputPRIs(PRF);

    displayCompletion();

    delay(1000); //same as above delay

    currentState = HIGH; // set these HIGH in case of timing error from last use of states in getPRF()
    lastState = HIGH;
    confirmed = false;
    rawSum = 0;
    rawSamples = 0;
    measuringCurrentMonitor = false;
    bool adcReadInProgress = false;

    while (!confirmed) {
      
      
      currentState = digitalReadFast(RF_Start);
      

      // detect falling edge of RF_Start
      if (lastState == HIGH && currentState == LOW) {

        PCR_triggertimer.begin(PCRTrigger, timing);

        measuringCurrentMonitor = false;  // stop measuring current after RF pulse, this is set true in PCRTrigger() method to start measuring when trigger is sent
        
        averageVoltageRaw = (float)rawSum / rawSamples;             //gives average raw voltage
        current_monitor_voltage = (averageVoltageRaw/4095.0) * 3.3; //converts from raw voltage to volts
        bleed_off_current = current_monitor_voltage * 10;

        rawSum = 0;     //reset these to read the current monitor for the next RF pulse
        rawSamples = 0;
        
        //displays the current monitor info. Currently when you push the button to show it, it flashes the screen, updating the info every about 3 
        //milliseconds. This is because it is called after each PCR trigger and we could not find a way to change it in time.
        //DO NOT HOLD THIS BUTTON DOWN AT THIS STAGE. Press and release it fast because the mcu might burn out from rapidly sending info to the display.
        if(digitalRead(BTN_INCORRECT) == LOW){
          displayCurrentMonitor();
        }
        
      }
        
      lastState = currentState;

      if (measuringCurrentMonitor) {

        //starts ADC read if one is not already running
        if (!adcReadInProgress) {
          adc->adc0->startSingleRead(Current_Monitor);
          adcReadInProgress = true;
        }

        //checks if ADC read is finished
        if (adcReadInProgress && adc->adc0->isComplete()) {
          rawSum += adc->adc0->readSingle();
          rawSamples++;
          adcReadInProgress = false;
        }
      }

      //exits and restarts program
      if (digitalRead(BTN_CORRECT) == LOW) {
        confirmed = true;
      }

      
    }

  }
}
