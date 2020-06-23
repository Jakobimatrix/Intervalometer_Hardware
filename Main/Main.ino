
// ATtiny85 Intervallometer. Function dictateing the intervalls between shots via Bluetooth and App for remote triggered time lapse.
// Author: Jakob Wandel
// Date: 22.06.2020
// Version: 0
// Limitations: All shots must be done withing 49 Days (millis() overflow)

// ATMEL ATTINY 85 / ARDUINO
//
//              +-\/-+
// !RESET PB5  1|    |8  Vcc
// RxD    PB3  2|    |7  PB2 HW_HOLD_SWITCH
// TxD    PB4  3|    |6  PB1 HW_TRIGGER
//        GND  4|    |5  PB0 HW_STATUS_LED
//              +----+

#include <SoftwareSerial.h>  //Software Serial Port
#include <avr/sleep.h>    // Sleep Modes
#include <avr/power.h>    // Power management
#include <avr/wdt.h>      // Watchdog timer

// definitions of possible states fo the intervallometer
const byte STATE_NO_INSTRUCTIONS = 0x0;
const byte STATE_RECEIVEING_DATA = 0x1;
const byte STATE_HOLD = 0x2;
const byte STATE_RUN = 0x3;
const byte STATE_FINISHED = 0x4;
const byte STATE_ERROR_FAILED_DATA_TRANSFERE = 0xF;

// definitions for status LED
const int LED_NO_INSTRUCTIONS_ON_MS = 1000;
const int LED_NO_INSTRUCTIONS_OFF_MS = 1000;
const int LED_STATE_RECEIVEING_DATA_ON_MS = 300;
const int LED_STATE_RECEIVEING_DATA_OFF_MS = 300;
const int LED_STATE_HOLD_ON_MS = 1000;
const int LED_STATE_HOLD_OFF_MS = 400;
const int LED_FINISHED_ON_MS = 300;
const int LED_FINISHED_OFF_MS = 300;
const byte LED_FINISHED_RUN = 3;
const int LED_FINISHED_PAUSE_MS = 1000;

// protocol definitions;
const byte SYMBOL_STOP = 0xF;
const byte SYMBOL_F_CONST = 0x1;
const byte SYMBOL_F_LIN = 0x2;
const byte SYMBOL_F_QUAD = 0x3;
const byte NUM_BYTES_VALUE = 4;
const byte NUM_VALUES_F_CONST = 2;
const byte NUM_VALUES_F_LIN = 3;
const byte NUM_VALUES_F_QUAD = 4;

// Returns the offset to the Function Type Symbol.
const int getPtrBeginOffsetProg(){
  // [ FUNC | ... ]
  //    ^
  return 0;
}

// Returns the offset to the Number of Pictures taken with a function.
const int getPtrBeginOffsetNumShots(){
  // [ FUNC | NUM | NUM | NUM | NUM | ... ]
  //           ^
  return 1;
}

// Returns the offset to the constant C of a function.
const int getPtrBeginOffsetC(){
  // [ FUNC | NUM | NUM | NUM | NUM | C | C | C | C | ... ]
  //                                  ^                          
  return 5;
}

// Returns the offset to the constant B of a function.
const int getPtrBeginOffsetB(){
  // [ FUNC | NUM | NUM | NUM | NUM | C | C | C | C | B | B | B | B | ... ]
  //                                                  ^                          
  return 9;
}

// Returns the offset to the constant A of a function.
const int getPtrBeginOffsetA(){
  // [ FUNC | NUM | NUM | NUM | NUM | C | C | C | C | B | B | B | B | A | A | A | A ]
  //                                                                  ^                          
  return 13;
}

// globals for status
const byte HW_STATUS_LED = PB0;
#define TRIGGER_DURATION_MS 300 // how many ms the trigger needs to be high to take a photo depends on the camera and the used transistor
const byte HW_TRIGGER = PB1;
#define HW_HOLD_SWITCH_HOLD_COND HIGH // define if pull up or pull down for HW_HOLD_SWITCH
const byte HW_HOLD_SWITCH = PB2;
const byte RxD = PB3;
const byte TxD = PB4;
byte state_prog = STATE_NO_INSTRUCTIONS;
long status_led_last_toggle_ms = 0;
byte num_runs_STATUS_LED = 0;

// Stores the program send via bluetooth.
// This array should use all the avaiable memory.
byte program[100];
// Points to the current index of the program.
int program_pointer = 0; // pointer to the current function
int read_pointer = 0; // pointer to the current read/write position
int shot_counter = 0; // number of photos taken with the current function
long last_shot_ms = 0; // last time a photo was taken
long f_nm1 = 0; // last value of the recursive function determineing the delay between shots

 
SoftwareSerial blueToothSerial(RxD,TxD);

void setup() 
{ 
  //resetWatchdog();
  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);
  setupBlueToothConnection();
  
  pinMode(HW_STATUS_LED,OUTPUT);
  pinMode(HW_TRIGGER,OUTPUT);
  pinMode(STATE_HOLD,INPUT);
  digitalWrite(HW_STATUS_LED,HIGH);
  digitalWrite(HW_TRIGGER,LOW);
} 
 
void loop()
{
  switch (state_prog) {
    case STATE_NO_INSTRUCTIONS:
    case STATE_RECEIVEING_DATA:
      read();
      break;
    case STATE_HOLD:
      checkIfHold();
      break;
    case STATE_RUN:
      checkIfHold();
      runIntervallometer();
      break;
      case STATE_FINISHED:
      // TODO go to sleep LED NEEDS BLINKING NO INTERRUPT FROM HOLD
      break;
    default:
      // TODO go to sleep LED NEEDS BLINKING NO INTERRUPT FROM HOLD
      break;
  }
  statusLed();
}

/*!
 * \brief toggleStatusLed toggles the state of the status led and sets the global variable
 * status_led_last_toggle_ms to the current millisecond counter
 */
void toggleStatusLed(){
  digitalWrite(HW_STATUS_LED,!digitalRead(HW_STATUS_LED));
  status_led_last_toggle_ms = millis();
}

/*!
 * \brief toggleStatusLedIf uses toggleStatusLed if the time since the last toggle
 * is greater then a given periode.
 * \param periode The periode after which the LED should be toggled.
 */
void toggleStatusLedIf(int periode){
  if(millis() - status_led_last_toggle_ms > periode){
    toggleStatusLed();
  }
}

/*!
 * \brief statusLed controlls the status LED depending on the current state.
 */
void statusLed(){
  bool led_is_on = digitalRead(HW_STATUS_LED) == HIGH;
  switch (state_prog) {
    case STATE_NO_INSTRUCTIONS:
      if(led_is_on){
        toggleStatusLedIf(LED_NO_INSTRUCTIONS_ON_MS);
      }else{
        toggleStatusLedIf(LED_NO_INSTRUCTIONS_OFF_MS);
      }
      break;
    case STATE_RECEIVEING_DATA:
      if(led_is_on){
        toggleStatusLedIf(LED_STATE_RECEIVEING_DATA_ON_MS);
      }else{
        toggleStatusLedIf(LED_STATE_RECEIVEING_DATA_OFF_MS);
      }
      break;
    case STATE_HOLD:
      if(led_is_on){
        toggleStatusLedIf(LED_STATE_HOLD_ON_MS);
      }else{
        toggleStatusLedIf(LED_STATE_HOLD_OFF_MS);
      }
      break;
    case STATE_RUN:
      if(led_is_on){
        toggleStatusLed();
      }
      break;
      case STATE_FINISHED:
        if(led_is_on){
          toggleStatusLedIf(LED_FINISHED_ON_MS);
          num_runs_STATUS_LED++;
        }else if(num_runs_STATUS_LED > LED_FINISHED_RUN){
          num_runs_STATUS_LED = 0;
          toggleStatusLedIf(LED_FINISHED_PAUSE_MS);
        }else{
          toggleStatusLedIf(LED_FINISHED_OFF_MS);
        }
      break;
    default:
      digitalWrite(HW_STATUS_LED,HIGH);
      break;
  }
}

/*!
 * \brief checkIfHold checks if the user switched the STATE_HOLD switch.
 * switch the state between STATE_HOLD and RUN
 */
void checkIfHold(){
  // No need to debounce.
  state_prog = (digitalRead(HW_HOLD_SWITCH) == HW_HOLD_SWITCH_HOLD_COND)?STATE_HOLD:STATE_RUN;
  // TODO set up watchdog
  // TODO go to sleep, wake up on pull down/up?
}

/*!
 * \brief getNumFunctionValues returns the number of values (constants) a Function has
 * \param function The Hex value of the function Symbol
 * \return the number of parameters of the requested function.
 * If END Symbol was given, 0 is returned.
 * If the given Hex does not belong to a function, 0xF will be returned.
 */
byte getNumFunctionValues(byte function){
 switch (function) {
    case SYMBOL_F_CONST:
    return NUM_VALUES_F_CONST;
    case SYMBOL_F_LIN:
    return NUM_VALUES_F_LIN;
    case SYMBOL_F_QUAD:
    return NUM_VALUES_F_QUAD;
    case SYMBOL_STOP:
    return 0; // End, no more values
 }
 state_prog = STATE_ERROR_FAILED_DATA_TRANSFERE;
 return 0;
}

/*!
 * \brief runIntervallometer Runs the main program.
 * It expects that the program pointers to be set to zero prior the first call and
 * not altered while the program runs. A class structure would be nice but eh...
 */
void runIntervallometer(){
  // program[read_pointer];
  // program[program_pointer]: points to the current function
  // shot_counter: how many pictures where taken with the current function
  // program[program_pointer+1]: numbers of pictures to take with this function
  // last_shot_ms: when the last shot was taken
  if(shot_counter >= readProgramValueAt(program_pointer + getPtrBeginOffsetNumShots())){
    program_pointer++;
  }
  const long ms_since_last_pic = millis() - last_shot_ms;
  long ms_until_next_pic;
  switch (program[program_pointer]) {
    case SYMBOL_F_CONST:
      ms_until_next_pic = getShutterDelayConst() - ms_since_last_pic;
      break;
    case SYMBOL_F_LIN:
      ms_until_next_pic = getShutterDelayLin()- ms_since_last_pic;
      break;
    case SYMBOL_F_QUAD:
      ms_until_next_pic = getShutterDelayQuad()- ms_since_last_pic;
      break;
    case SYMBOL_STOP:
      state_prog = STATE_FINISHED;
    return;
  }

  // TODO go into deep_sleep for ms_until_next_pic using watchdog BUT LISTEN FOR HOLD
  delay(ms_until_next_pic);
  takePhoto();
}

/*!
 * \brief takePhoto triggers the trigger to take a photo and keeps track when the foto was taken.
 */
void takePhoto(){
  // make sure the LED does not emit light (it should be off anyway)
  digitalWrite(HW_STATUS_LED, LOW);
  digitalWrite(HW_TRIGGER,HIGH);
  delay(TRIGGER_DURATION_MS);
  digitalWrite(HW_TRIGGER,LOW);
  shot_counter++;
  last_shot_ms = millis();
}

/*!
 * \brief readProgramValueAt converts beginning from at the next 4 bytes into an int
 */
long readProgramValueAt(int at){
  return (long)( (program[at] << 24) 
               + (program[at+1] << 16) 
               + (program[at+2] << 8) 
               + (program[at+3] ) );
}

/*!
 * \brief getShutterDelayConst Calculates the shutter delay for the n-th picture for a constant function.
 * \return how long to delay until the next picture should be taken in ms.
 */
long getShutterDelayConst(){
  //[ FUNC_SYMBOL | NUM_PICS | C ]
  // f(n) = f(n-1); f(0) = C;
  if(shot_counter == 0){
    f_nm1 = readProgramValueAt(program_pointer + getPtrBeginOffsetC);
  }
  return f_nm1;
}

/*!
 * \brief getShutterDelayLin Calculates the shutter delay for the n-th picture for a linear function.
 * \return how long to delay until the next picture should be taken in ms.
 */
long getShutterDelayLin(){
  //[ FUNC_SYMBOL | NUM_PICS | C | B ]
  // f(0) = C
  // f(n) = f(n-1) + B
  if(shot_counter == 0){
    f_nm1 = readProgramValueAt(program_pointer + getPtrBeginOffsetC());
  }else{
    f_nm1 = f_nm1 + readProgramValueAt(program_pointer + getPtrBeginOffsetB());
  }
  return f_nm1;
}

/*!
 * \brief getShutterDelayLin Calculates the shutter delay for the n-th picture for a quadratic function.
 * \return how long to delay until the next picture should be taken in ms.
 */
long getShutterDelayQuad(){
  //[ FUNC_SYMBOL | NUM_PICS | C | B | A ]
  // f(0) = C
  // f(n) = f(n-1) + A(n-1) + B
  if(shot_counter == 0){
    f_nm1 = readProgramValueAt(program_pointer + getPtrBeginOffsetC());
    return f_nm1;
  }else{
    f_nm1 = f_nm1
            + readProgramValueAt(program_pointer + getPtrBeginOffsetA())*(shot_counter-1)
            + readProgramValueAt(program_pointer + getPtrBeginOffsetB());
  }
  return f_nm1;
}
 
 
void setupBlueToothConnection()
{
  blueToothSerial.begin(9600); //Set BluetoothBee BaudRate to default baud rate 38400
  blueToothSerial.print("\r\n+STWMOD=0\r\n"); //set the bluetooth work in slave mode
  blueToothSerial.print("\r\n+STNA=HC-05\r\n"); //set the bluetooth name as "HC-05"
  blueToothSerial.print("\r\n+STOAUT=1\r\n"); // Permit Paired device to connect me
  blueToothSerial.print("\r\n+STAUTO=0\r\n"); // Auto-connection should be forbidden here
  
  delay(2000); // This delay is required.
  //blueToothSerial.print("\r\n+INQ=1\r\n"); //make the slave bluetooth inquirable 
  blueToothSerial.print("bluetooth connected!\n");
  
  delay(2000); // This delay is required.
  blueToothSerial.flush();
}

/*!
 * \brief read Reads the next bit send via bluetooth if avaiable and
 * CHANGES the state to STATE_HOLD if the SYMBOL_STOP was received.
 */
void read(){
  //check if there's any data sent from the remote bluetooth shield
  if(blueToothSerial.available()){
    state_prog = STATE_RECEIVEING_DATA;
    program[read_pointer] = blueToothSerial.read();

    const int diff = (read_pointer - program_pointer);
    const int num_expected_bytes = (int) getNumFunctionValues(program[program_pointer]) * (int) NUM_BYTES_VALUE;
    if(diff == 0 && num_expected_bytes == 0 && program[program_pointer] == SYMBOL_STOP){
      // Stop symbol received; transmission successful.
      // Reset the program pointers.
      program_pointer = 0;
      read_pointer = 0;
      checkIfHold();
      shutDownBluetooth();
      return;
    }
    
    if(diff == num_expected_bytes){
      // we read the last byte of the current function
      program_pointer++;
    }else if(diff > num_expected_bytes || diff < 0 || num_expected_bytes == 0){
      state_prog = STATE_ERROR_FAILED_DATA_TRANSFERE;
      return;
    }
    read_pointer++;
  }
}


/*!
 * \brief shutDownBluetooth releases the Pins from beeing RX and Tx and makes the gpios
 */
void shutDownBluetooth(){
  blueToothSerial.end();
}

//
//// ?
//// http://gammon.com.au/forum/?id=11497&reply=6#reply6Posteingangx
//ISR (PCINT0_vect) 
//{
// // do something interesting here
//}
//
//// ?
//// http://gammon.com.au/forum/?id=11497&reply=6#reply6Posteingangx
//// watchdog interrupt
//ISR (WDT_vect) 
//{
//   wdt_disable();  // disable watchdog
//   // WAKE UP? TODO 
//}
//
//
//// reset the watchdog for TODO millisecconds
//// http://gammon.com.au/forum/?id=11497&reply=6#reply6Posteingangx
//void resetWatchdog()
//{
//  // clear various "reset" flags
//  MCUSR = 0;     
//  // allow changes, disable reset, clear existing interrupt
//  WDTCR = bit(WDCE) | bit(WDE) | bit(WDIF);
//  // set interrupt mode and an interval (WDE must be changed from 1 to 0 here)
//  WDTCR = bit(WDIE) | bit(WDP3) | bit(WDP0);    // set WDIE, and 8 seconds delay
//  // pat the dog
//  wdt_reset();  
//}
//
//
//// put to sleep mode with watchdog on
//// http://gammon.com.au/forum/?id=11497&reply=6#reply6Posteingangx
//void goToSleep()
//{
//  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
//  ADCSRA = 0;            // turn off ADC
//  power_all_disable();  // power off ADC, Timer 0 and 1, serial interface
//  noInterrupts();       // timed sequence coming up
//  resetWatchdog();      // get watchdog ready
//  sleep_enable();       // ready to sleep
//  interrupts();         // interrupts are required now
//  sleep_cpu();          // sleep                
//  sleep_disable();      // precaution
//  power_all_enable();   // power everything back on
//}
