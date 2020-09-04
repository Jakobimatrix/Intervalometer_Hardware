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

#include "Arduino.h"
#include <SoftwareSerial.h>  //Software Serial Port
#include <avr/power.h>    // Power management

// by Connor Nishijima
#include "TinySnore/src/tinysnore.h"
// If included via submodule (not using Arduino libraries) I need this include too for some reason:
#include "TinySnore/src/tinysnore.cpp"


// definitions of possible states fo the intervallometer
const byte STATE_NO_INSTRUCTIONS = 0x00;
const byte STATE_RECEIVEING_DATA = 0x01;
const byte STATE_HOLD = 0x02;
const byte STATE_RUN = 0x03;
const byte STATE_ERROR_UNKNOWN_SYMBOL_REQUESTED = 0xFF;       // 1
const byte STATE_ERROR_UNCOOL_READ_POINTER_BEHAVIOUR = 0xFE;  // 2
const byte STATE_ERROR_MILLIS_OVERFLOW = 0xFD;                // 3
const byte STATE_ERROR_NEGATIVE_DELAY = 0xFC;                 // 4
const byte STATE_ERROR_PROGR_TOO_LONG = 0xFB;                 // 5
const byte STATE_ERROR_RECEIVING_TOO_LONG = 0xFA;             // 6
const long MAX_RECEIVING_TIME_MS = 3000;

// definitions for status LED
const int LED_NO_INSTRUCTIONS_ON_MS = 1700;
const int LED_NO_INSTRUCTIONS_OFF_MS = 1700;
const int LED_STATE_RECEIVEING_DATA_ON_MS = 10;
const int LED_STATE_RECEIVEING_DATA_OFF_MS = 10;
const int LED_STATE_HOLD_ON_MS = 333;
const int LED_STATE_HOLD_OFF_MS = 333;
const byte LED_STATE_HOLD_RUN = 3;
const int LED_STATE_HOLD_PAUSE_MS = 1000;
const int LED_ERROR_ON_MS = 600;
const int LED_ERROR_OFF_MS = 300;
const int LED_ERROR_PAUSE_MS = 3000;

// protocol definitions;
const byte SYMBOL_STOP = 0x00;
const byte SYMBOL_STOP_SHUTDOWN = 0x0F;
const byte SYMBOL_F_CONST = 0x01;
const byte SYMBOL_F_LIN = 0x02;
const byte SYMBOL_F_QUAD = 0x03;
const byte NUM_BYTES_VALUE = 4;
const byte NUM_VALUES_F_CONST = 2;
const byte NUM_VALUES_F_LIN = 3;
const byte NUM_VALUES_F_QUAD = 4;
const long FLOAT_PRECISION = 1000;

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
#define TRIGGER_DURATION_MS 100 // how many ms the trigger needs to be high to take a photo depends on the camera and the used transistor
const byte HW_TRIGGER = PB1;
#define HW_HOLD_SWITCH_HOLD_COND HIGH // define if pull up or pull down for HW_HOLD_SWITCH
const byte HW_HOLD_SWITCH = PB2;
const byte RxD = PB3;
const byte TxD = PB4;
byte state_prog = STATE_NO_INSTRUCTIONS;
long status_led_last_toggle_ms = 0;
byte num_runs_STATUS_LED = 0;

// Stores the program send via bluetooth.
// This array should use all the avaiable memory But mind that we need some free memory for dynamic variables.
const int NUM_FUNCTION_BYTES = 120; // as of version: V1.1
byte program[NUM_FUNCTION_BYTES];
// Points to the current index of the program.
int program_pointer = 0; // pointer to the current function
int read_pointer = 0; // pointer to the current read/write position
long shot_counter = 0; // number of photos taken with the current function
unsigned long last_shot_ms = 0; // last time a photo was taken
//long f_nm1 = 0; // last value of the recursive function determineing the delay between shots

SoftwareSerial blueToothSerial(RxD,TxD);

void setup() 
{ 
  //resetWatchdog();
  pinMode(RxD, INPUT);
  pinMode(TxD, OUTPUT);
  enableBluetooth(true);
  
  pinMode(HW_STATUS_LED,OUTPUT);
  pinMode(HW_TRIGGER,OUTPUT);
  pinMode(HW_HOLD_SWITCH,INPUT);
  digitalWrite(HW_HOLD_SWITCH, HIGH);  //  enable pullup resistor (normally high)
  
  digitalWrite(HW_STATUS_LED,LOW);
  digitalWrite(HW_TRIGGER,LOW);

  // disable unused peripherie
  // disable ADC
  ADCSRA = 0;  
  power_adc_disable ();
  // disable Universal Serial Interface 
  power_usi_disable();
}
 
void loop(){
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
    default:
      // TODO go to sleep LED NEEDS BLINKING NO INTERRUPT FROM HOLD
      break;
  }
  statusLed();
}

/*!
 * \brief resetProgram Prepares all global variables necessary to start reading (executeing) or writeing (receiveing) a program.
 */
void resetProgram(){
  program_pointer = 0; // pointer to the current function
  read_pointer = 0; // pointer to the current read/write position
  shot_counter = 0; // number of photos taken with the current function
  last_shot_ms = 0; // last time a photo was taken
  //f_nm1 = -1; // last value of the recursive function determineing the delay between shots
  // I set that one to -1 because if it does not get set before the first calculation, something went wrong since -1 is not valide.
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
bool toggleStatusLedIf(int periode){
  if(millis() - status_led_last_toggle_ms > periode){
    toggleStatusLed();
    return true;
  }
  return false;
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
        if(toggleStatusLedIf(LED_STATE_HOLD_ON_MS)){
          num_runs_STATUS_LED++;
        }
      }else if(num_runs_STATUS_LED > LED_STATE_HOLD_RUN){
        if(toggleStatusLedIf(LED_STATE_HOLD_PAUSE_MS)){
           num_runs_STATUS_LED = 0;           
        }
      }else{
        toggleStatusLedIf(LED_STATE_HOLD_OFF_MS);
      }
    break;      
    case STATE_RUN:
      if(led_is_on){
        toggleStatusLed();
      }
      break;
    default:
      // ERROR CASES
      // 0xFF-ERROR_CASE_HEX is the number of blinks before pause.
        if(led_is_on){
          if(num_runs_STATUS_LED > (0xFF - state_prog)){
            if(toggleStatusLedIf(LED_ERROR_PAUSE_MS)){
               num_runs_STATUS_LED = 0;           
            }
          }else{
            if(toggleStatusLedIf(LED_ERROR_ON_MS)){
              num_runs_STATUS_LED++;
            }
          }
        }else{
          toggleStatusLedIf(LED_ERROR_OFF_MS);
        }
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
 * If the given Hex does not belong to a function, -1 will be returned.
 */
int getNumFunctionValues(byte function){
 switch (function) {
    case SYMBOL_F_CONST:
    return (int) NUM_VALUES_F_CONST;
    case SYMBOL_F_LIN:
    return (int) NUM_VALUES_F_LIN;
    case SYMBOL_F_QUAD:
    return (int) NUM_VALUES_F_QUAD;
    case SYMBOL_STOP:
    case SYMBOL_STOP_SHUTDOWN:
    return 0; // End, no more values
 }
 return -1;
}

/*!
 * \brief runIntervallometer Runs the main program.
 * It expects that the program pointers to be set to zero prior the first call and
 * not altered while the program runs. A class structure would be nice but eh...
 */
void runIntervallometer(){
  // program[program_pointer]: points to the current function
  // shot_counter: how many pictures where taken with the current function
  // last_shot_ms: when the last shot was taken
  if(program_pointer == 0 && shot_counter == 0){
    // Take the very first picture without waiting.
    // This also sets last_shot_ms to a valide value.
    // The user must activate the hold switch before bluetooth transmittion to not start imediately.
    takePhoto();
    // dont count the very first picture
    shot_counter = 0;
    // now prepare the next ms_wait
  }
  if(shot_counter >= readUnsignedProgramValueAt(program_pointer + getPtrBeginOffsetNumShots())){
    setProgramPointerToNextFunction();
    shot_counter = 0; // reset to count the shots of the next function
  }
  if(last_shot_ms > millis()){
    state_prog = STATE_ERROR_MILLIS_OVERFLOW;
    return;
  }
  long next_delay;
  switch (program[program_pointer]) {
    case SYMBOL_F_CONST:
      next_delay = getShutterDelayConst();
      break;
    case SYMBOL_F_LIN:
      next_delay = getShutterDelayLin();
      break;
    case SYMBOL_F_QUAD:
      next_delay = getShutterDelayQuad();
      break;
    case SYMBOL_STOP:
      state_prog = STATE_NO_INSTRUCTIONS;
      // reset pointers for next read
      resetProgram();
      enableBluetooth(true);
      // TODO ENABLE BLUETOOTH
      return;
    case SYMBOL_STOP_SHUTDOWN:
      state_prog = STATE_NO_INSTRUCTIONS;
      // TODO SHUTDOWN or go in sleep
      return;
  }

  if(next_delay < 0){
    state_prog = STATE_ERROR_NEGATIVE_DELAY;
    return;
  }
  // TODO millis() is on hold while snorring so it did not cout TRIGGER_DURATION_MS?
  const unsigned long ms_since_last_pic = millis() - last_shot_ms;
  const unsigned long ms_wait = next_delay - (ms_since_last_pic + TRIGGER_DURATION_MS);
  
  if(ms_wait >  0){
    snore(ms_wait);
  }
  takePhoto();
}

/*!
 * \brief takePhoto triggers the trigger to take a photo and keeps track when the foto was taken.
 */
void takePhoto(){
  digitalWrite(HW_TRIGGER,HIGH);
  last_shot_ms = millis();
  shot_counter++;
  snore(TRIGGER_DURATION_MS);
  digitalWrite(HW_TRIGGER,LOW);
}

/*!
 * \brief readProgramValueAt converts beginning from at the next 4 bytes into an long
 */
long readProgramValueAt(int at){
  // unsigned long does not have a shift operator
  return ((long)(program[at]) << 24) 
       | ((long)(program[at+1]) << 16) 
       | ((long)(program[at+2]) << 8) 
       | ((long)(program[at+3]) );
}

/*!
 * \brief readProgramValueAt converts beginning from at the next 4 bytes into an unsigned long
 */
unsigned long readUnsignedProgramValueAt(int at){
  return readProgramValueAt(at);
}

/*!
 * \brief setProgramPointerToNextFunction Increases the program_pointer about the number of bytes defineing the current function it points at + 1.
 */
void setProgramPointerToNextFunction(){
  program_pointer += getNumFunctionValues(program[program_pointer]) * (int) NUM_BYTES_VALUE + 1;
}

/*!
 * \brief writeValueAt Writes val into bytes.
 * \param bytes Pointer to at byte array with at least 4 more places.
 * \param val The value to wirite into the byte array.
 */
void writeValueAt(byte *bytes, long val){
  *bytes = (byte)((val >> 24) & 0xFF);
  bytes++;
  *bytes = (byte)((val >> 16) & 0xFF);
  bytes++;
  *bytes = (byte)((val >> 8) & 0XFF);
  bytes++;
  *bytes = (byte)((val & 0XFF));
} 

/*!
 * \brief getShutterDelayConst Calculates the shutter delay for the n-th picture for a constant function.
 * \return how long to delay until the next picture should be taken in ms.
 */
long getShutterDelayConst(){
  //[ FUNC_SYMBOL | NUM_PICS | C ]
  // f(n) = f(n-1); f(0) = C;

  return readProgramValueAt(program_pointer + getPtrBeginOffsetC())/FLOAT_PRECISION;
  // recursve
  /*
  if(shot_counter == 0){
    f_nm1 = readProgramValueAt(program_pointer + getPtrBeginOffsetC())/FLOAT_PRECISION;
  }
  return f_nm1;
  */
}

/*!
 * \brief getShutterDelayLin Calculates the shutter delay for the n-th picture for a linear function.
 * \return how long to delay until the next picture should be taken in ms.
 */
long getShutterDelayLin(){
  //[ FUNC_SYMBOL | NUM_PICS | C | B ]
  // f(0) = C
  // f(n) = f(n-1) + B
  // f(n) = C + n*B
  return (  readProgramValueAt(program_pointer + getPtrBeginOffsetC())
          + shot_counter * readProgramValueAt(program_pointer + getPtrBeginOffsetB())
         )/FLOAT_PRECISION;
  /*
  // recursive
  if(shot_counter == 0){
    f_nm1 = readProgramValueAt(program_pointer + getPtrBeginOffsetC())/FLOAT_PRECISION;
  }else{
    f_nm1 = f_nm1 + readProgramValueAt(program_pointer + getPtrBeginOffsetB())/FLOAT_PRECISION;
  }
  return f_nm1;
  */
}

/*!
 * \brief getShutterDelayLin Calculates the shutter delay for the n-th picture for a quadratic function.
 * \return how long to delay until the next picture should be taken in ms.
 */
long getShutterDelayQuad(){
  //[ FUNC_SYMBOL | NUM_PICS | C | B | A ]
  // f(0) = C
  // f(n) = f(n-1) + A*(n-1) + B
  // f(n) = C + 0.5*A*(n)*(n-1) + B*n

  const long nn = shot_counter*(shot_counter-1);

  return (readProgramValueAt(program_pointer + getPtrBeginOffsetC())
          + 0.5*readProgramValueAt(program_pointer + getPtrBeginOffsetA())*nn
          + shot_counter*readProgramValueAt(program_pointer + getPtrBeginOffsetB()))/FLOAT_PRECISION;

  /*
  // recursive
  if(shot_counter == 0){
    f_nm1 = readProgramValueAt(program_pointer + getPtrBeginOffsetC())/FLOAT_PRECISION;
    return f_nm1;
  }else{
    f_nm1 = f_nm1
            + (
            + readProgramValueAt(program_pointer + getPtrBeginOffsetA())*(shot_counter-1)
            + readProgramValueAt(program_pointer + getPtrBeginOffsetB())
            )/FLOAT_PRECISION;
  }
  return f_nm1;
  */
}


void enableBluetooth(bool enable){
  if(enable){
    // todo connect hardware
    setupBlueToothConnection();
  }else{
    shutDownBluetooth();
    // todo disconnect hardware
  }
}
 
void setupBlueToothConnection(){
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

  // to fill the 3 missing bytes
  byte bytes[3];
  write(bytes, 3);
}

/*!
 * \brief isSymbolStopAtProgramCounter
 * \return True if the currend Symbol the program_pointer points to is a Stop Symbol
 */
bool isSymbolStopAtProgramCounter(){
  return (program[program_pointer] == SYMBOL_STOP || program[program_pointer] == SYMBOL_STOP_SHUTDOWN);
}


unsigned long begin_read = 0;
/*!
 * \brief read Reads the next bit send via bluetooth if avaiable and
 * CHANGES the state to STATE_HOLD if the SYMBOL_STOP was received.
 */
void read(){
  //check if there's any data sent from the remote bluetooth shield
  if(blueToothSerial.available()){
    if(begin_read == 0){
      begin_read = millis();
    }else{
      if(millis() - begin_read > MAX_RECEIVING_TIME_MS){
        state_prog = STATE_ERROR_RECEIVING_TOO_LONG;
        return;
      }
    }

    state_prog = STATE_RECEIVEING_DATA;
    program[read_pointer] = blueToothSerial.read();

    const int diff = (int) (read_pointer - program_pointer);
    const int num_expected_bytes = getNumFunctionValues(program[program_pointer]) * (int) NUM_BYTES_VALUE;
    if(diff == num_expected_bytes && !isSymbolStopAtProgramCounter()){
      // We read the last byte of the current function and it was not the SYMBOL_STOP. 
      // Set program_pointer to the position were the next function will be.
      setProgramPointerToNextFunction();
    }else if(diff > num_expected_bytes){
      // For some reason the read_pointer is outside the function at which program_pointer points.
      // That means, we somehow jumped over if(diff == num_expected_bytes).
      // So either read_pointer or program_pointer were altered without my consent.
      state_prog = STATE_ERROR_UNCOOL_READ_POINTER_BEHAVIOUR;
      return;
    } else if(diff < 0){
      // That means that the program_pointer is ahead of the read_pointer which should never happen.
      state_prog = STATE_ERROR_UNCOOL_READ_POINTER_BEHAVIOUR;
      return;   
    }else if(diff == 0){
      // We just read the next Function Symbol
      if(isSymbolStopAtProgramCounter()){
        // Stop symbol received; transmission successful.
        // Reset the program pointers.
        resetProgram();
        checkIfHold();
        enableBluetooth(false);
        return;
      }else if(num_expected_bytes < 0){
        // We read a new Function Symbol but it was not a valide Symbol.
        // If the number of expected bytes is below zero a unknown Symbol was transmitted.
        state_prog = STATE_ERROR_UNKNOWN_SYMBOL_REQUESTED;

        /*
        byte data[4];
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = program[program_pointer];
        write(data, 4);*/

        return;
      }
    }

    read_pointer++;
    if(read_pointer >= NUM_FUNCTION_BYTES){
      state_prog = STATE_ERROR_PROGR_TOO_LONG;
      return;
    }
  }
}

void write(byte *data, int length){
  blueToothSerial.write(data, length);
}


/*!
 * \brief shutDownBluetooth releases the Pins from beeing RX and Tx and makes the gpios
 */
void shutDownBluetooth(){
  blueToothSerial.end();
}
