//
//  Name:       slave2.ino
//  Date:       17 Oct 2024
//  Brief:      Radio-control Slave device
//  Author:     Simon Hopper
//
// ***********************************************************************************************
// Revisions:
//  Date        rev.    	who     what
//	17/10/2024	v2.01.0b	SJH		Add back servo control & fail-safe detection
//                                  Working code
//	14/10/2024	v2.01.0a	SJH		Add more version data
//  11/10/2024  vN      	SJH     Add SX1278 fault states
//                              	Add SX1278 initialisation code, sets XS1278 to 'listen' and receives a data message from master.
//  28/09/2024  vM      	SJH     Create v2 based on a state machine approach
//                              	Now have an FSM for OLED display tasks, ADC sampling and SX1278 control
//                              	No SX1278 functionality, but display & ADC work OK.
//  02/09/2024  vL      	SJH     Various spelling corrections
//	07/03/2024  vK      	SJH     Rebuild with known library revisions & invert rudder & 'planes servo action
//  08/02/2022          	SJH     Replace swFailSafe with constant SWFAILSAFE
//  06/02/2022  vJ      	SJH     Add code to the digital channels to allow easy inversion
//  13/10/2021  vH      	SJH     Add internal ADC read to measure the 12V battery voltage
//  09/05/2021          	SJH     Correct addition of BIAS to the servo message value to maintain a range 0 to 255
//  08/05/2021  vG      	SJH     Reduce servo pulse range from 0.58msec-2.5msec to 1msec-2msec
//                              	Add a bias value to individual servo message to compensate for individual devices
//  02/04/2021  vF      	SJH     Invert digital output 6 so it is normally ON, but turns OFF when activated, this will give a controllable
//                              	fail-safe output that will release when commanded or when the battery voltage fails.
//  27/02/2021          	SJH     Add delay before sending ACK
//                              	Add Rx Fail count before declaring FAILSAFE mode
//                              	Add display of RF channel number
//                              	Set bits in status byte returned to Master for Rx time-out, Fail-safe & message error, reset
//                              	using digital channel 7
//  26/02/2021          	SJH     Finish of using OLED display & displaying RF parameters on a scrolling display
//                              	Reduce the amount of data being sent to console serial port
//                              	Reduce the number of analogue channels to 4
//                              	Replace the '#define' with 'const ...'
//  25/02/2021  vE      	SJH     Add a different OLED library that fits in the FLASH
//  24/02/2021          	SJH     Add the rest of the digital channel controls
//  23/02/2021  vD      	SJH     Add code to output some of the digital channels
//                              	Remove OLED display code as it takes up too much memory
//  20/02/2021          	SJH     Add code to read hex DIL switch to define RF channel
//                              	Add I2C address definitions, even though currently default
//  13/02/2021  vC      	SJH     Move PWM calibration to separate function
//                              	Add ability to invert servo direction
//                              	Add revision as string constant
//  07/02/2021          	SJH     Add PCA9685 oscillator calibration code
//                          	    Comment out most print statements as they take too much time to process
//  06/02/2021  vB      	SJH     Add servo control for analogue channels
//  25/01/2021          	SJH     initial basic version
//
// ***********************************************************************************************
//
// This expects a series of message packets containing message header, servo position data plus 8 bits of ON/OFF data.
// Currently the message header is always 0x5A (MSG_CMND), but could be expanded e.g. to send fail-safe settings.
// In response a 2-byte message is returned, comprising an 'ACK' plus 8-bits of status data
//
// Uses LoRa mode as I can't get FSK to work.
//
// It uses the RadioLib library to provide the control for the SX1278 RF module, with control from a
// 3.3V Arduibo Pro-mini.
//
// It uses adafruit PWM library to control PCA9685 16-channel PWM controller
//
// It uses the SSD1306Ascii library by Bill Greiman as the adafruit library is too large
//
// module                       version
// Arduino                      3.3V pro-Mini
// Arduino IDE	                v2.3.2
// RadioLib                     v4.4.0
// Wire                         ?
// AdaFruit_PWMServoDriver      v2.4.0
// SSD1306Ascii                 v1.3.1
// SSD1306AsciiWire
//
// ***********************************************************************************************
//

#include "settings2.h"

// include the LoRa library
#include <RadioLib.h>

// include the PCA9685 PWM library
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// includes for SSD1306 OLED driver
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

SSD1306AsciiWire oled;


// SX1278 has the following connections with the Arduino pro-mini:
// NSS pin:   10
// DIO0 pin:  3
// RESET pin: 9
// DIO1 pin:  5
SX1278 radio = new Module(10, 3, 9, 5);

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(I2C_PWM_ADDR);

uint8_t LED1 = 8;         // LED 1
uint8_t HSW1 = 4;         // hex switch bit 1
uint8_t HSW2 = A3;        // hex switch bit 2
uint8_t HSW4 = A2;        // hex switch bit 4
uint8_t HSW8 = A1;        // hex switch bit 8

uint8_t ADC_IN = A6;       // analogue input

uint8_t hexSwitch;        // hex switch value read

// this is the RF channel number
int rfChannel = RF_CH_DEF;

// start & finish times used to calculate the loop time if required
// note that since 32-bits would equal 4,294,967,296 usec (4,294 secs = 70 mins) there is a chance that these could roll-over
// and cause time-out hang-ups.
//
// this needs sorting.
//
uint32_t rxStart,rxFinish,txStart,txFinish;

// declare 6 channels of servo position data, set to mid position as the default
byte servoArr[MAX_ANA_CHAN] = {SERVO_MID,SERVO_MID,SERVO_MID,SERVO_MID,SERVO_MID,SERVO_MID};

// byte used to store 8 x on/off channels, set to OFF as the default, except channel 6 that is used as a fail-safe
byte swByte =0b01000000;

// byte use to store slave status
byte statusByte = 0x00;

//
// return value from library function calls.
// should be ERR_NONE, but if not it contains the error code
int16_t state;

// byte array used to store messages.
// For received this is formed from:
//  message type; e.g. servo position data, fail-safe settings, etc.
//  6 x servo position data, channel 0 first, channel 5 last
//  switch channel data, (1=ON, 0=OFF), bit 0 = switch channel 0, bit 7 = switch channel 7
//
byte msgRxArr[NUM_ANA_CHAN +2];
//
// byte array used to store ACK message
//
byte msgTxArr[LEN_ACKMSG];

bool rxTimeout = false;                             // Rx time-out flag, 1 = timed out
bool txTimeout = false;                             // Tx time-out flag, 1 = timed out
bool rxMsgOk = false;                               // received valid message flag, 1 = OK
bool txMsgOk = false;                               // transmit message sent, 1 = OK

float centreFreq;                                   // RF centre frequency

uint8_t servoNum = 0;                               // current servo number

uint8_t PWM_CAL_IN = 2;                             // input pin for calibration PWM, via 22k series resistor
uint16_t pulseStart,pulseStop;                      // start & stop times for servo PWM
float servoConst[MAX_ANA_CHAN] = {0,0,0,0,0,0};     //
int16_t failSafe[MAX_ANA_CHAN] = {0,0,0,0,0,0};     //
uint16_t totalCounter;                              // PWM calibration cycle counter

int8_t rxFailCount;                                 // counts consecutive Rx fail
int8_t rxTimeoutCount;                              // counts consecutive Rx time-outs
int8_t txTimeoutCount;                              // counts consecutive Tx time-out
//int16_t txCount;                                    //
int8_t displayTask;                                 // current display task

uint16_t ADC_VAL = 0;                               // on-chip ADC value
float v_in;                                         // measured voltage
float v_bat = 12.0;                                 // battery voltage (default valid value)

unsigned long currentTime;                          // current time value

uint8_t loraFsmState;                               // LoRa FSM current state
unsigned long loraOldTime;                          // LoRa FSM original time when timeout set
unsigned long loraTimeOut;                          // LoRa FSM time out value

uint8_t adcFsmState;                                // SX FSM current state
unsigned long adcOldTime;                           // ADC FSM original time when timeout set
unsigned long adcTimeOut;                           // ADC FSM time out value

uint8_t dispFsmState;                               // display FSM current state
unsigned long dispOldTime;                          // display FSM original time when timeout set
unsigned long dispTimeOut;                          // display FSM time out value
//
uint8_t flashRate;                                  // used to flash LED
//
//
//
//
void setup() {
  uint8_t prescale;
  int8_t lastSample;

  // define input pins and their pull-ups
  pinMode(PWM_CAL_IN,INPUT_PULLUP);               // used to calibrate PWM module
  pinMode(HSW1,INPUT_PULLUP);                     // hex switch bit 1
  pinMode(HSW2,INPUT_PULLUP);                     // hex switch bit 2
  pinMode(HSW4,INPUT_PULLUP);                     // hex switch bit 4
  pinMode(HSW8,INPUT_PULLUP);                     // hex switch bit 8

  Serial.begin(115200);

  // initial banner
  Serial.println();
  Serial.print(F("Slave LoRa SX1278 rev: "));
  Serial.println(REV);
  Serial.println();

  // set up OLED display
  oled.begin(&Adafruit128x32, I2C_OLED_ADDR);
  oled.setFont(Callibri14);
  oled.setScrollMode(SCROLL_MODE_AUTO);
  oled.clear();
  oled.print("Slave rev. ");
  oled.println(REV);
  delay(2000);                                // 2 sec delay

  //
  // read hex switch inputs to get RF channel number
  // Since the switch values are 0..15 the RF channel are 3..33.
  // This allows a reasonable spread, but keeps within the value 1..40 range but keeps away from the RF band edges
  // to allow for the RF bandwidth actually used; approx 3 channels.
  hexSwitch = readHexSwitch();
  rfChannel = 2 * int(hexSwitch) + 3;
  oled.print("RF channel: ");
  oled.println(rfChannel);

  // calculate centre frequency from channel number
  centreFreq =CENTRE_FREQ_CH0 + RF_CH_SPACE * rfChannel; 
  Serial.print(F("RF centre frequency: "));
  Serial.print(centreFreq);
  Serial.print(F(" (MHz), RF channel: "));
  Serial.println(rfChannel);
  //
  delay(1000);                                // 1 sec delay

  //txCount = 0;
  displayTask = 0;
  rxFailCount = 0;
  rxTimeoutCount = 0;
  txTimeoutCount = 0;
  //
  // initialise the PCA0685 PWM
  Serial.print(F("[Slave] Setting up PCA0685 PWM module ..."));
  pwm.begin();                              // use default pre-scale
  pwm.setPWMFreq(SERVO_FREQ);               // nominal 50Hz
  // set default channel PWM to mid position or OFF
  setMidRange();
  
  pwm.setPWM(SERVO_CAL,0,2048);             // set to 50% square-wave
  prescale= pwm.readPrescale();

  Serial.print(F("Target frequency: "));
  Serial.println(SERVO_FREQ);  

  totalCounter = 0;                         // PWM cycle counter

  delay(500);                               // allow some time to stabilise
 
  cal_pwm();                                // set up PWM module

  Serial.print(F("calculating servo constants ..."));
  for (servoNum = 0; servoNum < NUM_ANA_CHAN; servoNum++)
  {
    pulseStart = PWMOFF[servoNum];
    pulseStop = PWMOFF[servoNum] + (SERVOMIN[servoNum] + SERVOMAX[servoNum])/2;
    pwm.setPWM(servoNum,pulseStart,pulseStop);
    // calculate the channel constants to save time later
    servoConst[servoNum] = (SERVOMAX[servoNum] - SERVOMIN[servoNum])/ float(SERVOMSGMAX + 1);
    failSafe[servoNum] = 128 + BIAS[servoNum];
  }
  // PWM stuff all done
  Serial.println(F("PWM configured"));
  oled.println("PWM OK");
  delay(1000);                               // 1,000 usec = 1 sec

  oled.println("Configured");

  delay(1000);                                      // 1 sec delay
 
  currentTime = micros();                           // current time value

  loraFsmState = LORA_INIT;                         // initialisation state
  loraOldTime = micros();                           // start time value
  loraTimeOut = ERR_TIMEOUT;                        // 2,000,000 usec = 2 secs

  adcFsmState = ADC_INIT;                           // initialisation state
  adcOldTime = micros();                            // start time value
  adcTimeOut = ADC_TIMEOUT;                         // ADC time out

  dispFsmState = DISP_INIT;                         // initialisation state
  dispOldTime = micros();                           // start time value
  dispTimeOut = DISP_TIMEOUT;                       // display time out

}

//
// flag to indicate that a packet was received.
// it is also used to indicate a packet was transmitted in this ping-pong mode
//
volatile bool receivedFlag = false;

//
// disable interrupt when it's not needed
//
volatile bool enableInterrupt = false;

//
// This function is called when a complete packet is received by the module.
// It is also called when a complete packet has been sent as the same interrupt pin 
// is used
//
// Essentially, if the interrupt enable flag is set then the 'received' flag is set.
// 
// IMPORTANT: this function MUST be 'void' type and MUST NOT have any arguments!
//
void setFlag(void) {
  // check if the interrupt is enabled
  if(enableInterrupt == true) {
	// we got a packet, set the flag
	receivedFlag = true;
  }
  return;
}

//
//
//
//
void loop() {
  int16_t rssi;
  int16_t snr;
  int16_t freq_err;
  char bytestr[1];                                        // used when converting a byte to a hex string

  //
  //  execute LoRa radio FSM
  //
  switch (loraFsmState) {
    case LORA_INIT:
	  // initialisation state
	  lora_Init();
	  break;
	
    case LORA_INITWAIT:
	  // initialisation wait state
	  lora_InitWait();
	  break;
	
	case LORA_RXSTART:
	  // Rx start state
	  lora_RxStart();
	  break;
	  
	case LORA_RXREADY:
	  // Rx ready state
	  lora_RxReady();
	  break;
	  
	case LORA_RXREAD:
	  // Rx read data state
	  lora_RxRead();
	  break;
	  
	case LORA_TXWAIT:
	  // Tx wait state
	  lora_TxWait();	  
	  break;
	  
	case LORA_TXSLEEP:
	  // Tx sleep state
	  lora_TxSleep();
	  break;
	  
	case LORA_TXREADY:
	  // TX ready state
	  lora_TxReady();
	  break;
	  
	case LORA_RXSLEEP:
	  // Rx sleep state
	  lora_RxSleep();
	  break;
	
    case LORA_RXTIMEOUT:
      // Rx time-out state
      lora_RxTimeOut();
      break;
	  
    case LORA_RXERROR:
      // RX data error state
      lora_RxError();
	  break;
	  
	case LORA_TXTIMEOUT:
	 // Tx time-out state
	 lora_TxTimeOut();
	 break;
	 
   case LORA_TXERROR:
     // Tx error state
     lora_TxError();
     break;
     
  	// fault states that flash LED
	case LORA_FLASH:
	  // 'fault' state i.e. LED on
	  lora_flash();
	  break;
	  
	case LORA_FLASH2:
	  // 'fault2' state i.e. LED off
	  lora_flash();
	  break;
	  
	default:
	  // invalid state values
	  lora_Init();
	  break;
  }

  // update current time value
  currentTime = micros();                               // current time value

  // check for any new Rx data messages


  //
  // execute display FSM
  //
  switch (dispFsmState) {
    case DISP_INIT:
	  // initialisation state
	  disp_Init();
	  break;
	  
	case DISP_WRITE:
	  // read state
	  disp_Write();
	  break;
	  
	case DISP_WAIT:
	  // wait state
	  disp_Wait();
	  break;
	  	  
	default:
	  // invalid state values
	  disp_Init();
	  break;
  }  
  //
  // execute ADC FSM
  //
  switch (adcFsmState) {
    case ADC_INIT:
	  // initialisation state
	  adc_Init();
	  break;
	  
	case ADC_READ:
	  // read state
	  adc_Read();
	  break;
	  
	case ADC_WAIT:
	  // wait state
	  adc_Wait();
	  break;
	  	  
	default:
	  adc_Init();
	  break;
  } 
  
}                                                    // end of task loop

//
// process received message data bytes
//
void processRxMessage(){
  // check message command
  if (msgRxArr[0] == MSG_CMND) {
	//
    // valid servo message command so copy the servo message bytes from message to servo data array
	//
    for (servoNum = 0; servoNum < NUM_ANA_CHAN; servoNum++) {
      // check for range 0 to 255
      if (msgRxArr[servoNum +1] + BIAS[servoNum] > 255){
        servoArr[servoNum] = 255;
      }
      else {
        if (msgRxArr[servoNum +1] + BIAS[servoNum] < 0){
          servoArr[servoNum] = 0;
        }
        else {
          servoArr[servoNum] = msgRxArr[servoNum +1] + BIAS[servoNum];
        }
      }
    }
    //
    // copy on/off channel message here
    swByte = msgRxArr[NUM_ANA_CHAN +1];                        // digital switch data

    //
    // for each servo the pulse width is given by:
    //
    //  pulse width = servo_min + servo_val * (servo_max - servo_min) / (msg_max +1)
    //
    // where:
    //  servo_min, servo_max are the minimum & maximum pulse widths acceptable to the servo, found from previous calibration data
    //  servo_val is the requested position message
    //  msg_max is the maximum servo_val value (currently 255)
    //
    // To save time during the processing loop an intermediate value has been calculated during initialisation:
    //  servo_const = (servo_max - servo_min) / (msg_max +1)
    //
    //
    for (servoNum = 0; servoNum < NUM_ANA_CHAN; servoNum++)
    {
      if (SERVOINV[servoNum] == 1) {
        // invert servo direction
        pulseStart = PWMOFF[servoNum];
        pulseStop = pulseStart + SERVOMIN[servoNum] + servoConst[servoNum]* (SERVOMSGMAX - servoArr[servoNum]);
        pwm.setPWM(servoNum,pulseStart,pulseStop);          
      }
      else
      {
        pulseStart = PWMOFF[servoNum];
        pulseStop = pulseStart + SERVOMIN[servoNum] + servoConst[servoNum]* servoArr[servoNum];
        pwm.setPWM(servoNum,pulseStart,pulseStop);
      }
    }
    //
    // do on/off channels here
    // Not very sophisticated but efficient code
	//
    // bit 0
    if ((swByte & 0x01) == DIGITALINV[0]){
      // turn ON
      pwm.setPWM(8,4096,0);                 // fully on
    }
    else {
      // turn OFF
      pwm.setPWM(8,0,4096);                 // fully off
    }
    // bit 1
    if ((swByte & 0x02) == DIGITALINV[1]){
      pwm.setPWM(9,4096,0);                 // fully on
    }
    else {
      pwm.setPWM(9,0,4096);                 // fully off
    }
    // bit 2
    if ((swByte & 0x04) == DIGITALINV[2]){
      pwm.setPWM(10,4096,0);                 // fully on
    }
    else {
      pwm.setPWM(10,0,4096);                 // fully off
    }
    // bit 3
    if ((swByte & 0x08) == DIGITALINV[3]){
      pwm.setPWM(11,4096,0);                 // fully on
    }
    else {
      pwm.setPWM(11,0,4096);                 // fully off
    }
    // bit 4
    if ((swByte & 0x10) == DIGITALINV[4]){
      pwm.setPWM(12,4096,0);                 // fully on
    }
    else {
      pwm.setPWM(12,0,4096);                 // fully off
    }
    // bit 5
    if ((swByte & 0x20) == DIGITALINV[5]){
      pwm.setPWM(13,4096,0);                 // fully on
    }
    else {
      pwm.setPWM(13,0,4096);                 // fully off
    }
    // bit 6
    if ((swByte & 0x40) == DIGITALINV[6]){
      pwm.setPWM(14,0,4096);                 // fully off
    }
    else {
      pwm.setPWM(14,4096,0);                 // fully on
    }
    // bit 7
    if ((swByte & 0x80) == DIGITALINV[7]){
      pwm.setPWM(15,4096,0);                 // fully on
      statusByte = 0x00;                     // clear status byte
    }
    else {
      pwm.setPWM(15,0,4096);                 // fully off
    }
    //
    // clear status error bits?
    //


  } else {
    // some other error occurred whilst reading data
    Serial.print(F("[Slave] Failed data read, code "));
    Serial.println(state,HEX);
    //
    // set error bit in status byte here
    statusByte = statusByte | FLG_MSGERR;
  }	
}

//
// set mid-range values for all PWM channels, 'off' for all digital channels
//
void setMidRange(){
  for (servoNum = 0; servoNum < NUM_ANA_CHAN; servoNum++) 
  {
    pulseStart = PWMOFF[servoNum];
    pulseStop = PWMOFF[servoNum] + (SERVOMIN[servoNum] + SERVOMAX[servoNum])/2;
    pwm.setPWM(servoNum,pulseStart,pulseStop);
    pwm.setPWM(servoNum,pulseStart,pulseStop);
  }

  // now set 'off' values for the digital channels
  pwm.setPWM(8,0,4096);                  // fully off
  pwm.setPWM(9,0,4096);                  // fully off
  pwm.setPWM(10,0,4096);                 // fully off
  pwm.setPWM(11,0,4096);                 // fully off
  pwm.setPWM(12,0,4096);                 // fully off
  pwm.setPWM(13,0,4096);                 // fully off
  pwm.setPWM(14,0,4096);                 // fully off
  pwm.setPWM(15,0,4096);                 // fully off

}


//
// set fail-safe values for all PWM channels
//
void setFailSafe(){
  for (servoNum = 0; servoNum < NUM_ANA_CHAN; servoNum++) 
  {
    pulseStart = PWMOFF[servoNum];
    pulseStop = pulseStart + SERVOMIN[servoNum] + servoConst[servoNum]* failSafe[servoNum];
    pwm.setPWM(servoNum,pulseStart,pulseStop);
  }

   // now set fail-safe values for the digital channels
   // bit 0
   if (SWFAILSAFE & 0x01){
     // turn ON
     pwm.setPWM(8,4096,0);                 // fully on
   }
   else {
     // turn OFF
     pwm.setPWM(8,0,4096);                 // fully off
   }
   // bit 1
   if (SWFAILSAFE & 0x02){
     // turn ON
     pwm.setPWM(9,4096,0);                 // fully on
   }
   else {
     // turn OFF
     pwm.setPWM(9,0,4096);                 // fully off
   }
   // bit 2
   if (SWFAILSAFE & 0x04){
     // turn ON
     pwm.setPWM(10,4096,0);                 // fully on
   }
   else {
     // turn OFF
     pwm.setPWM(10,0,4096);                 // fully off
   }
   // bit 3
   if (SWFAILSAFE & 0x08){
     // turn ON
     pwm.setPWM(11,4096,0);                 // fully on
   }
   else {
     // turn OFF
     pwm.setPWM(11,0,4096);                 // fully off
   }
   // bit 4
   if (SWFAILSAFE & 0x10){
     // turn ON
     pwm.setPWM(12,4096,0);                 // fully on
   }
   else {
     // turn OFF
     pwm.setPWM(12,0,4096);                 // fully off
   }
   // bit 5
   if (SWFAILSAFE & 0x20){
     // turn ON
     pwm.setPWM(13,4096,0);                 // fully on
   }
   else {
     // turn OFF
     pwm.setPWM(13,0,4096);                 // fully off
   }
   // bit 6
   if (SWFAILSAFE & 0x40){
     // turn ON
     pwm.setPWM(14,4096,0);                 // fully on
   }
   else {
     // turn OFF
     pwm.setPWM(14,0,4096);                 // fully off
   }
   // bit 7
   if (SWFAILSAFE & 0x80){
     // turn ON
     pwm.setPWM(15,4096,0);                 // fully on
   }
   else {
     // turn OFF
     pwm.setPWM(15,0,4096);                 // fully off
   }
}


//
// ************ LoRa FSM states ******************
//
//


//
// LoRa Initialisation state	(LORA_INIT)
//
// Allow an initial delay to allow everything to settle.
// Set the radio state to 'begin', if successful change state to 'Init wait'.
// If not successful change state to 'fast flash' as it is a fatal error
//
void lora_Init() {
  // check timer
  if (checkTimeout(loraOldTime,loraTimeOut)) {
    // timer expired
    Serial.println(F("[Slave] Initializing LoRa ... "));
    state = radio.begin(centreFreq,BANDWIDTH,SPREADINGFACTOR,CODERATIO,SYNC_WORD,TXPMAX,NUM_PREAMBLE,RXGAIN);
    //
    // is module configured?
    if (state == ERR_NONE) {
      //
	  // module configured
	  //
      Serial.println(F("[Slave] SX_INIT success change to LORA_INITWAIT ... "));
      loraOldTime = micros();                     // start time
      loraTimeOut = 0;                            // delay time not used
      loraFsmState = LORA_INITWAIT;               // change state
    } else {
      //
	  // module failed to configure
	  //
      oled.print("*** Failed to config SX1278 ***\n");
      // set to 'failed' state
      Serial.println(F("[Slave] LORA_INIT failed to initialise change to LORA_FAULT ... "));
      Serial.print(F("failed, code "));
      Serial.println(state,HEX);

      loraOldTime = micros();                     // start time
      loraTimeOut = FASTFLASH;                    // delay time = fast flash period
      flashRate = FASTFLASH;                      // set flash rate
      loraFsmState = LORA_FLASH;                  // change state
    }
  }
}


//
// LoRa Initialisation wait state	(LORA_INITWAIT)
//
// Set the radio interrupt action, then change state to 'Rx start'
//
void lora_InitWait() {
  // set the function that will be called when new packet is received
  radio.setDio0Action(setFlag);

  loraOldTime = micros();                         // start time
  loraTimeOut = 0;                                // delay time not used
  loraFsmState = LORA_RXSTART;                    // change state
}


//
// LoRa receive start state	(LORA_RXSTART)
//
// Set the radio to 'listen'
// If successful change state to 'rx start', if not change state to 
// 'medium flash' as a fatal error.
//
// The LED is set 'off' whilst in 'listen' mode
//
void lora_RxStart(){
  // start listening for LoRa packets
//  Serial.print(F("[Slave] Starting to listen ... "));
  
  state = radio.startReceive();
  if (state == ERR_NONE) {
	//
	// successfully set radio to 'listen'
	//
    digitalWrite(LED1, LOW);                      // set LED OFF

    receivedFlag = false;                         // clear interrupt flag
    enableInterrupt = true;                       // enable interrupt service routine
  
    // set to 'success' state
    loraOldTime = micros();                       // start time
    loraTimeOut = RX_TIMEOUT;                     // delay time = 2,000,000 usec = 2sec
    loraFsmState = LORA_RXREADY;                  // change state
  } else {
	//
    // failed to set radio to 'listen'
	//
    oled.print("*** Failed to start Rx ***\n");

    Serial.println(F("[Slave] LORA_RXDELAY failed to set 'listen' change to LORA_FLASH ... "));
    Serial.print(F("failed, code "));
    Serial.println(state,HEX);

    loraOldTime = micros();                       // start time
    loraTimeOut = MEDFLASH;                       // delay time = medium flash period
    flashRate = MEDFLASH;                         // set flash rate
    loraFsmState = LORA_FLASH;                    // change state
  }
}


//
// LoRa Rx ready state	(LORA_RXREADY)
//
// Wait for interrupt flag indicating that a message has been received.
// If no message is received within the time-out change state to 'Rx time-out'.
// When a message is received change state to 'Rx read'
//
void lora_RxReady(){
  if (receivedFlag == true) {
	//
    // Rx message received
	//
    // disable the interrupt service routine while processing the data
    enableInterrupt = false;                      // disable further interrupts
    receivedFlag = false;                         // clear interrupt flag
    
	rxTimeoutCount = 0;                           // clear rxTimeoutCount
    rxFinish = micros();                          // rx finish time

    loraOldTime = micros();                       // start time
    loraTimeOut = 0;                              // delay time not used
    loraFsmState = LORA_RXREAD;                   // change state
  } else {
	//
	// no received data so check timer
    //
    if (checkTimeout(loraOldTime,loraTimeOut)) {
      //
      // timer expired
	  //
	  // block any further interrupts while we process the time-out
      enableInterrupt = false;                    // disable interrupts
      receivedFlag = false;                       // clear interrupt flag
	  rxTimeout = true;                           // set Rx time-out flag

      if (rxTimeoutCount < 100){
		rxTimeoutCount = rxTimeoutCount +1;       // rxTimeoutCount counts up to 100
	  }
	  if (rxTimeoutCount > FSAFE_COUNT){
        Serial.println(F("[Slave] Rx time-outs exceeded set Fail-safe ... "));
		setFailSafe();                            // set fail-safe conditions
	  }
	  
      Serial.println(F("[Slave] LORA_RXREADY expired change to LORA_RXTIMEOUT ... "));
      loraOldTime = micros();                     // start time
      loraTimeOut = ERR_TIMEOUT;                  // delay time = 2,000,000 usec = 2 sec
      loraFsmState = LORA_RXTIMEOUT;              // change state
    }
  }
}


//
// LoRa read data state	(LORA_RXREAD)
//
// Read received message into array msgRxArr.
// If the read is successful change state to 'Tx wait',
// if not change state to 'Rx error'
//
void lora_RxRead(){
  //
  // read received data as byte array
  state = radio.readData(msgRxArr, NUM_ANA_CHAN+2);
  if (state == ERR_NONE){
	//
    // packet was successfully received
	//
    rxMsgOk = true;                               // set flag to indicate valid received packet
	rxFailCount = 0;                              // reset Rx fail counter
	// 
	// now update control channels with new message data
	processRxMessage();
	
    loraOldTime = micros();                       // start time
    loraTimeOut = DIR_TIMEOUT;                    // delay time = 20,000 usec = 20msec
    loraFsmState = LORA_TXWAIT;                   // change state
  } else {
	//
	// packet was not successfully read
	//
	if (rxFailCount < 100) {
	  rxFailCount = rxFailCount +1;               // rxFailCount counts up to 100
    }
    Serial.println(F("[Slave] LORA_RXREAD failed change to LORA_RXERROR ... "));
    loraOldTime = micros();                       // start time
    loraTimeOut = ERR_TIMEOUT;                    // delay time = 2,000,000 usec = 2sec
    loraFsmState = LORA_RXERROR;                  // change state
  }
}


//
// LoRa Tx wait state	(LORA_TXWAIT)
//
// This is a short delay to allow the master to change from 'transmit' to 'receive'
// and be ready to receive the ACK message back.
//
// The slave radio is set to 'sleep' as an interim state between 'listen' and
// 'transmit'
//
// The LED is set ON whilst transmitting the 'ACK' message
//
void lora_TxWait(){
  // check timer
  if (checkTimeout(loraOldTime,loraTimeOut)) {
    // timer expired
//    Serial.println(F("[Slave] LORA_TXDELAY expired change to LORA_SLEEP ... "));

    digitalWrite(LED1, HIGH);                     // set LED ON whilst sending ACK message
    
    state = radio.sleep();                        // set to 'sleep' in between Rx & Tx modes
    // set up ACK message
    msgTxArr[0] = MSG_ACK;
    msgTxArr[1] = statusByte;

    loraOldTime = micros();                       // start time
    loraTimeOut = 0;                              // delay time not used
    loraFsmState = LORA_TXSLEEP;                  // change state
  }
}


//
// LoRa Tx sleep state	(LORA_TXSLEEP)
//
// Set the radio to 'transmit' to return the 'ACK' message.
// If the radio fails to be set to 'transmit' change state to 'Tx error'.
//
void lora_TxSleep(){
  // set radio to 'transmit'
  state = radio.startTransmit(msgTxArr, LEN_ACKMSG);
  if (state == ERR_NONE) {
	//
	// radio successfully set to 'transmit'
	//
//    txTimeout = false;                            // clear Tx time-out flag
    receivedFlag = false;                         // clear interrupt flag
    enableInterrupt = true;                       // enable interrupts

    loraOldTime = micros();                       // start time
    loraTimeOut = TX_TIMEOUT;                     // delay time = 2,000,000 usec = 2 sec
    loraFsmState = LORA_TXREADY;                  // change state
  } else {
	//
	// failed to set radio to 'transmit'
	//
    Serial.println(F("[Slave] LORA_TXSLEEP failed change to SX_TXERRR ... "));
    loraOldTime = micros();                       // start time
    loraTimeOut = ERR_TIMEOUT;                    // delay time = 2,000,000 usec = 2 sec
    loraFsmState = LORA_TXERROR;                  // change state  
  }
}


//
// LoRa Tx ready state	(LORA_TXREADY)
//
// Wait for the interrupt flag to indicate the 'ACK' message has been sent, then
// change state to 'Rx sleep'.
// If the 'ACK' message is not sent within the time-out then change the 
// state to 'Tx time-out'
//
void lora_TxReady() {
  if (receivedFlag == true){
    //
    // 'ACK' message sent
    //
    // block any further interrupts while we process the tx data
    enableInterrupt = false;                      // disable interrupts
    receivedFlag = false;                         // clear interrupt flag
    txMsgOk = true;                               // set true when a valid message is sent
	txTimeoutCount = 0;                              // reset Tx fail count

    loraOldTime = micros();                       // start time
    loraTimeOut = 0;                              // delay time not used
    loraFsmState = LORA_RXSLEEP;                  // change state
  } else {
    // check timer
    if (checkTimeout(loraOldTime,loraTimeOut)) {
      //
      // timer expired
	  //
	  // block any further interrupts while we process the time-out
      enableInterrupt = false;                      // disable interrupts
      receivedFlag = false;                         // clear interrupt flag
	  txTimeout = true;                             // set Tx time-out flag
	
	  if (txTimeoutCount < 100) {
		txTimeoutCount = txTimeoutCount +1;               // txTimeoutCount counts up to 100
	  }
      Serial.println(F("[Slave] LORA_TXREADY expired change to LORA_TIMEOUT ... "));
      loraOldTime = micros();                       // start time
      loraTimeOut = ERR_TIMEOUT;                    // delay time = 2,000,000 usec = 2 sec
      loraFsmState = LORA_TXTIMEOUT;                // change state
    }
  }
}


//
// LoRa Rx sleep state	(LORA_RXSLEEP)
//
// Set the radio to 'sleep' as an interim state between 'transmit' and 'listen'
// Change state to 'Rx start'.
//
void lora_RxSleep(){
  radio.sleep();                                  // set to 'sleep' in between Tx & Rx modes
  	
  loraOldTime = micros();                         // start time
  loraTimeOut = 0;                                // delay time not used
  loraFsmState = LORA_RXSTART;                    // change state
}


//
// LoRa Rx time-out state	(LORA_RXTIMEOUT)
//
// No received message within the time-out so simply reset radio to 'listen'
// Change state to 'Rx sleep'
//
void lora_RxTimeOut(){
  if (checkTimeout(loraOldTime,loraTimeOut)) {
    // timer expired
    Serial.println(F("[Slave] LORA_RXTIMEOUT expired change to LORA_RXSLEEP ... "));
    loraOldTime = micros();                       // start time
    loraTimeOut = 0;                              // delay time not used
    loraFsmState = LORA_RXSLEEP;                  // change state
  }
}


//
// LoRa Rx error state	(LORA_RXERROR)
//
// Error detected in received message string, but after setting error flags
// simply reset to 'listen' and try again.
//
void lora_RxError(){
  if (checkTimeout(loraOldTime,loraTimeOut)) {
    // timer expired
    // check error code
    if (state == ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("[Slave] LORA_RXERROR CRC error!"));
      //
      // set error bit in status byte here
      statusByte = statusByte | FLG_MSGERR;

    } else {
      // some other error occurred whilst reading data
      Serial.print(F("[Slave] LORA_RXERROR Failed data read, code "));
      Serial.println(state,HEX);
      //
      // set error bit in status byte here
      statusByte = statusByte | FLG_MSGERR;
    }
	
    Serial.println(F("[Slave] LORA_RXERROR change to LORA_RXSLEEP ... "));
    loraOldTime = micros();                       // start time
    loraTimeOut = 0;                              // delay time not used
    loraFsmState = LORA_RXSLEEP;                  // change state
  }
}


//
// LoRa Tx time-out state	(LORA_TXTIMEOUT)
//
// No 'ACK' message sent within the time-out so simply reset radio back to 'listen'
// and try again.
// Change state to 'Rx sleep'
//
void lora_TxTimeOut(){
  if (checkTimeout(loraOldTime,loraTimeOut)) {
    // timer expired
    Serial.println(F("[Slave] LORA_TXTIMEOUT change to LORA_RXSLEEP ... "));
    loraOldTime = micros();                       // start time
    loraTimeOut = 0;                              // delay time not used
    loraFsmState = LORA_RXSLEEP;                  // change state
  }
}


//
// LoRa Tx error state (LORA_TXERROR)
//
// Radio failed to be set to 'transmit' so simply reset to 'listen'
// and try again.
// Change state to 'Rx sleep'
//
void lora_TxError(){
  if (checkTimeout(loraOldTime,loraTimeOut)) {
    // timer expired
    Serial.println(F("[Slave] LORA_TXERROR change to LORA_RXSLEEP ... "));
    loraOldTime = micros();                       // start time
    loraTimeOut = 0;                              // delay time not used
    loraFsmState = LORA_RXSLEEP;                  // change state
  }
}


//
// LoRa LED flash state	(LORA_FLASH)
//
void lora_flash(){
  digitalWrite(LED1,HIGH);
  // check timer
  if (checkTimeout(loraOldTime,loraTimeOut)) {
    // timer expired
    loraOldTime = micros();                       // start time
    loraTimeOut = flashRate;                      // delay time = flash period
    loraFsmState = LORA_FLASH2;                   // change state
  }
}

//
// LoRa LED flash 2 state	(LORA_FLASH2)
//
void lora_flash2(){
  digitalWrite(LED1,LOW);
  // check timer
  if (checkTimeout(loraOldTime,loraTimeOut)) {
    // timer expired
    loraOldTime = micros();                       // start time
    loraTimeOut = flashRate;                      // delay time = flash period
    loraFsmState = LORA_FLASH;                    // change state
  }
}

//
// ************ ADC FSM states ******************
//
//

//
// ADC Initialisation state
//
void adc_Init() {
  // check timer
  if (checkTimeout(adcOldTime,adcTimeOut)) {
    // timer expired
    Serial.println(F("[Slave] ADC_INIT expired change to ADC_READ ... "));
    adcOldTime = micros();                        // start time
    adcTimeOut = ADC_TIMEOUT;                     // delay time = ADC time out
    adcFsmState = ADC_READ;                       // change state
  }
}

//
// ADC Read state
//
void adc_Read() {
  ADC_VAL = analogRead(ADC_IN);                       // internal ADC
  v_in = float(ADC_VAL) * ADC_FSD/1023;               // measure voltage
  v_bat = v_in * ADC_ATTEN;                           // actual battery voltage

  if (v_bat < LOW_BAT){
    statusByte = statusByte | FLG_LOWBAT;
    Serial.print(F("[Slave] Low battery, "));
    Serial.println(v_bat);
  } else {
    Serial.print(F("[Slave] v_bat = "));
    Serial.println(v_bat);
  }
  
//  Serial.println(F("[Slave] ADC_READ expired change to ADC_WAIT ... "));
  adcOldTime = micros();                             // start time
  adcTimeOut = ADC_TIMEOUT;                          // delay time = ADC time outc
  adcFsmState = ADC_WAIT;                            // change state

}

//
// ADC Wait state
//
void adc_Wait() {
  // check timer
  if (checkTimeout(adcOldTime,adcTimeOut)) {
    // timer expired
//    Serial.println(F("[Slave] ADC_WAIT expired change to ADC_READ ... "));
    adcOldTime = micros();                        // start time
    adcTimeOut = ADC_TIMEOUT;                     // delay time = ADC time out
    adcFsmState = ADC_READ;                       // change state
  }
}


//
// ************ Display FSM states ******************
//
//

//
// display Initialisation state
//
void disp_Init() {
  // check timer
  if (checkTimeout(dispOldTime,dispTimeOut)) {
    // timer expired
    Serial.println(F("[Slave] DISP_INIT expired change to DISP_WRITE ... "));
    dispOldTime = micros();                        // start time
    dispTimeOut = DISP_TIMEOUT;                    // delay time = display time out
    dispFsmState = DISP_WRITE;                     // change state
  }
}

//
// display Write state
//
// every pass through this state the OLED display is updated to the next display field
//
void disp_Write() {
  int16_t rssi;
  int16_t snr;
  int16_t freq_err;
  char bytestr[1];                                        // used when converting a byte to a hex string
  //
  // check for Rx Timeouts and/or 'failsafe' mode
  //
  if (rxTimeoutCount > 0){
    if (rxTimeoutCount > FSAFE_COUNT){
	  // in 'failsafe' mode
      Serial.println(F("[Slave] **FAILSAFE**"));
      oled.println("**FAILSAFE**");
	} else {
      // not yet reached the count limit
      oled.println("**Rx Timeout **");		
	}
  } else {
	//
	// if not, display selected data
	//
    switch(displayTask){
      case 0:
        // print received packet data
        Serial.print(F("[Slave] Data:\t\t"));
        for (uint8_t i=0; i< NUM_ANA_CHAN+2;i++){
          Serial.print(msgRxArr[i],HEX);
          Serial.print(F(" "));
          if (i>=1){
            byte2str(bytestr,msgRxArr[i]);                // don't bother printing byte[0]
            oled.print(bytestr);
            oled.print(" ");
          }
        }
        Serial.println();
        oled.println();
        displayTask = 1;
        break;

      case 1:
        // print RSSI (Received Signal Strength Indicator)
//      rssi = radio.getRSSI();
        rssi = 0;
        oled.print("RSSI: ");
        oled.print(rssi);
        oled.println(" dBm");
            
        displayTask = 2;
        break;

      case 2:
        // print SNR (Signal-to-Noise Ratio)
//      snr = radio.getSNR();
        snr = 0;
        oled.print("SNR: ");
        oled.print(snr);
        oled.println(" dB");
            
        displayTask = 3;
        break;

      case 3:
        // print frequency error
//      freq_err = radio.getFrequencyError();
        freq_err = 0;
        oled.print("Freq Err: ");
        oled.print(freq_err);
        oled.println(" Hz");
             
        displayTask = 4;
        break;

      case 4:
        // print battery voltage
        oled.print("VBatt: ");
        oled.print(v_bat);
        oled.println(" volt");

        displayTask = 0;
        break;
            
      default:
        displayTask = 0;
        break;
    }
  }
//  Serial.println(F("[Slave] DISP_READ expired change to DISP_WAIT ... "));
  dispOldTime = micros();                        // start time
  dispTimeOut = DISP_TIMEOUT;                    // delay time = display time out
  dispFsmState = DISP_WAIT;                      // change state

}

//
// display Wait state
//
void disp_Wait() {
	if (checkTimeout(dispOldTime,dispTimeOut)) {
	  // timer expired
//    Serial.println(F("[Slave] DISP_WAIT expired change to DISP_WRITE ... "));
	  dispOldTime = micros();                        // start time
	  dispTimeOut = DISP_TIMEOUT;                    // delay time = display time out
	  dispFsmState = DISP_WRITE;                     // change state
	}
}




//
// check if time out has expired
//
// this only works for the 1st 70 mins () as it doesn't handle when the 32-bit timer rolls over after about 70 mins
// needs some extra code to check this
//
bool checkTimeout(unsigned long oldTime, unsigned long timeOut){
  if (currentTime > oldTime + timeOut) {
    // timer expired
    return true;
  }
  else {
    // not yet expired
	  return false;
  }
}


//
// read Hex switch
//Note that coding is inverted; a switch bit 'On' = '0'
//
uint8_t readHexSwitch() {
  uint8_t hexSwitch;
   hexSwitch = 0;
   if (digitalRead(HSW1) == 0){
    hexSwitch = 1;
   }
  if (digitalRead(HSW2) == 0){
    hexSwitch = hexSwitch +2;
  }
  if (digitalRead(HSW4) == 0){
    hexSwitch = hexSwitch +4;
  }
  if (digitalRead(HSW8) == 0){
    hexSwitch = hexSwitch +8;
  }
  Serial.print(F("Hex switch: "));
  Serial.println(hexSwitch,HEX);

  return hexSwitch;
}

   
//
// this sets up the PCA9685 PWM chip
//
// Initially it sets up a 50% square-wave on test output at the nominal frequency, and then measures the frequncy generated
// this is used to calculate a fiddle factor to correct the PWM frequency to that required.
// 
void cal_pwm() {
  int32_t timeStart, timeStop, timePeriod;                                 //
  float freqScale;

  //
  // now calibrate the PCA9685 PWM frequency.
  // This is done by measuring the time for a number of test PWM pulses, and calculating the 
  // correction required to achieve 50Hz as nearly as possible.
  //
  // This is quite crude code, simply waiting for a series of edges to arrive, but seems to work.
  //
  // If the PCA9685 fails to generate the square-wave then the slave will never progress out of calibration.
  //
  // wait for Hi or rising edge
//  Serial.print(F("wait for rising edge ...."));
  while (digitalRead(PWM_CAL_IN) == 0)
  {
  }
//  Serial.print(F(" wait for falling edge .."));
  // wait for falling edge
  while (digitalRead(PWM_CAL_IN) == 1)
  {
  }
  // falling edge
//  Serial.println(F("now start timing loop"));
  timeStart = micros();
  // now start loop....
  while (totalCounter < NUM_CAL_CYCLES)
  {
    // wait for Hi edge
    while (digitalRead(PWM_CAL_IN) == 0)
    {
    }

    // wait for falling edge
    while (digitalRead(PWM_CAL_IN) == 1)
    {
    }
    // falling edge; have we had enough falling edges?
    totalCounter += 1;
    timeStop = micros();                      // possible stop time
  }
  // sufficient falling edges have been counted, so do the calculations

  timePeriod = timeStop - timeStart;
  totalCounter = 0;
    
  Serial.print(F("Time period = "));
  Serial.print(timePeriod);
  Serial.print(F(" usec for "));
  Serial.print(NUM_CAL_CYCLES);
  Serial.println(F(" pulses"));

  delay(1000);                                // 1 sec delay

  // frequency scaling factor = wanted frequency / measured frequency 
  // this can be used as a multiplier when setting the PWM (frame) frequency in
  // the real servo code
  Serial.print(F("PWM frequency scaling factor = "));
  freqScale = timePeriod * SERVO_FREQ / (NUM_CAL_CYCLES * 1000.0 * 1000.0);
  Serial.println(freqScale);
  //
  // check for sensible range
  if ((freqScale < 0.9) || (freqScale > 1.1))
  {
    Serial.println(F(" *** failed to calibrate PWM ***"));
    // failed, so endless loop
    while (true){
      // failed to calibrate, treat as fatal
      //
      // medium flashing LED
      digitalWrite(LED1,HIGH);
      delay(MEDFLASH);
      digitalWrite(LED1,LOW);
      delay(MEDFLASH);
    }
  }
//  Serial.println(F(" ... success!"));
  
  // now update PCA9685 PWM frequency with corrected value
  pwm.setPWMFreq(SERVO_FREQ * freqScale);  // Analogue servos run at ~50 Hz updates
  
}

//
//
// some byte to (hex) string conversion functions
void byte2str(char* buff, uint8_t val) {  // convert an 8-bit byte to a string of 2 hexadecimal characters
  buff[0] = nibble2hex(val >> 4);
  buff[1] = nibble2hex(val);
}

char nibble2hex(uint8_t nibble) {  // convert a 4-bit nibble to a hexadecimal character
  nibble &= 0xF;
  return nibble > 9 ? nibble - 10 + 'A' : nibble + '0';
}
