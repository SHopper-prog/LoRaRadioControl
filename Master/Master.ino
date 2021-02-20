
//  Name:       master.ino
//  Date:       20 Feb 2021
//  Brief:      Radio-control Master device
//  Author:     Simonn Hopper
//
// ***********************************************************************************************
// Revisions:
//  Date        rev,    who     what
//  20/02/2021  vC      SJH     Add code to read hex DIL switch to define RF channel
//                              Add revision as a string constant
//                              Add I2C address definitions, even though currently default
//  10/02/2021  vB      SJH     Add loops so that one display task is done every NUM_CYCLES so as to not overload the display/printing
//                              Removed delay so Tx/Rx cycles start immediately.
//  30/01/2021          SJH     Added ADS1015 analogue-to-digital module
//  25/01/2021          SJH     initial basic version
//
// ***********************************************************************************************
//
// This generates a series of message packets containing servo postion data plus 8 bits of ON/OFF data.
// In response a 2-byte message is returned, comrising an 'ACK' plus 8-bits of status data
//
// Uses LoRa mode as I cant get FSK to work.
//
// It uses the RadioLib library to provide the control for the SX1278 RF module, with control from a
// 3.3V Arduibo Pro-mini.
//
// It uses adafruit libraries for the ADS1015 4 channel analogue input module
//
// ***********************************************************************************************
//

#include "settings.h"

// include the library for SX1278 RF module
#include <RadioLib.h>

// include the libraries for ADS1015 ADC module
#include <Wire.h>
#include <Adafruit_ADS1015.h>

// SX1278 has the following connections with the Arduino pro-mini:
// NSS pin:   10
// DIO0 pin:  3
// RESET pin: 9
// DIO1 pin:  5
SX1278 radio = new Module(10, 3, 9, 5);

// ADS1015 module
Adafruit_ADS1015 ads1015(I2C_ADC_ADDR);           // define (default) I2C address

uint8_t LED1 = 8;
uint8_t HSW1 = 4;       // hex switch bit 1
uint8_t HSW2 = A3;      // hex switch bit 2
uint8_t HSW4 = A2;      // hex switch bit 4
uint8_t HSW8 = A1;      // hex switch bit 8

uint8_t hexSwitch;         // hex switch value read

// RF channel number, but this should be overwritten later by the switch value
int rfChannel = RF_CH_DEF;

// save transmission state between loops
int16_t state = ERR_NONE;

// start & finish times used to calulate the loop time if required
// note that since 32-bits would equal 4,294,967,296 usec (4,294 secs) there is a chance that these could roll-over
// and cause timeout hang-ups.
//
// this needs sorting.
//
uint32_t txStart,txFinish,rxStart,rxFinish;

// declare 6 channels of servo position data, set to mid positiion as the default
uint16_t servoArr[NUM_ANA_CHAN] = {SERVO_DEF,SERVO_DEF,SERVO_DEF,SERVO_DEF,SERVO_DEF,SERVO_DEF};

// byte used to store 8 x on/off channels, set to OFF as the default
byte swByte =0b00000000;

// byte array used to store messages
// this is formed from:
//  message type; e.g. servo pisition data, failsafe settings, etc.
//  6 x servo position data, channel 0 first, channel 5 last
//  switch channel data, (1=ON, 0=OFF), bit 0 = switch channel 0, bit 7 = switch channel 7
byte msgArr[NUM_ANA_CHAN +2];

bool txTimeout = false;                           // Tx timeout flag
bool rxTimeout = false;                           // Rx timeout flag
bool rxMsgOk = false;                             // received valid message flag

// variables for joystick inputs
int16_t vmeas;
int16_t offset[NUM_ANA_CHAN] = {0,0,0,0,0,0};                  // offset values

int16_t txCount;                                    // counts Tx/Rx cycles
int8_t  displayTask;                                // display task
//
//
//
//
//
//
void setup() {
  float centreFreq;

  // define input pins and their pullups
  pinMode(HSW1,INPUT_PULLUP);                     // hex switch bit 1
  pinMode(HSW2,INPUT_PULLUP);                     // hex switch bit 2
  pinMode(HSW4,INPUT_PULLUP);                     // hex switch bit 4
  pinMode(HSW8,INPUT_PULLUP);                     // hex switch bit 8
  
  Serial.begin(115200);

  // initial banner
  Serial.println();
  Serial.print(F("Master LoRa SX1278 rev: "));
  Serial.print(REV);
  Serial.println();

  txCount = 0;
  displayTask = 0;
  
  //
  // read hex switch inputs to get RF channel number
  // Since the switch values are 0..15 the RF channel are 3..33.
  // This allows a reasonable spread, but keeps within the valie 1..40 range but keeps away from the RF band edges
  // to allow for the RF bandwidth actually used; approx 3 channels.
  hexSwitch = readHexSwitch();
  rfChannel = 2 * int(hexSwitch) + 3;


  // calculate centre frequency from channel number
  centreFreq =CENTRE_FREQ_CH0 + RF_CH_SPACE * rfChannel; 
  Serial.print(F("RF centre frequency: "));
  Serial.print(centreFreq);
  Serial.print(F(" (MHz), RF channel: "));
  Serial.println(rfChannel);

  // enble ADC
  Serial.println(F("Initialising A-to-D converters..."));
  ads1015.begin();

  
  //
  // initialize SX1278 with required settings
  Serial.print(F("[Master] Initializing LoRa... "));

//
//  int state = radio.begin();

  // initialise as LoRa
  state = radio.begin(centreFreq,BANDWIDTH,SPREADINGFACTOR,CODERATIO,SYNC_WORD,TXPMAX,NUM_PREAMBLE,RXGAIN);

  // check for configuration
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true) {
      // failed to configure, treat as fatal
      //
      // fast flash LED
      digitalWrite(LED1,HIGH);
      delay(FASTFLASH);
      digitalWrite(LED1,LOW);
      delay(FASTFLASH);
    }
  }

  // set the function that will be called when packet transmission is finished
  radio.setDio0Action(setFlag);

  // start transmitting the first packet
//  Serial.print(F("[Master] Sending first packet ... "));

  // set up message buffer and the default servo position array
  msgArr[0] = MSG_CMND;
  for (int i =0; i< NUM_ANA_CHAN; i++){
    servoArr[i] = SERVO_DEF;
    msgArr[i+1] = servoArr[i];
  }
  msgArr[NUM_ANA_CHAN+1] = swByte;

  txStart = micros();                                     // capture start time
  txTimeout = false;                                      // clear Tx timeout flag

  state = radio.startTransmit(msgArr, NUM_ANA_CHAN+2);    // start transmit mode
}

//
// flag to indicate that a packet was sent
// This flag is also used to indicate the a packet has been received no we have a ping-pong system, but
// retain the name
//
volatile bool transmittedFlag = false;

//
// disable interrupt when it's not needed
//
volatile bool enableInterrupt = true;

// this function is called when a complete packet is transmitted by the module
// IMPORTANT: this function MUST be 'void' type and MUST NOT have any arguments!
//
// it now also called when a complete packet has been received
//
void setFlag(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }

  // we sent a packet, set the flag
  transmittedFlag = true;
}

//
//
//
void loop() {
  int32_t adcStart,adcStop;                       // used to time the execution time of ADC
  int16_t adc0;                                   // raw ADC value
  float adc1;                                     // scaled value
  
  // check if the previous transmission finished
  if(transmittedFlag == false) {
    //
    // Tx data not yet sent, so check for Tx timeout
    //
    if (micros() > txStart + TX_TIMEOUT){
      Serial.print(F("[Master] ******* Tx Timeout ********"));
      txTimeout = true;

      while (false){
        // failed to send, treat as fatal at the moment.
        // could simply try and re-send
        //
        // slow flashing LED
        digitalWrite(LED1,HIGH);
        delay(SLOWFLASH);
        digitalWrite(LED1,LOW);
        delay(SLOWFLASH);
      }
    }
  } else {
    // Tx message sent
    //
    // disable the interrupt service routine while processing the data
    enableInterrupt = false;                                // disable interrupts
    transmittedFlag = false;                                // clear interrupt flag
    txTimeout = false;                                      // clear Tx timeout flag
    // check response to previous start transmit
    if (state == ERR_NONE) {
      //
      // packet was successfully sent
      //
//      Serial.println(F("transmission finished!"));

    } else {
      //
      // not quite sure what to do if it fails to send
      //
      Serial.print(F("failed, code "));
      Serial.println(state);
    }
    txFinish = micros();                                 // capture end of Tx time
    digitalWrite(LED1, LOW);                             // turn LED off
    //
    // finished processing transmit data,
    // set up to wait for ACK returned
    //
    radio.sleep();                                       // set to 'sleep' inbetween Tx & Rx
    
    // start listening for LoRa packets
//    Serial.print(F("[Master] Starting to listen ... "));
    state = radio.startReceive();
    if (state == ERR_NONE) {
//      Serial.println(F("success!"));
    } else {
      // failed to set to receive mode, treat as fatal at the moment
      //
      // flash LED
      Serial.print(F("failed, code "));
      Serial.println(state);
      while (true){
        // medium flashing LED
        digitalWrite(LED1,HIGH);
        delay(MEDFLASH);
        digitalWrite(LED1,LOW);
        delay(MEDFLASH);
      }
    }

    rxStart = micros();                               // capture start of rx time
    rxTimeout = false;                                // clear rx timeout flag
    enableInterrupt = true;                           // enable interrupts
    
    // wait until the flag is set.
    // as mentioned above the 'transmittedFlag' is also used to indicate received data
    while (transmittedFlag == false) {
      if (micros() > rxStart + RX_TIMEOUT) {
        // time out
        Serial.println(F("[Master] ******** Rx Timeout *******"));
        rxTimeout = true;
        break;
      }
      //
      // not yet timed out so go round again
      //
    }

    // block any further interrupts while we process the received data
    enableInterrupt = false;                          // disable interrupt
    transmittedFlag = false;                          // clear interrupt flag
    rxMsgOk = false;                                  // set true when a valid message is received
    rxFinish = micros();                              // capture end of Rx time
  
    if (rxTimeout == false){
      // receive didn't timeout, so read & check
      state = radio.readData(msgArr, LEN_ACKMSG);
      if (state == ERR_NONE) {
        // packet was successfully received
        rxMsgOk = true;                               // set Rx messag OK flag
//        Serial.println(F("[Master] Received packet!"));
        //
        // only display data every so many Tx/Rx cycles
        if (txCount > NUM_CYCLES) {
          txCount = 0;                                  // reset
          // get various parameters of the received packet
          // 
          switch(displayTask) {
            case 0:
              // print data of the packet
              Serial.print(F("[Master] Data:\t\t"));
              for (uint8_t i=0; i< LEN_ACKMSG;i++){
                Serial.print(msgArr[i],HEX);
                Serial.print(F(" "));
              }
              Serial.println();
              displayTask = 1;                        // next display task
              break;

            case 1:        
              // print RSSI (Received Signal Strength Indicator)
              Serial.print(F("[Master] RSSI:\t\t"));
              Serial.print(radio.getRSSI());
              Serial.println(F(" dBm"));
              displayTask = 2;                        // next display task
              break;

            case 2:
              // print SNR (Signal-to-Noise Ratio)
              Serial.print(F("[Master] SNR:\t\t"));
              Serial.print(radio.getSNR());
              Serial.println(F(" dB"));
              displayTask = 3;                        // next display task
              break;

            case 3:
              // print frequency error
              Serial.print(F("[Master] Frequency error:\t"));
              Serial.print(radio.getFrequencyError());
              Serial.println(F(" Hz"));
              displayTask = 4;                        // next display task
              break;

            case 4:
              // print the time it has taken to transmit packet and recive the 'ACK' response
              // note that this is only valid if the actual message is invalid
      
              Serial.print(F("[Master] Loop time: "));
              Serial.print(rxFinish - txStart);
              Serial.println(F(" us"));
              displayTask = 0;                        // next display task
              break;

            default:
              displayTask = 0;
              break;
              
          }
        }
        else {
          txCount = txCount +1;
        }
        //
        //
        // anything else when the message is OK, e.g. could display on a display screen
        //

       
  
      } else if (state == ERR_CRC_MISMATCH) {
        // packet was received, but is malformed
        Serial.println(F("[Master] CRC error!"));
  
      } else {
        // some other error occurred
        Serial.print(F("[Master] Failed, code "));
        Serial.println(state);
      }
    }

    // 
    // finished processing ACK message or Rx message timed out
    // either way set back to Tx and sed the next message
    //
    radio.sleep();                                        // put module to sleep mode between Tx & Rx

    //
    //
    // perform other stuff like reading servo analogue channels etc. here
    //
    //
    //

    //
    // read digital switch inputs here
    //

    //
    // read joystick analogue inputs
    //
    adcStart = micros();
    // execute for each servo channel
    for (int8_t n=0; n<4; n++)
    {
      //
      // initiate a conversion, not the quickest, simplest.
      // to shave a bit of time of set the ADC to continuous conversion then simply
      // read the last value.
      //
      adc0 = ads1015.readADC_SingleEnded(n);              // read ADC value
      //
      // re-scale to be 0 to RANGE, and add the offset
      // this is simply:
      //
      //  scaled = RANGE * (Vmeas - Vmin) / (Vmax - Vmin) + offset
      //
      // where the Vmax & Vmin have been pre-measured using this sketch but with default values.
      // Vmin & Vmax can be noted down from the reported adc0 values as the joystick is physically
      // moved from one extreme to the other.
      // Ensure that Vmax != Vmin to prevent divide-by-zero errors
      //
      // the offset is for future possible use with some kind of trim function, possibly using
      // another set of ADC & potentiometers.
      // 
      adc1 = RANGE * (float(adc0) - float(vmin[n])) / (float(vmax[n]) - float(vmin[n])) + float(offset[n]);
      // clip the value to be between 0 and RANGE
      if (adc1 > RANGE)
      {
        adc1 = RANGE;
      }
      if (adc1 < 0)
      {
        adc1 = 0;
      }
      // store in the servo data array as 16-bit (unsigned) data
      servoArr[n] = adc1;
  
      // now display
      /*
      Serial.print(F(" channel "));
      Serial.print(n);
      Serial.print(F(" = "));
      Serial.print(adc0);
      Serial.print(F(" : "));
      Serial.println(servoArr[n]);
      */
    }
    /*
    adcStop = micros() - adcStart;
    Serial.print(F("ADC Loop time: "));
    Serial.println(adcStop);
    Serial.println();
    */


//    // wait a second before transmitting again, just for now to slow things down a bit
//    delay(1000);

    // send next packet
//    Serial.print(F("[Master] Sending another packet ... "));

    // set up message array
    msgArr[0] = MSG_CMND;
    for (int i =0; i< NUM_ANA_CHAN; i++){
      msgArr[i+1] = (servoArr[i] & 0x00FF);               // extract LSB only
    }
    msgArr[NUM_ANA_CHAN+1] = swByte;

    digitalWrite(LED1, HIGH);                             // set LED ON

    txStart = micros();                                   // capture start time (usec)
    state = radio.startTransmit(msgArr, NUM_ANA_CHAN+2);

    transmittedFlag = false;                              // clear interrupt flag, if not already
    txTimeout = false;                                    // clear Tx timeout flag
    enableInterrupt = true;                               // enable interrupt service routine
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
