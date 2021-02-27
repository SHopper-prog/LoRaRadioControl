
//  Name:       master.ino
//  Date:       26 Feb 2021
//  Brief:      Radio-control Master device
//  Author:     Simonn Hopper
//
// ***********************************************************************************************
// Revisions:
//  Date        rev,    who     what
//  27/02/2021          SJH     Add display of RF channel number
//  26/02/2021  vE      SJH     Add code to drive OLED display & display RF parameters etc. on a scrolling display
//                              Reduce data being sent to console serial port
//                              Reduce number of analogue channels to 4
//                              Replace the '#define' with 'const ....'
//  24/02/2021          SJH     Add trim facility to joystick channels 0 & 1
//                              Change LED1 to flash when a trim increment/decrement is seen.
//  23/02/2021  vD      SJH     Add code to read MCP23017 IO expander to read 16-bits switch data, bank A is used
//                              as the data sent to the slave unit, bank B not currently used.
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
// As the MCP23017 is simple no libraries are used to control, simply in-line type code only
//
// It uses the SSD1306Ascii library by Bill Greiman, as the adafruit library is too large
//
// ***********************************************************************************************
//

#include "settings.h"

// include the library for SX1278 RF module
#include <RadioLib.h>

// include the libraries for ADS1015 ADC module
#include <Wire.h>
#include <Adafruit_ADS1015.h>
Adafruit_ADS1015 ads1015(I2C_ADC_ADDR);           // define (default) I2C address

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
uint16_t servoArr[MAX_ANA_CHAN] = {SERVO_DEF,SERVO_DEF,SERVO_DEF,SERVO_DEF,SERVO_DEF,SERVO_DEF};

// byte used to store 8 x on/off channels, set to OFF as the default
byte swByteA =0b00000000;                         // bank A inputs
byte swByteB =0b00000000;                         // bank B inputs

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
int16_t offset[MAX_ANA_CHAN] = {0,0,0,0,0,0};       // offset values

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

  // set up OLED display
  oled.begin(&Adafruit128x32, I2C_OLED_ADDR);
  oled.setFont(Callibri14);
  oled.setScrollMode(SCROLL_MODE_AUTO);
  oled.clear();
  oled.print("Master s/ware, rev. ");
  oled.println(REV);

  delay(2000);                                            // 2 sec delay
  
  txCount = 0;
  displayTask = 0;
  
  //
  // read hex switch inputs to get RF channel number
  // Since the switch values are 0..15 the RF channel are 3..33.
  // This allows a reasonable spread, but keeps within the valie 1..40 range but keeps away from the RF band edges
  // to allow for the RF bandwidth actually used; approx 3 channels.
  hexSwitch = readHexSwitch();
  rfChannel = 2 * int(hexSwitch) + 3;

  oled.print("RF channel: ");
  oled.println(rfChannel);
  delay(2000);                                            // 2 sec delay

  // calculate centre frequency from channel number
  centreFreq =CENTRE_FREQ_CH0 + RF_CH_SPACE * rfChannel; 
  Serial.print(F("RF centre frequency: "));
  Serial.print(centreFreq);
  Serial.print(F(" (MHz), RF channel: "));
  Serial.println(rfChannel);

  // enble ADC
  Serial.println(F("Initialising A-to-D converters..."));
  ads1015.begin();

  // setup MCP23017
  Serial.println(F("Initialising IO expander..."));
   Wire.begin(); // wake up I2C bus
  // set I/O pins to inputs
  Wire.beginTransmission(I2C_DIO_ADDR);                     // I2C address
  Wire.write(0x00);                                         // IODIRA register
  Wire.write(0xFF);                                         // set all of port A to inputs (default)
  Wire.endTransmission();
  Wire.beginTransmission(I2C_DIO_ADDR);                     // I2C address
  Wire.write(0x01);                                         // IODIRB register
  Wire.write(0xFF);                                         // set all of port A to inputs (default)
  Wire.endTransmission();
  // invert polarity of all inputs so that a switch 'closed' = 1
  Wire.beginTransmission(I2C_DIO_ADDR);                     // I2C address
  Wire.write(0x02);                                         // IOPOLA register
  Wire.write(0xFF);                                         // set all of port A to inverted
  Wire.endTransmission();
  Wire.beginTransmission(I2C_DIO_ADDR);                     // I2C address
  Wire.write(0x03);                                         // IOPOLBB register
  Wire.write(0xFF);                                         // set all of port A to inverted
  Wire.endTransmission();
  // enable all internal pull-ups
  Wire.beginTransmission(I2C_DIO_ADDR);                     // I2C address
  Wire.write(0x0C);                                         // GPPUA register
  Wire.write(0xFF);                                         // set all of port A to pull-ups enabled
  Wire.endTransmission();
  Wire.beginTransmission(I2C_DIO_ADDR);                     // I2C address
  Wire.write(0x0D);                                         // GPPUB register
  Wire.write(0xFF);                                         // set all of port A to pull-ups enabled
  Wire.endTransmission();
  
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
  msgArr[NUM_ANA_CHAN+1] = swByteA;

  txStart = micros();                                     // capture start time
  txTimeout = false;                                      // clear Tx timeout flag

  oled.println("configured");
  
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

  int16_t snr;
  int16_t rssi;
  int16_t freq_err;

  char bytestr[1];                                // used when converting a byte to a hex string
  
  // check if the previous transmission finished
  if(transmittedFlag == false) {
    //
    // Tx data not yet sent, so check for Tx timeout
    //
    if (micros() > txStart + TX_TIMEOUT){
      Serial.print(F("[Master] ******* Tx Timeout ********"));
      txTimeout = true;
      oled.println("*** Tx timeout ***");
      
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
      oled.println("*** failed to send packet ***");
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
      oled.println("*** Failed to set Rx mode ***");
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
        oled.println("*** Rx timeout ***");
        
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
        rxMsgOk = true;                               // set Rx message OK flag
//        Serial.println(F("[Master] Received packet!"));
        //
        // only display data every so many Tx/Rx cycles
        if (txCount >= MSG_COUNT) {
          txCount = 0;                                  // reset

          // this is the place to check swByteB inputs to act on them, as it only happens every so many cycles
          // e.g. check for a trim inputs being '1' and increment/decrement the servo offset data.

          digitalWrite(LED1,LOW);                       // set LED OFF
        
          // joystick channel 0 trim
          // bit 0 increments, bit 1 decrements, if both set then no change
          if (swByteB & 0x01){
            // bit set
            offset[0] = offset[0] +1;
            digitalWrite(LED1,HIGH);                   // set LED ON
//            Serial.println(F("Joystick 0 offset incremented"));
            oled.println("joystick 0 incremented");
          }
          if (swByteB & 0x02){
            // bit set
            offset[0] = offset[0] -1;
            digitalWrite(LED1,HIGH);                   // set LED ON
//            Serial.println(F("Joystick 0 offset decremented"));
            oled.println("joystick 0 decremented");
          }

          // joystick channel 1 trim
          // bit 2 increments, bit 3 decrements, if both set then no change
          if (swByteB & 0x04){
            // bit set
            offset[1] = offset[1] +1;
            digitalWrite(LED1,HIGH);                   // set LED ON
//            Serial.println(F("Joystick 1 offset incremented"));
            oled.println("joystick 1 incremented");
          }
          if (swByteB & 0x08){
            // bit set
            offset[1] = offset[1] -1;
            digitalWrite(LED1,HIGH);                   // set LED ON
//            Serial.println(F("Joystick 1 offset decremented"));
            oled.println("joystick 1 decremented");
          }


          // display digital inputs
//          Serial.print(F("Digital inputs: "));
//          Serial.print(swByteA,HEX);
//          Serial.print(F(" "));
//          Serial.println(swByteB,HEX);
          
          // get various parameters of the received packet
          // 
          switch(displayTask) {
            case 0:
              // print data of the packet
              oled.print("Data: ");
//              Serial.print(F("[Master] Data:\t\t"));
              for (uint8_t i=0; i< LEN_ACKMSG;i++){
//                Serial.print(msgArr[i],HEX);
//                Serial.print(F(" "));

                byte2str(bytestr,msgArr[i]);
                oled.print(bytestr);
                oled.print(" ");
              }
//              Serial.println();
              oled.println();

              displayTask = 1;                        // next display task
              break;

            case 1:        
              // print RSSI (Received Signal Strength Indicator)
//              Serial.print(F("[Master] RSSI:\t\t"));
              rssi = radio.getRSSI();
//              Serial.print(rssi);
//              Serial.println(F(" dBm"));

              oled.print("RSSI: ");
              oled.print(rssi);
              oled.println(" dBm                  ");
              
              displayTask = 2;                        // next display task
              break;

            case 2:
              // print SNR (Signal-to-Noise Ratio)
//              Serial.print(F("[Master] SNR:\t\t"));
              snr = radio.getSNR();
//              Serial.print(snr);
//              Serial.println(F(" dB"));

              oled.print("SNR: ");
              oled.print(snr);
              oled.println(" dB                        ");
              displayTask = 3;                        // next display task
              break;

            case 3:
              // print frequency error
//              Serial.print(F("[Master] Frequency error:\t"));
              freq_err = radio.getFrequencyError();
//              Serial.print(freq_err);
//              Serial.println(F(" Hz"));

              oled.print("Freq err: ");
              oled.print(freq_err);
              oled.println(" Hz                         ");
              displayTask = 4;                        // next display task
              break;

            case 4:
              // print the time it has taken to transmit packet and recive the 'ACK' response
              // note that this is only valid if the actual message is valid
      
              Serial.print(F("[Master] Loop time: "));
              Serial.print(rxFinish - txStart);
              Serial.println(F(" us"));

              oled.print("Loop time: ");
              oled.print(rxFinish - txStart);
              oled.println(" usec");
              
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
        oled.println("CRC error");
  
      } else {
        // some other error occurred
        Serial.print(F("[Master] Failed, code "));
        Serial.println(state);
        oled.println("Other error");
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

    // read the inputs of bank A; these are the digial channels controls
    Wire.beginTransmission(I2C_DIO_ADDR);
    Wire.write(0x12);                                 // bank A 
    Wire.endTransmission();
    Wire.requestFrom(int(I2C_DIO_ADDR), 1);
    swByteA = Wire.read();

    // read the inputs of bank B; these could be used for other tasks, e.g. as a trim input for some of the joystick channels
    Wire.beginTransmission(I2C_DIO_ADDR);
    Wire.write(0x13);                                 // bank B
    Wire.endTransmission();
    Wire.requestFrom(int(I2C_DIO_ADDR), 1);
    swByteB = Wire.read();



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
    msgArr[NUM_ANA_CHAN+1] = swByteA;

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

