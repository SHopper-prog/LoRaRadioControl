//
//  Name:       slave.ino
//  Date:       26 Feb 2021
//  Brief:      Radio-control Slave device
//  Author:     Simon Hopper
//
// ***********************************************************************************************
// Revisions:
//  Date        rev.    who     what
//  27/02/2021          SJH     Add delay before sending ACK
//                              Add Rx Fail count before declaring FAILSAFE mode
//                              Add display of RF channel number
//                              Set bits in status byte returned to Master for Rx timeout, Failsafe & message error, reset
//                              using digital channel 7
//  26/02/2021          SJH     Finish of using OLED display & displaying RF parameters on a scrolling display
//                              Reduce the amount of data being sent to colsole serial port
//                              Reduce the number of analogue channels to 4
//                              Replace the '#define' with 'const ...'
//  25/02/2021  vE      SJH     Add a different OLED library that fits in the FLASH
//  24/02/2021          SJH     Add the rest of the digital channel controls
//  23/02/2021  vD      SJH     Add code to output some of the digital channels
//                              Remove OLED display code as it takes up too much memory
//  20/02/2021          SJH     Add code to read hex DIL switch to define RF channel
//                              Add I2C address definitions, even though currently default
//  13/02/2021  vC      SJH     Move PWM calibratiion to seperate function
//                              Add ability to invert servo direction
//                              Add revision as string constant
//  07/02/2021          SJH     Add PCA9685 oscillator calibration code
//                              Comment out most print statements as they take too much time to process
//  06/02/2021  vB      SJH     Add servo control for analogue channels
//  25/01/2021          SJH     initial basic version
//
// ***********************************************************************************************
//
// This expects a series of message packets containing message header, servo postion data plus 8 bits of ON/OFF data.
// Currently the message header is always 0x5A (MSG_CMND), but could be expanded e.g. to send fail-safe settings.
// In response a 2-byte message is returned, comrising an 'ACK' plus 8-bits of status data
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
// ***********************************************************************************************
//

#include "settings.h"

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

uint8_t hexSwitch;         // hex switch value read

// this is the RF channel number
int rfChannel = RF_CH_DEF;

// start & finish times used to calculate the loop time if required
// note that since 32-bits would equal 4,294,967,296 usec (4,294 secs) there is a chance that these could roll-over
// and cause timeout hang-ups.
//
// this needs sorting.
//
uint32_t rxStart,rxFinish,txStart,txFinish;

// declare 6 channels of servo position data, set to mid positiion as the default
byte servoArr[MAX_ANA_CHAN] = {SERVO_MID,SERVO_MID,SERVO_MID,SERVO_MID,SERVO_MID,SERVO_MID};

// byte used to store 8 x on/off channels, set to OFF as the default
byte swByte =0b00000000;

// byte use to store slave status
byte statusByte = 0x00;

//
// return value from library function calls.
// should be ERR_NONE, but if not it contains the error code
int16_t state;

// byte array used to store messages.
// For received this is formed from:
//  message type; e.g. servo position data, failsafe settings, etc.
//  6 x servo position data, channel 0 first, channel 5 last
//  switch channel data, (1=ON, 0=OFF), bit 0 = switch channel 0, bit 7 = switch channel 7
//
byte msgArr[NUM_ANA_CHAN +2];

bool rxTimeout = false;                             // Rx timeout flag, 1 = timed out
bool txTimeout = false;                             // Tx timeout flag, 1 = timed out
bool rxMsgOk = false;                               // received valid message flag, 1 = OK
bool txMsgOk = false;                               // transmit message sent, 1 = OK

uint8_t servoNum = 0;                               // current servo number

uint8_t PWM_CAL_IN = 2;                             // input pin for calibration PWM, via 22k series resistor
uint16_t pulseStart,pulseStop;                      // start & stop times for servo PWM
float servoConst[MAX_ANA_CHAN] = {0,0,0,0,0,0};     //
int16_t failSafe[MAX_ANA_CHAN] = {0,0,0,0,0,0};     //
uint16_t totalCounter;                              // PWM calibration cycle counter

int8_t rxFailCount;                                   // counts consecutive Rx Fail timeoute
int16_t txCount;                                    //
int8_t displayTask;                                   //


//
//
//
//
//
void setup() {
  float centreFreq;
  uint8_t prescale;
  int8_t lastSample;

  // define input pins and their pullups
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
  oled.print("Slave software, rev. ");
  oled.println(REV);
  delay(2000);                                // 2 sec delay

  //
  // read hex switch inputs to get RF channel number
  // Since the switch values are 0..15 the RF channel are 3..33.
  // This allows a reasonable spread, but keeps within the valie 1..40 range but keeps away from the RF band edges
  // to allow for the RF bandwidth actually used; approx 3 channels.
  hexSwitch = readHexSwitch();
  rfChannel = 2 * int(hexSwitch) + 3;
  oled.print("RF channel: ");
  oled.println(rfChannel);

  // calulate centre frequency from channel number
  centreFreq =CENTRE_FREQ_CH0 + RF_CH_SPACE * rfChannel; 
  Serial.print(F("RF centre frequency: "));
  Serial.print(centreFreq);
  Serial.print(F(" (MHz), RF channel: "));
  Serial.println(rfChannel);
  //
  txCount = 0;
  displayTask = 0;
  rxFailCount = 0;
  //
  // initialise the PCA0685 PWM
  Serial.print(F("[Slave] Setting up PCA0685 PWM module ..."));
  pwm.begin();                              // use default prescale
  pwm.setPWMFreq(SERVO_FREQ);               // nominal 50Hz
  pwm.setPWM(SERVO_CAL,0,2048);             // set to 50% squarewave
  prescale= pwm.readPrescale();

  Serial.print(F("Target frequency: "));
  Serial.println(SERVO_FREQ);  
  Serial.print(F("Applied prescale: "));
  Serial.println(prescale);

  totalCounter = 0;
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
    failSafe[servoNum] = 128;
  }
  Serial.println(F(" success!"));
  
  //
  // initialize SX1278 with default settings
  //
  
//
//  int state = radio.begin();

  // set up LoRa mode
  Serial.print(F("[Slave] Initializing LoRa ... "));
  state = radio.begin(centreFreq,BANDWIDTH,SPREADINGFACTOR,CODERATIO,SYNC_WORD,TXPMAX,NUM_PREAMBLE,RXGAIN);
  //
  // is module configured?
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state,HEX);
    oled.print("*** Failed to config SX1278 ***\n");
    while (true){
      //
      // failed to setup SX1278 module, treat as fatal
      //
      // fast flash LED
      digitalWrite(LED1,HIGH);
      delay(FASTFLASH);
      digitalWrite(LED1,LOW);
      delay(FASTFLASH);
    }
  }
  
  // set the function that will be called when new packet is received
  radio.setDio0Action(setFlag);

  // start listening for LoRa packets
  Serial.print(F("[Slave] Starting to listen ... "));
  
  state = radio.startReceive();
  if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state,HEX);
    oled.print("*** Failed to start Rx ***\n");
    while (true){
      // failed to start receiver, treat as fatal
      //
      // could set 'fail safe' settings?
      //
      // medium flashing LED
      digitalWrite(LED1,HIGH);
      delay(MEDFLASH);
      digitalWrite(LED1,LOW);
      delay(MEDFLASH);
    }
  }
  
  rxStart = micros();                               // set start time
  rxTimeout = false;                                // clear Rx timeout flag

  // if needed, 'listen' mode can be disabled by calling
  // any of the following methods:
  //
  // radio.standby()
  // radio.sleep()
  // radio.transmit();
  // radio.receive();
  // radio.readData();
  // radio.scanChannel();

  oled.println("Configured");
}

//
// flag to indicate that a packet was received.
// it is aslos used to indicate a packet was transmitted in this ping-pong mode
//
volatile bool receivedFlag = false;

//
// disable interrupt when it's not needed
//
volatile bool enableInterrupt = true;

//
// this function is called when a complete packet is received by the module
// IMPORTANT: this function MUST be 'void' type and MUST NOT have any arguments!
//
// It is also called when a complete packet has been sent as the same interrupt pin is used
//
void setFlag(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }
  // we got a packet, set the flag
  receivedFlag = true;
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
  
  // check if the interrupt flag is set
  if(receivedFlag == false) {
    // not yet received message

    if (micros() > rxStart + RX_TIMEOUT){
      // Rx timeout
      Serial.println(F("[Slave] ******** Rx timeout **********"));
      enableInterrupt = false;                            // disable interrupts
      receivedFlag = false;                               // clear interrupt flag
      rxTimeout = true;                                   // set Rx timeout flag

      oled.println("** Rx Timeout ***");
      
      statusByte = statusByte | FLG_RXTIMEOUT;

      if (rxFailCount >= FSAFE_COUNT)
      {
        //
        // do any timeout 'failsafe' actions here
        //
        Serial.println(F("[Slave] ******** FAILSAFE **********"));
        oled.println("**** FAILSAFE ****");

        statusByte = statusByte | FLG_FAILSAFE;
        
        for (servoNum = 0; servoNum < NUM_ANA_CHAN; servoNum++)
        {
          pulseStart = PWMOFF[servoNum];
          pulseStop = pulseStart + SERVOMIN[servoNum] + servoConst[servoNum]* failSafe[servoNum];
          pwm.setPWM(servoNum,pulseStart,pulseStop);
        }

        //
        // set status error bit?
        //
      }
      else
      {
        rxFailCount = rxFailCount +1;
        // do we set error status bit?
      }

      // 
      // now restart Receiver
      //
      state = radio.sleep();                                // set to 'sleep' inbetween Tx & Rx modes
    
      // start listening for LoRa packets
//      Serial.print(F("[Slave] Starting to listen after Rx timeout ... "));
    
      state = radio.startReceive();
      if (state == ERR_NONE) {
        Serial.println(F("success!"));
      } else {
        Serial.print(F("failed restart, code "));
        Serial.println(state,HEX);
        while (true){
          // failed to start receive mode, treat as fatal
          //
          // possibly set 'fail safe' conditions
          //
          // medium flashing LED
          digitalWrite(LED1,HIGH);
          delay(MEDFLASH);
          digitalWrite(LED1,LOW);
          delay(MEDFLASH);
        }
      }
    
      rxStart = micros();                                   // capture start of receive
      rxTimeout = false;                                    // clear timeout flag
      rxMsgOk = false;                                      // clear message OK flag
      receivedFlag = false;                                 // clear interrupt flag
      // we're ready to receive more packets,
      
      enableInterrupt = true;                               // enable interrupt service routine
    }
    //
    // just go round loop again
    //
  } else {
    //
    // Rx message received
    // disable the interrupt service routine while processing the data
    enableInterrupt = false;

    // reset flag
    receivedFlag = false;
    
    rxFinish = micros();                                    // rx finish time
    rxMsgOk = false;
    rxFailCount = 0;                                        // reset Rx fail count
    //
    // read received data as byte array
    state = radio.readData(msgArr, NUM_ANA_CHAN+2);
    if (state == ERR_NONE){
      // packet was successfully received
      rxMsgOk = true;                                       // set flag to indicate valid received packet
//      Serial.println(F("[Slave] Received packet!"));
      if (txCount >= MSG_COUNT)
      {
        //
        // Every so many Rx messages extract various nuggets of information about received packet
        // and display one at a time
        //
        txCount = 0;
        switch(displayTask){
          case 0:
            // print data of the packet
            Serial.print(F("[Slave] Data:\t\t"));
            for (uint8_t i=0; i< NUM_ANA_CHAN+2;i++){
              Serial.print(msgArr[i],HEX);
              Serial.print(F(" "));
              if (i>=1){
                byte2str(bytestr,msgArr[i]);                // don't bother printing byte[0]
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
            rssi = radio.getRSSI();
//            Serial.print(F("[Slave] RSSI:\t\t"));
//            Serial.print(rssi);
//            Serial.println(F(" dBm"));
            
            oled.print("RSSI: ");
            oled.print(rssi);
            oled.println(" dBm");
            
            displayTask = 2;
            break;

          case 2:
            // print SNR (Signal-to-Noise Ratio)
            snr = radio.getSNR();
//            Serial.print(F("[Slave] SNR:\t\t"));
//            Serial.print(snr);
//            Serial.println(F(" dB"));

            oled.print("SNR: ");
            oled.print(snr);
            oled.println(" dB");
            
            displayTask = 3;
            break;

          case 3:
            // print frequency error
            freq_err = radio.getFrequencyError();
//            Serial.print(F("[Slave] Frequency error:\t"));
//            Serial.print(freq_err);
//            Serial.println(F(" Hz"));

            oled.print("Freq Err: ");
            oled.print(freq_err);
            oled.println(" Hz");
            
            displayTask = 0;
            break;

          default:
            displayTask = 0;
            break;
        }
      }
      else
      {
        txCount = txCount +1;
      }
      //
      //
      // process valid message data here
      //
      // check message command
      if (msgArr[0] == MSG_CMND) {
        // valid servo message command so copy the servo message bytes from message to servo data array
        for (servoNum = 0; servoNum < NUM_ANA_CHAN; servoNum++)
        {
          servoArr[servoNum] = msgArr[servoNum +1];
        }
        //
        // copy on/off channel message here
        swByte = msgArr[NUM_ANA_CHAN +1];                        // digital switch data
        
      }

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

      //
      // do on/off channels here
      // Not very sophisticated but efficient code
      // bit 0
      if (swByte & 0x01){
        // turn ON
        pwm.setPWM(8,4096,0);                 // fully on
      }
      else {
        // turn OFF
        pwm.setPWM(8,0,4096);                 // fully off
      }
      // bit 1
      if (swByte & 0x02){
        pwm.setPWM(9,4096,0);                 // fully on
      }
      else {
        pwm.setPWM(9,0,4096);                 // fully off
      }
      // bit 2
      if (swByte & 0x04){
        pwm.setPWM(10,4096,0);                 // fully on
      }
      else {
        pwm.setPWM(10,0,4096);                 // fully off
      }
      // bit 3
      if (swByte & 0x08){
        pwm.setPWM(11,4096,0);                 // fully on
      }
      else {
        pwm.setPWM(11,0,4096);                 // fully off
      }
      // bit 4
      if (swByte & 0x10){
        pwm.setPWM(12,4096,0);                 // fully on
      }
      else {
        pwm.setPWM(12,0,4096);                 // fully off
      }
      // bit 5
      if (swByte & 0x20){
        pwm.setPWM(13,4096,0);                 // fully on
      }
      else {
        pwm.setPWM(13,0,4096);                 // fully off
      }
      // bit 6
      if (swByte & 0x40){
        pwm.setPWM(14,4096,0);                 // fully on
      }
      else {
        pwm.setPWM(14,0,4096);                 // fully off
      }
      // bit 7
      if (swByte & 0x80){
        pwm.setPWM(15,4096,0);                 // fully on
        statusByte = 0x00;                    // clear status byte
      }
      else {
        pwm.setPWM(15,0,4096);                 // fully off
      }


      //
      // clear status error bits?
      //


    } else if (state == ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("[Slave] CRC error!"));
      //
      // set error bit in status byte here
      statusByte = statusByte | FLG_MSGERR;

    } else {
      // some other error occurred whilst reading data
      Serial.print(F("[Slave] Failed data read, code "));
      Serial.println(state,HEX);
      //
      // set error bit in status byte here
      statusByte = statusByte | FLG_MSGERR;
      
    }

    // *********************************************************************************
    //
    // The master takes a finite time to switch from Tx to Rx and even with the Rx message processing
    // some extra delay may be required if the Master signals the occasional 'Rx Failure'.
    // This extra padding is added here to prevent the Slave from sending the ACK message before 
    // the Master is ready to receive.
    delay(20);                                           // msec delay



    //
    // send ACK message back
    //
    digitalWrite(LED1, HIGH);                           // set LED ON whilst sending ACK message
    
    state = radio.sleep();                              // set to 'sleep' inbetween Rx & Tx modes
    // set up ACK message
//    Serial.print(F("[Slave] sending ACK ..."));
    msgArr[0] = MSG_ACK;
    msgArr[1] = statusByte;
    txStart = micros();                                 // capture start time

    state = radio.startTransmit(msgArr, LEN_ACKMSG);
    if (state == ERR_NONE) {
//      Serial.println(F("success!"));

      txTimeout = false;
      receivedFlag = false;
      enableInterrupt = true;                           // enable interrupts

      while (receivedFlag == false){
        // wait for send to complete or Tx timeout
        if (micros() > txStart + TX_TIMEOUT){
          //
          // if Tx timeout occurs, simply move on to set up the next receive phase
          txTimeout = true;
          
          //
          // set status error bit?
          //
          
          Serial.println(F("[Slave] ****** Tx timeout ******"));

          oled.println("*** Tx timeout ***");
          break;  
        }
      }
      //
      // block any further interrupts while we process the tx data
      receivedFlag = false;
      enableInterrupt = false;                          // disable interrupts
      txMsgOk = true;                                   // set true when a valid message is sent
      txFinish = micros();                              // capture end of Tx time
      //
      // ACK message sent (may have failed) so nothing else to do
      //
    } else {
      // failed to start Tx
      // This is treated as non-fatal, in that we simply go round and wait for the next received message
      //
      Serial.print(F("failed to send, code "));
      Serial.println(state,HEX);
      oled.println("Failed to send");
    }

    digitalWrite(LED1, LOW);                            // set LED OFF

    //
    // finished previous received messsage & ACK or a tx timeout
    // either way put module back to listen mode
    //
    radio.sleep();                                      // set to 'sleep' inbetween Tx & Rx modes
  
    // start listening for LoRa packets again
//    Serial.print(F("[Slave] Starting to listen again ... "));
    state = radio.startReceive();
    if (state == ERR_NONE) {
//      Serial.println(F("success!"));
    } else {
      Serial.print(F("failed to set up Rx mode, code "));
      Serial.println(state,HEX);

      oled.println("*** Failed to setup Rx mode ***");
      while (true){
        // this is treated as fatal so set LED to flash
        //
        // could set to 'failsafe' settings as well
        //
        // medium flashing LED
        digitalWrite(LED1,HIGH);
        delay(MEDFLASH);
        digitalWrite(LED1,LOW);
        delay(MEDFLASH);
      }
    }
    
    rxStart = micros();                                   // capture start of receive
    rxTimeout = false;                                    // clear timeout flag
    rxMsgOk = false;                                      // clear message OK flag
    receivedFlag = false;                                 // clear interrupt flag
    // we're ready to receive more packets,
    
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
  // correctiion required to acheive 50Hz as nearly as possible.
  //
  // This is quite crude code, simply waiting for a series of edges to arrive, but seems to work.
  //
  // If the PCA9685 fails to generate the squarewave then the slave will never progress out of calibration.
  //
  // wait for Hi or rising edge
  Serial.print(F("wait for rising edge ...."));
  while (digitalRead(PWM_CAL_IN) == 0)
  {
  }
  Serial.print(F(" wait for falling edge .."));
  // wait for falling edge
  while (digitalRead(PWM_CAL_IN) == 1)
  {
  }
  // falling edge
  Serial.println(F("now start timing loop"));
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
    // faling edge; have we had enough falling edges?
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

  // frequency scaling factor = wanted frequency / measured frequency 
  // this can be used as a multipler when setting the PWM (frame) frequency in
  // the real servo code
  Serial.print(F("PWM frequency scaling factor = "));
  freqScale = timePeriod * SERVO_FREQ / (NUM_CAL_CYCLES * 1000.0 * 1000.0);
  Serial.print(freqScale);
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
  Serial.println(F(" ... success!"));
  
  // now update PCA9685 PWM frequency with corrected value
  pwm.setPWMFreq(SERVO_FREQ * freqScale);  // Analog servos run at ~50 Hz updates
  
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
