//
//
//
const String REV            = "v2.01.0b";       // Slave software revision 
//
// I2C bus addresses
const uint8_t I2C_PWM_ADDR  = 0x40;             // <1000 000> PWM PCA9685 module
const uint8_t I2C_ADC_ADDR  = 0x48;             // <1001 000> ADC ADS1015 module
const uint8_t I2C_DIO_ADDR  = 0x27;             // <0100 111> IO expander module
const uint8_t I2C_OLED_ADDR = 0x3C;             // SSD1306 OLED controller; 0X3C+SA0 - 0x3C or 0x3D


//
// RF link
const float CENTRE_FREQ_CH0   = 458.4775;       // channel 1, legal channels are 1..40
const float RF_CH_SPACE       = 0.025;          // 25kHz = 0.025MHz
const uint8_t RF_CH_MAX       = 40;             // s/ware channel =0 would be invalid
const uint8_t RF_CH_DEF       = 3;              // default RF channel
const float BANDWIDTH         = 62.5;           // 62.5 kHz
const uint8_t SPREADINGFACTOR = 7;              //
const uint8_t CODERATIO       = 6;              //
const uint8_t TXPMAX          = 17;             // +17dBm
const uint8_t RXGAIN          = 1;              // maximum

//
// SX1278 FSM states
//
const uint8_t LORA_INIT         = 0;              // initialisation state
const uint8_t LORA_INITWAIT     = 1;              // initialisation wait state
const uint8_t LORA_RXSTART      = 2;              // Rx ready start state
const uint8_t LORA_RXREADY      = 3;              // Rx ready state
const uint8_t LORA_RXREAD       = 4;              // Rx read state
const uint8_t LORA_TXWAIT       = 5;              // Tx wait state
const uint8_t LORA_TXSLEEP      = 6;              // Tx sleep state
const uint8_t LORA_TXREADY      = 7;              // Tx ready state
const uint8_t LORA_RXSLEEP      = 8;              // RX sleep state
const uint8_t LORA_RXTIMEOUT    = 9;              // Rx timeout state
const uint8_t LORA_RXERROR      = 10;             // Rx data read error
const uint8_t LORA_TXTIMEOUT    = 11;             // Tx timeout state
const uint8_t LORA_TXERROR      = 12;             // Tx error state
const uint8_t LORA_FLASH        = 13;             // LED flash (on)
const uint8_t LORA_FLASH2       = 14;             // LED flash (off)

//
// messages
//
const uint8_t NUM_PREAMBLE  = 8;                  // pre-amble bits
const uint8_t SYNC_WORD     = 0x12;               // sync word, i.e. network ID; this must be identical in master & Slave
const uint8_t NUM_DIG_CHAN  = 8;                  // 8 bits = 1 byte
const uint8_t MAX_ANA_CHAN  = 6;                  // max number of analogue channels
const uint8_t NUM_ANA_CHAN  = 4;                  // 4 analogue (servo) channels; command + analogue channels + digital channel byte
const uint8_t LEN_ACKMSG    = 2;                  // length of slave ACK message; ACK & status byte
const uint8_t SERVOMSGMAX   = 255;                // maximum value for a servo position value

const uint8_t MSG_ACK       = 0xA5;               // ACK from slave
const uint8_t MSG_CMND      = 0x5A;               // servo data from master
const uint8_t MSG_FLSF      = 0xAA;               // fail-safe servo data from master
//
// status flags from slave
const uint8_t FLG_FAILSAFE  = 0b00000001;         // fail-safe activated
const uint8_t FLG_RXTIMEOUT = 0b00000010;         // Rx time-out error
const uint8_t FLG_MSGERR    = 0b00000100;         // error in received message 
const uint8_t FLG_LOWBAT    = 0b00001000;         // low battery voltage

//
// Timeouts
const uint32_t ERR_TIMEOUT  = 1000000;            // 1,000,000 usec = 1 sec time-out for error states
const uint32_t RX_TIMEOUT   = 1000000;            // 1,000,000 usec = 1 sec time-out for loss of Rx data
const uint32_t TX_TIMEOUT   = 500000;             // 500,000 usec = 0.5 sec time-out for Tx failure
const uint32_t DIR_TIMEOUT  = 20000;              // 20,000 usec = 20msec time-out for Rx to Tx chageover
const uint8_t FSAFE_COUNT   = 6;                  // number of consecutive Rx message time-outs before 'fail-safe' is invoked

const uint32_t ADC_TIMEOUT  = 3000000;            // cycle time between ADC reads = 3 sec
const uint32_t DISP_TIMEOUT = 2000000;            // cycle time between OLED display updates = 2 sec
//
// LED flash times
const uint8_t FASTFLASH     = 200;                // 200 msec ON, 200 msec OFF
const uint8_t MEDFLASH      = 500;                // 0.5 sec ON, 0.5 sec OFF
const uint8_t SLOWFLASH     = 1000;               // 1 sec ON, 1 sec OFF

//
// display settings
const uint8_t MSG_COUNT     = 10;                 // one line of display updated every 10 messages
const uint8_t DISP_INIT      = 0;                 // display FSM initialisation state
const uint8_t DISP_WRITE     = 1;                 // display FSM write state
const uint8_t DISP_WAIT      = 2;                 // display FSM wait state
//
// internal ADC value
const float ADC_FSD         = 3.3;                // internal ADC FSD voltage
const float ADC_ATTEN       = 4.3;                // external voltage attenuator ratio to internal ADC  
const float LOW_BAT         = 11.0;               // low battery alarm voltage

const uint8_t ADC_INIT      = 0;                  // ADC FSM initialisation state
const uint8_t ADC_READ      = 1;                  // ADC FSM read state
const uint8_t ADC_WAIT      = 2;                  // ADC FSM wait state

// servo settings
//
byte SERVO_MID              = 0x80;               // servo default mid-position message value
const int16_t SERVO_FREQ    = 50;                 // 50Hz
uint8_t SERVO_CAL           = 7;                  // PCA9685 PWM clock calibration channel
uint16_t NUM_CAL_CYCLES     = 100;                // number of PWM calibration cycles to measure
//
// the frequency scale is used to correct the PCA9685 oscillator frequency, based on previous measurements
const float FREQ_SCALE      = 0.97;
//
// The max and mnin values are determined from measurements on actual servos, and may be different for each servo type
// They are the min and max values for the pulse width relative to the 4096 total count, assuming a 20msec frame
// As a guide:
// TowerPro SG90 digital servo, gives 180deg movement with values of
//  120 = 0.58 msec
//  512 = 2.5 msec
//
// Standard values for 90deg movement are:
//  205 = 1.0009 msec
//  409 = 1.997 msec
//
// analogue channels defined by NUM_ANA_CHAN:
//
//	chan 0 = drive motor
//	chan 1 = rudder 
//	chan 2 = dive planes
//	chan 3 =
//
const int16_t SERVOMIN[MAX_ANA_CHAN]  = {205,205,205,205,205,205};
const int16_t SERVOMAX[MAX_ANA_CHAN]  = {409,409,409,409,409,409};
// the invert array is used to reverse the servo direction when set to 1
const int8_t SERVOINV[MAX_ANA_CHAN]   = {0,0,1,0,0,0,};
// the offset are the start positions for each servo pulse so the pulses for each servo are at a different time so no overlap
const int16_t PWMOFF[MAX_ANA_CHAN]    = {0,512,1024,1536,2048,2560};
//
// the bias values are used to tweak the message values such that message value 0x80 is a true 'neutral'
// The value -10 is for the G2 Hydra15 ESC & motor
const int16_t BIAS[MAX_ANA_CHAN]  = {-10,0,0,0,0,0};

//
// digital channels settings
//
//  bit 0 = 'surface'         1 = on
//  bit 1 = 'dive'            1 = on
//  bit 2 = not used          1 = on
//  bit 3 = emergency buoy    1 = off, so it is usually held on until a fault is detected
//  bit 4 = not used          1 = on
//  bit 5 = not used          1 = on
//  bit 7 = clear status byte 1 = on
//
// a '0' in the bit position will invert the digital channel control
const int8_t DIGITALINV[NUM_DIG_CHAN] = {0,0,0,0x08,0,0,0,0};
//
// Fail-safe values set by '1' = not activated, '0' = activated
// the fail-safe values are thus 
//  bit 0 activated to select 'surface'
//  bit 3 activated (due to inversion shown above) to hold the emergency buoy
const uint8_t SWFAILSAFE    =0xf6;                 // fail-safe values 
//
