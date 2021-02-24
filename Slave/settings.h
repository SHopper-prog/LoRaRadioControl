//
//
//
const String REV            = "vD";             // software revision 
//
// I2C bus addresses
const uint8_t I2C_PWM_ADDR  = 0x40;             // <1000 000> PWM PCA9685 module
const uint8_t I2C_ADC_ADDR  = 0x48;             // <1001 000> ADC ADS1015 module
const uint8_t I2C_DIO_ADDR  = 0x27;             // <0100 111> IO expander module


//
// RF link
#define CENTRE_FREQ_CH0     458.4775            // channel 1, legal channels are 1..40
#define RF_CH_SPACE         0.025               // 25kHz = 0.025MHz
#define RF_CH_MAX           40                  // s/ware channel =0 would be invalid
#define RF_CH_DEF           3                   // default RF channel
#define BANDWIDTH           62.5                // 62.5 kHz
#define SPREADINGFACTOR     7                   //
#define CODERATIO           6                   //
#define TXPMAX              17                  // +17dBm
#define RXGAIN              1                   // maximum

//
// messages
//
#define NUM_PREAMBLE        8                   // pre-amble bits
#define SYNC_WORD           0x12                // sync word, i.e. network ID
#define NUM_DIG_CHAN        8                   // 8 bits = 1 byte
#define NUM_ANA_CHAN        6                   // 6 analogue (servo) channels; command + analogue channels + digital channel byte
#define LEN_ACKMSG          2                   // length of slave ACK messsage; ACK & status byte
const uint8_t SERVOMSGMAX   = 255;              // maximum value for a servo position value

#define MSG_ACK             0xA5                // ACK from slave
#define MSG_CMND            0x5A                // servo data from master
#define MSG_FLSF            0xAA                // failsafe servo data from master
//
// status flags from slave
#define FLG_FAILSAFE        0b00000001          // failsafe activated
#define FLG_RXTIMEOUT       0b00000010          // Rx timeout error
#define FLG_MSGERR          0b00000100          // error in received message 

//
// Timeouts
#define RX_TIMEOUT          4000000             // 3,000,000 usec = 3 sec
#define TX_TIMEOUT          1000000             // 1,000,000 usec = 1 sec

//
// LED flash times
#define FASTFLASH           200                 // 200 msec ON, 200 msec OFF
#define MEDFLASH            500                 // 0.5 sec ON, 0.5 sec OFF
#define SLOWFLASH           1000                // 1 sec ON, 1 sec OFF

// servo settings
//
byte SERVO_MID              = 0x80;              // servo default mid-position message value
const int16_t SERVO_FREQ    = 50;               // 50Hz
uint8_t SERVO_CAL           = 7;                // PCA9685 PWM clock calibratiion channel
uint16_t NUM_CAL_CYCLES     = 100;              // number of PWM calibration cycles to measure
//
// the frequency scale is used to correct the PCA9685 oscillator frequency, based on previous measurements
const float FREQ_SCALE      = 0.97;
//
// The max and mnin values are determined from measurements on actual servos, and may be different for each servo type
// They are the min and max values for the pulse width relative to the 4096 total count, assuming a 20msec frame
// As a guide:
//  120 = 0.58 msec
//  512 = 2.5 msec
//
const int16_t SERVOMIN[NUM_ANA_CHAN]  = {120,120,120,120,120,120};
const int16_t SERVOMAX[NUM_ANA_CHAN]  = {512,512,512,512,512,512};
// the invert array is used to reverse the servo direction when set to 1
const int8_t SERVOINV[NUM_ANA_CHAN]   = {0,1,0,0,0,0,};
// the offset are the start positions for each servo pulse so the pulses for each servo are at a different time so no overlap
const int16_t PWMOFF[NUM_ANA_CHAN]    = {0,512,1024,1536,2048,2560};
