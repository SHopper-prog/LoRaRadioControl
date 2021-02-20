//
//
const String REV            = "vC";             // software revision 
//
// I2C bus addresses
const uint8_t I2C_PWM_ADDR  = 0x40;             // <1000 000> PWM PCA9685 module
const uint8_t I2C_ADC_ADDR  = 0x48;             // <1001 000> ADC ADS1015 module

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
#define NUM_ANA_CHAN        6                   // 6 analogue (servo) channels; sommand + analogue channels + digital channel byte
#define LEN_ACKMSG          2                   // length of slave ACK messsage; ACK & status byte

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
#define RX_TIMEOUT      2000000         // 2,000,000 usec = 2 sec
#define TX_TIMEOUT      1000000         // 1,000,000 usec = 1 sec

//
//
const int16_t NUM_CYCLES    = 10;               // number of Tx/Rx cycles between display updates

//
// LED flash times
#define FASTFLASH           200                 // 200 msec ON, 200 msec OFF
#define MEDFLASH            500                 // 0.5 sec ON, 0.5 sec OFF
#define SLOWFLASH           1000                // 1 sec ON, 1 sec OFF

//
// joystick & servo characteristics
//
const int16_t RANGE               = 255;        // servo data is 0 to RANGE
//
// the VMIN and VMAX are recorded values of the raw ADC values
// channesl 5 & 6 are dummy data
const int16_t vmax[NUM_ANA_CHAN]  = {1104,1100,1104,1099,1100,1100};
const int16_t vmin[NUM_ANA_CHAN]  = {3,7,1,5,3,3};

const int16_t SERVO_DEF           = 0x80;       // sero default mid-position
