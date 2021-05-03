//
//
const String REV            = "vG";             // Master software revision 
const uint8_t REVLEN        = 3;                // s/w revision length, plus terminator
//
// I2C bus addresses
const uint8_t I2C_PWM_ADDR  = 0x40;             // <1000 000> PWM PCA9685 module
const uint8_t I2C_ADC_ADDR  = 0x48;             // <1001 000> ADC ADS1015 module
const uint8_t I2C_DIO_ADDR  = 0x27;             // <0100 111> IO expander MCP23017 module
const uint8_t I2C_OLED_ADDR = 0x3C;             // SSD1306 OLED controller; 0X3C+SA0 - 0x3C or 0x3D

//
// RF link
const float CENTRE_FREQ_CH0   = 458.4775;       // channel 1, legal channels are 1..40
const float RF_CH_SPACE       = 0.025;          // 25kHz = 0.025MHz
const uint8_t  RF_CH_MAX      = 40;             // s/ware channel =0 would be invalid
const uint8_t  RF_CH_DEF      = 3;              // default RF channel
const float BANDWIDTH         = 62.5;           // 62.5 kHz
const uint8_t SPREADINGFACTOR = 7;              //
const uint8_t CODERATIO       = 6;              //
const uint8_t TXPMAX          = 17;             // +17dBm
const uint8_t RXGAIN          = 1;              // maximum

//
// messages
//
const uint8_t NUM_PREAMBLE    = 8;              // pre-amble bits
const uint8_t SYNC_WORD       = 0x12;           // sync word, i.e. network ID; this must be identical in master & Slave
const uint8_t NUM_DIG_CHAN    = 8;              // 8 bits = 1 byte
const uint8_t MAX_ANA_CHAN    = 6;              // max number of analogue (servo) channels
const uint8_t NUM_ANA_CHAN    = 4;              // 4 analogue (servo) channels; command + analogue channels + digital channel byte
const uint8_t LEN_ACKMSG      = 2;              // length of slave ACK messsage; ACK & status byte

const uint8_t MSG_ACK         = 0xA5;           // ACK from slave
const uint8_t MSG_CMND        = 0x5A;           // servo data from master
const uint8_t MSG_FLSF        = 0xAA;           // failsafe servo data from master
//
// status flags from slave
const uint8_t FLG_FAILSAFE    = 0b00000001;     // failsafe activated
const uint8_t FLG_RXTIMEOUT   = 0b00000010;     // Rx timeout error
const uint8_t FLG_MSGERR      = 0b00000100;     // error in received message 

//
// Timeouts
const uint32_t RX_TIMEOUT   = 300000;           // 300,000 usec = 0.3 sec
const uint32_t TX_TIMEOUT   = 500000;           // 500,000 usec = 0.5 sec

//
//
const int16_t MSG_COUNT     = 10;               // number of Tx/Rx cycles between display updates

//
// LED flash times
const int16_t FASTFLASH     = 200;              // 200 msec ON, 200 msec OFF
const int16_t MEDFLASH      = 500;              // 0.5 sec ON, 0.5 sec OFF
const int16_t SLOWFLASH     = 1000;             // 1 sec ON, 1 sec OFF

//
// joystick & servo characteristics
//
const int16_t RANGE               = 255;        // servo data is 0 to RANGE
//
// the VMIN and VMAX are recorded values of the raw ADC values
// channesl 5 & 6 are dummy data
const int16_t vmax[MAX_ANA_CHAN]  = {1104,1100,1104,1099,1100,1100};
const int16_t vmin[MAX_ANA_CHAN]  = {3,7,1,5,3,3};

const int16_t SERVO_DEF           = 0x80;       // sero default mid-position
