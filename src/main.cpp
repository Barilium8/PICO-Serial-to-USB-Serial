
#include <Arduino.h>
#include <pico/stdlib.h> // added to make UF2 button work, see SR1_BUTNUM_HELP
#include <pico/bootrom.h> // added to make UF2 button work, see SR1_BUTNUM_HELP

#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "AD6886_ADC.h"
#include "PCA9702_GATE.h"

#include "dma_ws2812.h"
#include "adc_4051_dma.h"
uint16_t adcInputs[8] = {0,0,0,0, 0,0,0,0};

// DMA can be used with PIO:
// https://gregchadwick.co.uk/blog/playing-with-the-pico-pt2/
// https://github.com/adafruit/Adafruit_NeoPXL8

// More advanced Arduino RP2040 board support that includes the pioasm.exe
// https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
// found from https://github.com/LifeWithDavid/Raspberry-Pi-Pico-PIO/blob/7a2236b4c6fcffb71231792e9fab8367e1386c0c/Episode%2013%20Arduino%20IDE%20files.txt

// Online PioAsm.exe (probably the best choice for now to make some progress)
// https://wokwi.com/tools/pioasm

#include "hardware/uart.h"
#include <MIDI.h> // by Francois Best
#include <Adafruit_TinyUSB.h>

#include "ScanShiftRegs.h"
#include "Log.h"
#include "SliderLib.h"

#include "../WTS_SHARED/ParameterIndex.h"
#include "../WTS_SHARED/ControllerEnums.h"
#include "../WTS_SHARED/ControllerMidi.h"
#include "../WTS_SHARED/GlobalDefinitions.h"


#define PRINTB32(Num) for(int i=0; i<32; i++) { if(i%8==0){Serial0_USB.print("    ");} if(i%2Serial0_USBerial.prinSerial0_USB;}  Serial0_USB.write(((Num >> i) & 1) == 1 ?Serial0_USB '0'); } Serial0_USB0_USB.println();
#define PRINTB8(Num) for(int i=0; i<8; i++) { Serial0_USB.write(((Num >> i) & 1) == 1 ? '1' : '0'Serial0_USBerial.print(" ");

#define PICO_DEFAULT_LED_GPIO 25
#define HW_PROGRAM_BUTTON 26 // 26 new hw // was 2 // was 9

// DAC Reset & Mute
#define DAC_MUTE_GPIO 5     // pin7 GPIO 5
#define DAC_RESET_GPIO 4     // pin6 GPIO 4

// I2C setup for DAC and HeadPhone Amp
#define I2C1_SDA 2   // pin4 GPIO 2
#define I2C1_SCL 3    // pin5 GPIO 3

// Shift Reg PIN def
#define SRX_LATCH_GPIO 18     // pin24 GPIO 18
#define SRX_CLK_GPIO 19       // pin25 GPIO 19
#define SR1_DATA_IN_GPIO 20   // pin26 GPIO 20  Shift Reg port 1
#define SR2_DATA_IN_GPIO 21   // pin27 GPIO 21  Shift Reg port 2

// MIDI PIN def
#define MIDI_LED_GPIO 22  // pin29 GPIO 22
//#define MIDI_RX1_GPIO  1  // pin2 GPIO 1
//#define MIDI_TX1_GPIO  0  // pin1 GPIO 0

// Fast LED PIN def
#define NEOPIX_GPIO  11    // pin15 GPIO 11
#define NUM_NEOPIX 47 // +1

#define SR0 0
#define SR1 1

struct GRBPixelType {
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;
    //uint8_t alpha = 0; // not used
};

struct aniPixelType {
    GRBPixelType theRGBStart = {0,0,0}; // initial color val
    GRBPixelType theRGBEnd = {0,0,0};   // final color val
    uint16_t duration = 0;              // ms - how it takes to complete transition, 0 = forever
    uint16_t elpTime = 0;               // ms - how it takes to complete transition, 0 = forever
    uint8_t theAniType = 0;             // what kind of transition, colLEPR, blink
};

GRBPixelType theLEDs[NUM_NEOPIX];      // this is the LED data buffer that gets sent to the LEDs
GRBPixelType theTempLEDs[NUM_NEOPIX];  // this is the 'temp' LED data buffer that temporiliy stores the LED vals
aniPixelType temporalLEDs[NUM_NEOPIX]; // this is the transitioning data for animating the LEDs

const float dimLED_R = 0.20f; // used to dim inbound sysex LED msgs
const float dimLED_G = 0.15f; // used to dim inbound sysex LED msgs
const float dimLED_B = 0.20f; // used to dim inbound sysex LED msgs

#define BRIGHTNESS_DIM 3
#define BRIGHTNESS_BRIGHT 31 // 5 bits 0-31
uint8_t gNeoPixBrightness = 11; //BRIGHTNESS_DIM; // 0-31 5bits

// Create 2x UART based MIDI ports on PICO: MIDI_5_PIN_UART0  &  MIDI_PICO_TO_CM_UART1

// Creates MIDI on Serial1 aka UART0 (GPIO 0/1) for 5-Pin MIDI
struct MIDI_5_PIN_Settings : public midi::DefaultSettings{
   static const bool UseRunningStatus = false;
   static const bool HandleNullVelocityNoteOnAsNoteOff = true;
   static const bool Use1ByteParsing = true;
   static const long BaudRate = 31'250;
   static const unsigned SysExMaxSize = 128;
};
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial1, MIDI_5_PIN_UART0, MIDI_5_PIN_Settings);

// Creates MIDI on Serial2 aka UART1 (GPIO 8/9) for PICO to ComputeModule MIDI "bus"
struct MIDI_UART1_Settings : public midi::DefaultSettings{
   static const bool UseRunningStatus = false;
   static const bool HandleNullVelocityNoteOnAsNoteOff = true;
   static const bool Use1ByteParsing = true;
    // tested at 2'000'000 and works
   static const long BaudRate = 1'000'000; //38'400; //31'250; //460'800; // 460800 baud tested output on PICO pin via analyser. Reqs 2MHz scan on the analyser! :-)
   static const unsigned SysExMaxSize = 128;
};
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial2, MIDI_PICO_TO_CM_UART1, MIDI_UART1_Settings);


// Create two MIDI wrapped serial objects over USB: MIDI_USB_SERIAL_DEV  &  MIDI_USB_DEV

// The 1st Serial on USB happens automagically, would look like this...
// Adafruit_USBD_CDC Serial0_USB;  (aka Serial)

// create USB_SERIAL (CDC) object
Adafruit_USBD_CDC TinyUSB_SerialMIDI; // declare a USB 'Serial' port and wrap it in a MIDI intrerface
struct USB_SM_Dev_Settings : public midi::DefaultSettings{
    static const bool UseRunningStatus = false;
    static const bool HandleNullVelocityNoteOnAsNoteOff = true;
    static const bool Use1ByteParsing = true;
    static const long BaudRate = 115'200; // 31250;
    static const unsigned SysExMaxSize = 128;
};
MIDI_CREATE_CUSTOM_INSTANCE(Adafruit_USBD_CDC, TinyUSB_SerialMIDI, MIDI_USB_SERIAL_DEV, USB_SM_Dev_Settings);

// create USB_MIDI object
Adafruit_USBD_MIDI TinyUSB_MIDI; // declare a USB 'MIDI' port and wrap it in a MIDI intrerface
struct USBDev_Settings : public midi::DefaultSettings{
    static const bool UseRunningStatus = false;
    static const bool HandleNullVelocityNoteOnAsNoteOff = true;
    static const bool Use1ByteParsing = true;
    static const long BaudRate = 115'200; // 31250;
    static const unsigned SysExMaxSize = 128;
};
MIDI_CREATE_CUSTOM_INSTANCE(Adafruit_USBD_MIDI, TinyUSB_MIDI, MIDI_USB_DEV, USBDev_Settings);


enum class MIDI_Destinations_Enums // : uint8_t
{
    UART0_5Pin_MIDI,       // 5-Pin DIN on back of WTS
    UART1_Pico_To_CM_MIDI, // Hard wired connection - PICO <-> CM5
    USB_MIDI,              // MIDI over USB to PC (DAW)
    USB_Serial1,           // Raw Serial to PC for development ONLY (no debug strings on this port)
    END_OF_LIST,
};

// Change the destination for the 'PICO to CM' MIDI msgs here
// Production = UART1_Pico_To_CM_MIDI
// PC/MAC dev mode = USB_Serial1_Raw - sends Raw MIDI bytes over USB serial so Synth Engine can mimic Linux mode
MIDI_Destinations_Enums PICO_TO_CM = MIDI_Destinations_Enums::USB_MIDI;                 // aka TinyUSB_MIDI - oringial way to debug over USB
//MIDI_Destinations_Enums PICO_TO_CM = MIDI_Destinations_Enums::USB_Serial1;              // aka TinyUSB_SerialMIDI - serial port MIDI debug over USB
//MIDI_Destinations_Enums PICO_TO_CM = MIDI_Destinations_Enums::UART1_Pico_To_CM_MIDI     // aka Serial2 - Chip (UART) to Chip (UART) production release

// this is used to 'remember' which MIDI port was the last to send a msg
// if it changes the synth engine needs to be notified via MIDI CC msg: WTS_SRC_PORT
wtsSourcePort prevMidiMsgSrcPort = wtsSourcePort::SRC_MIDI_CNTRLR;


// Static CV values use to compare newly read data to
//uint16_t cvPrevVoltages[8], cvPrev2Voltages[8];
uint16_t AD8668Voltages[8], cvPrevAveVoltages[8], cvPrev2AveVoltages[8], cvSumVoltages[8];
static const uint8_t cvToMidiLUT[16] = {
    static_cast<uint8_t>(ControllerCvExpEnums::CV_1_MSB),
    static_cast<uint8_t>(ControllerCvExpEnums::CV_1_LSB),
    static_cast<uint8_t>(ControllerCvExpEnums::CV_2_MSB),
    static_cast<uint8_t>(ControllerCvExpEnums::CV_2_LSB),
    static_cast<uint8_t>(ControllerCvExpEnums::CV_3_MSB),
    static_cast<uint8_t>(ControllerCvExpEnums::CV_3_LSB),

    static_cast<uint8_t>(ControllerCvExpEnums::CV_4_MSB),
    static_cast<uint8_t>(ControllerCvExpEnums::CV_4_LSB),
    static_cast<uint8_t>(ControllerCvExpEnums::CV_5_MSB),
    static_cast<uint8_t>(ControllerCvExpEnums::CV_5_LSB),
    static_cast<uint8_t>(ControllerCvExpEnums::CV_6_MSB),
    static_cast<uint8_t>(ControllerCvExpEnums::CV_6_LSB),

    static_cast<uint8_t>(ControllerCvExpEnums::Exp_Pedal_1_MSB),
    static_cast<uint8_t>(ControllerCvExpEnums::Exp_Pedal_1_LSB),
    static_cast<uint8_t>(ControllerCvExpEnums::Exp_Pedal_2_MSB),
    static_cast<uint8_t>(ControllerCvExpEnums::Exp_Pedal_2_LSB),
};

uint8_t PCA9702PrevGates;
static const uint8_t gateToMidiLUT[15] = {
    static_cast<uint8_t>(ControllerGateEnums::Gate_1),

    static_cast<uint8_t>(ControllerGateEnums::Gate_2),
    static_cast<uint8_t>(ControllerGateEnums::Gate_3),
    static_cast<uint8_t>(ControllerGateEnums::Gate_4),
    static_cast<uint8_t>(ControllerGateEnums::Gate_5),
    static_cast<uint8_t>(ControllerGateEnums::Gate_6),
    static_cast<uint8_t>(ControllerGateEnums::Gate_Pedal),
    static_cast<uint8_t>(ControllerGateEnums::Analog_Clock_Tic),
};

// create Shift Reg objects
const uint8_t sr1_numModules = 2;
uint8_t sr1_modList[sr1_numModules] = { CL_WTS_V2B_SR1_A, CL_WTS_V2B_SR1_B };
const uint16_t sr1_holdTime = 2000;
ScanShiftRegs sr1Port(SR0, sr1_numModules, sr1_modList);

const uint8_t sr2_numModules = 3;
uint8_t sr2_modList[sr2_numModules] = { CL_WTS_V2B_SR2_A, CL_WTS_V2B_SR2_B, CL_WTS_V2B_SR2_C };
const uint16_t sr2_holdTime = 2000;
ScanShiftRegs sr2Port(SR1, sr2_numModules, sr2_modList);


uint8_t theMIDIChan = 16;
const uint8_t  NOT_PRESSED = 0;
const uint8_t  PRESSED = 1;
const uint8_t  press = 0;
const uint8_t  latch = 1;
const uint8_t  ROTATION_INC = 1;
const uint8_t  ROTATION_DEC = -1;

const uint8_t SLOW_INC = 0;
const uint8_t NORMAL_INC = 1;
const uint8_t FAST_INC = 2;
const uint8_t WARP_FACTOR_9 = 3;

// LED HARDWARE (v3) defs
const uint8_t LED_VAL = 0;
const uint8_t LED_LFO = 1;
const uint8_t LED_ENV = 2;
const uint8_t LED_EXP = 3;

const uint8_t LED_FILTER_SHORTCUT = 4;

const uint8_t LED_USER5 = 5;
const uint8_t LED_USER1 = 6;
const uint8_t LED_USER2 = 8; // was 7 fix for button lights 2&4 reversed
const uint8_t LED_USER4 = 7; // was 8 fix for button lights 2&4 reversed

const uint8_t LED_VOICE_MOD = 9;
const uint8_t LED_FILTER_MOD = 10;
const uint8_t LED_PATH_MOD = 11;
const uint8_t LED_TERRAIN_MOD = 12;

const uint8_t LED_MODULATION_ENC0 = 13;
const uint8_t LED_MODULATION_ENC1 = 14;
const uint8_t LED_MODULATION_ENC2 = 15;
const uint8_t LED_MODULATION_ENC3 = 16;

const uint8_t LED_LATCH = 17; // aka HOLD
const uint8_t LED_PLAY = 18;  // aka AUDITION
const uint8_t LED_FAVS = 19;

const uint8_t LED_MENU_ENC5 = 20;
const uint8_t LED_MENU_ENC4 = 21;
const uint8_t LED_MENU_ENC3 = 22;
const uint8_t LED_MENU_ENC2 = 23;
const uint8_t LED_MENU_ENC1 = 24;
const uint8_t LED_MENU_ENC0 = 25;

const uint8_t LED_MIX_PAN = 26; // Mixer Pan
const uint8_t LED_MIX_VOL = 27; //aka Mixer Vol

const uint8_t LED_POWER = 28;

const uint8_t LED_WTS = 29;
const uint8_t LED_SYNTH_A = 30;
const uint8_t LED_SYNTH_B = 31;
const uint8_t LED_SYNTH_C = 32;
const uint8_t LED_SYNTH_D = 33;

const uint8_t LED_AMP_SHORTCUT = 34;

const uint8_t LED_USER0 = 35;
const uint8_t LED_USER3 = 36;

const uint8_t LED_SEQ = 37;
const uint8_t LED_ARP = 38;

const uint8_t LED_SEQARP_REC = 39;
const uint8_t LED_SEQARP_STOP = 40;
const uint8_t LED_SEQARP_PLAYPAUSE = 41;

const uint8_t LED_FX_MOD = 42;

const uint8_t LED_SETTINGS_MOD = 43;
const uint8_t LED_PATCHLIB_MOD = 44;
const uint8_t LED_MIXER_MOD = 45;


struct ledGroupInfoType {
    uint8_t start = 0;
    uint8_t end = 0;
};

const uint8_t NUM_LED_GROUPS = 15;
const uint8_t MAX_NUM_LEDS_PER_GROUP = 8;
const uint8_t NO_LED = 255;
// THE NEXT VERSION OF THE HW MIGHT NOT HAVE CONTUIGUOUS LED GROUPS...
// SO MAKE THIS AN ARRAY (NOT START - END) AND LOOP THRU THEM.
// AND MAKE A "NONE" LED TYPE TO PAD THE EMPTY SLOTS
// ledGroupInfoType ledGroupInfo[NUM_LED_GROUPS] = {
//   /*00*/ {LED_LFO, LED_EXP,}, // need to add Val
//   /*01*/ {LED_USER0, LED_USER3,},
//   /*02*/ {LED_VOICE_MOD, LED_SETTINGS_MOD,},
//   /*03*/ {LED_LATCH, LED_LATCH,},
//   /*04*/ {LED_PLAY, LED_PLAY,},
//   /*05*/ {LED_FAVS, LED_FAVS,},
//   /*06*/ {LED_MENU_ENC5, LED_MENU_ENC0,},
//   /*07*/ {LED_MIX_PAN, LED_MIX_VOL,},
//   /*08*/ {LED_POWER, LED_POWER,},
//   /*09*/ {LED_WTS, LED_WTS,},
//   /*10*/ {LED_SYNTH_A, LED_SYNTH_D,},
//   /*11*/ {LED_AMP_SHORTCUT, LED_FILTER_SHORTCUT,},
//   // for next HW ver if we add Modulation enc LEDs
//   /*12*/ //{LED_MOD_ENC3, LED_MOD_ENC0,},
// };

uint8_t ledGroupInfo[NUM_LED_GROUPS][MAX_NUM_LEDS_PER_GROUP] = {
  /*00*/ {LED_VAL, LED_LFO, LED_ENV, LED_EXP,                                                           NO_LED, NO_LED, NO_LED, NO_LED},
  /*01*/ {LED_USER0, LED_USER1, LED_USER2, LED_USER3, LED_USER4, LED_USER5,                             NO_LED, NO_LED},
  /*02*/ {LED_VOICE_MOD, LED_FILTER_MOD, LED_PATH_MOD, LED_TERRAIN_MOD, LED_FX_MOD, LED_SETTINGS_MOD, LED_PATCHLIB_MOD, LED_MIXER_MOD},
  /*03*/ {LED_LATCH,                                                                                    NO_LED, NO_LED, NO_LED, NO_LED, NO_LED, NO_LED, NO_LED},
  /*04*/ {LED_PLAY,                                                                                     NO_LED, NO_LED, NO_LED, NO_LED, NO_LED, NO_LED, NO_LED},
  /*05*/ {LED_FAVS,                                                                                     NO_LED, NO_LED, NO_LED, NO_LED, NO_LED, NO_LED, NO_LED},
  /*06*/ {LED_MENU_ENC5, LED_MENU_ENC4, LED_MENU_ENC3, LED_MENU_ENC2, LED_MENU_ENC1, LED_MENU_ENC0,     NO_LED, NO_LED},
  /*07*/ {LED_MIX_PAN, LED_MIX_VOL,                                                                     NO_LED, NO_LED, NO_LED, NO_LED, NO_LED, NO_LED},
  /*08*/ {LED_POWER,                                                                                    NO_LED, NO_LED, NO_LED, NO_LED, NO_LED, NO_LED, NO_LED},
  /*09*/ {LED_WTS,                                                                                      NO_LED, NO_LED, NO_LED, NO_LED, NO_LED, NO_LED, NO_LED},
  /*10*/ {LED_SYNTH_A, LED_SYNTH_B, LED_SYNTH_C, LED_SYNTH_D,                                           NO_LED, NO_LED, NO_LED, NO_LED},
  /*11*/ {LED_AMP_SHORTCUT, LED_FILTER_SHORTCUT,                                                        NO_LED, NO_LED, NO_LED, NO_LED, NO_LED, NO_LED},
  /*12*/ {LED_MODULATION_ENC0, LED_MODULATION_ENC1, LED_MODULATION_ENC2, LED_MODULATION_ENC3,           NO_LED, NO_LED, NO_LED, NO_LED},
  /*13*/ {LED_SEQ, LED_ARP,                                                                             NO_LED, NO_LED, NO_LED, NO_LED, NO_LED, NO_LED},
  /*14*/ {LED_SEQARP_REC, LED_SEQARP_STOP, LED_SEQARP_PLAYPAUSE,                                                NO_LED, NO_LED, NO_LED, NO_LED, NO_LED}
};


// the magic LUT that maps the inbound SysEx LED#/val msg to the proper HW Cntrlr LEDs
// matches - enum class Numbers : unsigned char
// this is dumb!!!

uint8_t ledIndexLUT[47] = {
    LED_POWER,
    LED_WTS,

    LED_SYNTH_A,
    LED_SYNTH_B,
    LED_SYNTH_C,
    LED_SYNTH_D,

    LED_VAL,
    LED_LFO,
    LED_ENV,
    LED_EXP,

    LED_AMP_SHORTCUT,
    LED_FILTER_SHORTCUT,

    LED_USER0,
    LED_USER1,
    LED_USER2,
    LED_USER3,
    LED_USER4,
    LED_USER5,

    LED_MIX_VOL,
    LED_MIX_PAN,

    LED_MENU_ENC0,
    LED_MENU_ENC1,
    LED_MENU_ENC2,
    LED_MENU_ENC3,
    LED_MENU_ENC4,
    LED_MENU_ENC5,

    LED_VOICE_MOD,
    LED_TERRAIN_MOD,
    LED_PATH_MOD,
    LED_FILTER_MOD,
    LED_MIXER_MOD,
    LED_PATCHLIB_MOD,
    LED_FX_MOD,
    LED_SETTINGS_MOD,

    LED_LATCH,
    LED_PLAY,
    LED_FAVS,

    LED_SEQ,
    LED_ARP,
    LED_SEQARP_REC,
    LED_SEQARP_STOP,
    LED_SEQARP_PLAYPAUSE,

    LED_MODULATION_ENC0,
    LED_MODULATION_ENC1,
    LED_MODULATION_ENC2,
    LED_MODULATION_ENC3
};


// GRBX
const GRBPixelType theColBlack = {0,0,0};
const GRBPixelType theColWhite = {12,10,11}; //0x12'10'11'00;
const GRBPixelType theColRed = {0,14,0};     //0x00'14'00'00;
const GRBPixelType theColRed2 = {1,13,1};    //0x00'14'00'00;
const GRBPixelType theColGreen = {16,0,0};   //0x16'00'00'00;
const GRBPixelType theColBlue = {0,0,16};    //0x00'00'16'00;
const GRBPixelType theColYellow = {16,13,0};
const GRBPixelType theColPurple = {0,13,13}; //0x00'13'13'00;
const GRBPixelType theColOrange = {7,14,0};  //0x07'14'00'00;
const GRBPixelType theColTeal = {14,2,14};   //0x14'02'14'00;

//==============================================================================

// v3 button defs
// Shift Reg Bus 0 (SR0)
const uint8_t SR0_BUTNUM_FILTER_SHORTCUT = 0;
const uint8_t SR0_BUTNUM_AMP_SHORTCUT = 1;

const uint8_t SR0_BUTNUM_SYNTH_A_SOLO = 2;
const uint8_t SR0_BUTNUM_SYNTH_A_MUTE = 3;
const uint8_t SR0_BUTNUM_SYNTH_B_SOLO = 4;
const uint8_t SR0_BUTNUM_SYNTH_B_MUTE = 5;
const uint8_t SR0_BUTNUM_SYNTH_C_SOLO = 6;
const uint8_t SR0_BUTNUM_SYNTH_C_MUTE = 7;
const uint8_t SR0_BUTNUM_SYNTH_D_SOLO = 8;
const uint8_t SR0_BUTNUM_SYNTH_D_MUTE = 9;
const uint8_t SR0_BUTNUM_MIX_PAN_MAC = 10;

const uint8_t SR0_BUTNUM_MENU_ENC0 = 11;
const uint8_t SR0_BUTNUM_MENU_ENC1 = 12;
const uint8_t SR0_BUTNUM_MENU_ENC2 = 13;
const uint8_t SR0_BUTNUM_MENU_ENC3 = 14;
const uint8_t SR0_BUTNUM_MENU_ENC4 = 15;
const uint8_t SR0_BUTNUM_MENU_ENC5 = 16;

const uint8_t SR0_BUTNUM_LOW_INIT = 17;
const uint8_t SR0_BUTNUM_RND = 18;
const uint8_t SR0_BUTNUM_LATCH = 19; // aka Hold
const uint8_t SR0_BUTNUM_PLAY = 20;  // aka Aud(ition)
const uint8_t SR0_BUTNUM_FAVS = 21;
const uint8_t SR0_BUTNUM_UNDO = 22;
const uint8_t SR0_BUTNUM_COPY = 23;
const uint8_t SR0_BUTNUM_REDO = 24;
const uint8_t SR0_BUTNUM_PASTE = 25;

ControllerButtonEnums SR0_ControllerButtonToSynthEnumLUT[26] = {
  ControllerButtonEnums::FilterButton,
  ControllerButtonEnums::AmpButton,

  ControllerButtonEnums::SoloButtonA,
  ControllerButtonEnums::MuteButtonA,
  ControllerButtonEnums::SoloButtonB,
  ControllerButtonEnums::MuteButtonB,
  ControllerButtonEnums::SoloButtonC,
  ControllerButtonEnums::MuteButtonC,
  ControllerButtonEnums::SoloButtonD,
  ControllerButtonEnums::MuteButtonD,
  ControllerButtonEnums::VolMacroPanButton,

  ControllerButtonEnums::MenuButton0,
  ControllerButtonEnums::MenuButton1,
  ControllerButtonEnums::MenuButton2,
  ControllerButtonEnums::MenuButton3,
  ControllerButtonEnums::MenuButton4,
  ControllerButtonEnums::MenuButton5,

  ControllerButtonEnums::InitButtonRelease, // Specificly RELEASE
  ControllerButtonEnums::RndButtonRelease,  // Specificly RELEASE

  ControllerButtonEnums::LatchButton,
  ControllerButtonEnums::PlayButton,
  ControllerButtonEnums::FavsButton,

  ControllerButtonEnums::UndoButtonRelease, // Specificly RELEASE
  ControllerButtonEnums::CopyButtonRelease, // Specificly RELEASE
  ControllerButtonEnums::RedoButtonRelease, // Specificly RELEASE
  ControllerButtonEnums::PasteButtonRelease // Specificly RELEASE
};

//==============================================================================

// Shift Reg Bus 1 (SR1)
const uint8_t SR1_BUTNUM_LEARN = 0;

const uint8_t SR1_BUTNUM_LFO = 1;
const uint8_t SR1_BUTNUM_ENV = 2;
const uint8_t SR1_BUTNUM_SEQ = 3;

const uint8_t SR1_BUTNUM_MODULATE_ENC0 = 4;
const uint8_t SR1_BUTNUM_MODULATE_ENC1 = 5;
const uint8_t SR1_BUTNUM_MODULATE_ENC2 = 6;
const uint8_t SR1_BUTNUM_MODULATE_ENC3 = 7;

const uint8_t SR1_BUTNUM_ROW_UP = 8;
const uint8_t SR1_BUTNUM_ROW_DOWN = 9;

const uint8_t SR1_BUTNUM_WTS = 10;
const uint8_t SR1_BUTNUM_SYNTH_A = 11;
const uint8_t SR1_BUTNUM_SYNTH_B = 12;
const uint8_t SR1_BUTNUM_SYNTH_C = 13;
const uint8_t SR1_BUTNUM_SYNTH_D = 14;

const uint8_t SR1_BUTNUM_HIGH_INIT = 15;
const uint8_t SR1_BUTNUM_HELP = 16;
const uint8_t SR1_BUTNUM_POV = 17;
const uint8_t SR1_BUTNUM_VAL = 18;

const uint8_t SR1_BUTNUM_USER_SW = 19;
const uint8_t SR1_BUTNUM_USER_NW = 20;
const uint8_t SR1_BUTNUM_USER_NE = 21;
const uint8_t SR1_BUTNUM_USER_SE = 22;

const uint8_t SR1_BUTNUM_PATH_MOD = 23;
const uint8_t SR1_BUTNUM_GLOBAL_MOD = 24;

// The WTS engine should keep state of this!!!  Just a hack until we sync up with the Engine!
      uint8_t globalModule = 0;
      const uint8_t SR1_BUTNUM_GLOBAL_MOD_SETTINGS = 0;
      const uint8_t SR1_BUTNUM_GLOBAL_MOD_PATCHLIB = 1;
      const uint8_t SR1_BUTNUM_GLOBAL_MOD_MIXER = 2;

const uint8_t SR1_BUTNUM_VOICE_MOD = 25;
const uint8_t SR1_BUTNUM_FILTER_MOD = 26;
const uint8_t SR1_BUTNUM_TERRAIN_MOD = 27;

const uint8_t SR1_BUTNUM_ENTER = 28;

const uint8_t SR1_BUTNUM_GRID = 29;
const uint8_t SR1_BUTNUM_SHIFT = 30;
const uint8_t SR1_BUTNUM_LOAD = 31;
const uint8_t SR1_BUTNUM_SAVE = 32;

const uint8_t SR1_BUTNUM_EXP = 33;

const uint8_t SR1_BUTNUM_REC = 34;
const uint8_t SR1_BUTNUM_ARP = 35;
const uint8_t SR1_BUTNUM_TIE = 36;
const uint8_t SR1_BUTNUM_REST = 37;
const uint8_t SR1_BUTNUM_PLAY_PAUSE = 38;
const uint8_t SR1_BUTNUM_STOP = 39;

ControllerButtonEnums SR1_ControllerButtonToSynthEnumLUT[40] = {
  ControllerButtonEnums::LearnButtonRelease,  // Specificly RELEASE

  ControllerButtonEnums::LFOButton,
  ControllerButtonEnums::ENVButton,
  ControllerButtonEnums::SeqButton,

  ControllerButtonEnums::ModulatorButton0,
  ControllerButtonEnums::ModulatorButton1,
  ControllerButtonEnums::ModulatorButton2,
  ControllerButtonEnums::ModulatorButton3,

  ControllerButtonEnums::MenuUpButton,
  ControllerButtonEnums::MenuDownButton,

  ControllerButtonEnums::NotUsed,         // WTS Button
  ControllerButtonEnums::SynthAButton,
  ControllerButtonEnums::SynthBButton,
  ControllerButtonEnums::SynthCButton,
  ControllerButtonEnums::SynthDButton,

  ControllerButtonEnums::InitButtonRelease,  // Specificly RELEASE
  ControllerButtonEnums::HelpButtonRelease,  // Specificly RELEASE
  ControllerButtonEnums::POVButton,
  ControllerButtonEnums::VALButton,

  ControllerButtonEnums::UserButton4,
  ControllerButtonEnums::UserButton5,
  ControllerButtonEnums::UserButton1,
  ControllerButtonEnums::UserButton2,

  ControllerButtonEnums::PathSelectButton,
  ControllerButtonEnums::NotUsed,             // Global Select Button

  ControllerButtonEnums::VoiceSelectButton,
  ControllerButtonEnums::FilterSelectButton,
  ControllerButtonEnums::TerrainSelectButton,

  ControllerButtonEnums::JogEnterButton,

  ControllerButtonEnums::GridButton,
  ControllerButtonEnums::ShiftButtonRelease,  // Specificly RELEASE
  ControllerButtonEnums::LoadButtonRelease,   // Specificly RELEASE
  ControllerButtonEnums::SaveButtonRelease,   // Specificly RELEASE

  ControllerButtonEnums::EXPButton,

  ControllerButtonEnums::ArpButton,
  ControllerButtonEnums::RestButton,
  ControllerButtonEnums::TieButton,
  ControllerButtonEnums::RecButton,
  ControllerButtonEnums::StopButton,
  ControllerButtonEnums::PlayPauseButton

};

//==============================================================================

// v3 encoder defs
// Shift Reg Bus 0 (SR0)
const uint8_t SR0_ENCNUM_MAIN_VOLUME = 0;

const uint8_t SR0_ENCNUM_MIXER_A = 1;
const uint8_t SR0_ENCNUM_MIXER_B = 2;
const uint8_t SR0_ENCNUM_MIXER_C = 3;
const uint8_t SR0_ENCNUM_MIXER_D = 4;

const uint8_t SR0_ENCNUM_MENU0 = 5;
const uint8_t SR0_ENCNUM_MENU1 = 6;
const uint8_t SR0_ENCNUM_MENU2 = 7;
const uint8_t SR0_ENCNUM_MENU3 = 8;
const uint8_t SR0_ENCNUM_MENU4 = 9;
const uint8_t SR0_ENCNUM_MENU5 = 10;


ControllerEncoderEnums SR0_ControllerEncToSynthEnumLUT[11] = {
  ControllerEncoderEnums::MainVolumeEncoder,

  ControllerEncoderEnums::MixerEncoder0,
  ControllerEncoderEnums::MixerEncoder1,
  ControllerEncoderEnums::MixerEncoder2,
  ControllerEncoderEnums::MixerEncoder3,

  ControllerEncoderEnums::MenuEncoder0,
  ControllerEncoderEnums::MenuEncoder1,
  ControllerEncoderEnums::MenuEncoder2,
  ControllerEncoderEnums::MenuEncoder3,
  ControllerEncoderEnums::MenuEncoder4,
  ControllerEncoderEnums::MenuEncoder5
};

//==============================================================================

// Shift Reg Bus 0 (SR0)
const uint8_t SR1_ENCNUM_MODULATION_ENC0 = 0;
const uint8_t SR1_ENCNUM_MODULATION_ENC1 = 1;
const uint8_t SR1_ENCNUM_MODULATION_ENC2 = 2;
const uint8_t SR1_ENCNUM_MODULATION_ENC3 = 3;

const uint8_t SR1_ENCNUM_JOG_ENC = 4;

const uint8_t SR1_ENCNUM_USER_ENC0 = 5;
const uint8_t SR1_ENCNUM_USER_ENC1 = 6;


ControllerEncoderEnums SR1_ControllerEncToSynthEnumLUT[7] = {
  ControllerEncoderEnums::ModulatorEncoder0,
  ControllerEncoderEnums::ModulatorEncoder1,
  ControllerEncoderEnums::ModulatorEncoder2,
  ControllerEncoderEnums::ModulatorEncoder3,

  ControllerEncoderEnums::JogWheelEncoder,

  ControllerEncoderEnums::UserEncoder0,
  ControllerEncoderEnums::UserEncoder1,
};

//==============================================================================

const uint8_t  LED_LERP = 0;
const uint8_t  LED_BLINK = 1;

const uint16_t allButDimONTime = 150;
const uint16_t allButDimOFFTime = allButDimONTime * 2;
const uint16_t buttonDimONTime = allButDimONTime;
const uint16_t buttonDimOFFTime = allButDimOFFTime;
const uint16_t synthSelDimONTime = allButDimONTime;
const uint16_t synthSelDimOFFTime = allButDimOFFTime;
const uint16_t moduleSelDimONTime = allButDimONTime;
const uint16_t moduleSelDimOFFTime = allButDimOFFTime;
const uint16_t gridButtonsDimONTime = 100;
const uint16_t gridButtonsDimOFFTime = 150;
const uint16_t P_B_KB_ARPButtonsDimONTime = allButDimONTime;
const uint16_t P_B_KB_ARPButtonsDimOFFTime = allButDimOFFTime;
const uint16_t shiftButDimONTime = 100;
const uint16_t shiftButDimOFFTime = 150;


bool gMidiHandShakeComplete = false;
bool lastModuleWasGlobalMod = false;
bool gShiftHeld = false;
bool gHelpHeld = false;

bool gUserNwHeld = false;
bool gUserNeHeld = false;
bool gUserSeHeld = false;
bool gUserSwHeld = false;

bool gVoiceHeld = false;
bool gFilterHeld = false;
uint8_t gVolMacro = 0; // Vol=0 Mac=1

uint8_t gUserEncMode = 0;
const uint8_t  maxNumUserEncModes = 4;
uint8_t gEncLFOParam = 2;
uint8_t gCurrentPatch = 0;
uint8_t gCurrentBank = 0;
uint8_t gShiftEncVel = 1;

bool gEncoderSelectMode = false;

const uint8_t cc_ENCODER_SELECT = 121;
const uint8_t CCButtonPress = static_cast<uint8_t>(ControllerMidiEnums::Button);

// used for debugging... stepping thru the LEDs
//bool nextLed = false;
//int ledIndex = 0;

// function prototypes
void PICO_ADC_scanAll();

void HandleCC(byte channel, byte number, byte value);
int whichLEDGroup(unsigned int ledNum);
void HandleSysEx(byte *data, unsigned int length);
void MIDI_USB_DEV_Get();
void MIDI_5_PIN_Get();
void MIDIPanic();

void ButPressed(uint8_t srPortNum, butInfoType butInfo);
void ButHeld(uint8_t srPortNum, butInfoType butInfo);
void ButRelease(uint8_t srPortNum, butInfoType butInfo);
void EncChanged(uint8_t srPortNum, encInfoType encInfo);

void handleInBoundModuleChangeLED(uint8_t CC10_ModSelect);
void handleInBoundSynthChangeLED(uint8_t CC10_ModSelect);
void BootUpLEDLightShow();

GRBPixelType LEDLERP(GRBPixelType theRGBStart, GRBPixelType theRGBEnd, float lerpPos);
void addAniLED(uint8_t ledNum, GRBPixelType theRGBEnd, uint16_t duration = 200, uint8_t theAniType = 0);
void updateAniLEDs();
void cancelAniLEDs();

void tempLightInitLEDs(boolean turnOn);
void tempLightLoadLEDs(boolean turnOn);
void tempLightSaveLEDs(boolean turnOn);

void FillColSolidLEDarray(GRBPixelType theCol);
void FillColWashLEDarray(GRBPixelType theLEDsarray[], uint8_t sizeofarray);
GRBPixelType GetGradVal(uint8_t theGradPosition, GRBPixelType theMinCol, GRBPixelType theMidCol, GRBPixelType theMaxCol);
float fmap(float x, float a, float b, float c, float d);
void SendControllerInfo();

//void Send_MIDI_Raw(uint8_t midi_msg_type, uint8_t data1, uint8_t data2, uint8_t channel);
void Send_MIDI_To_Port(MIDI_Destinations_Enums thePort, uint8_t midi_msg_type, uint8_t data1, uint8_t data2, uint8_t channel);

struct ControllerInfoType {
    uint8_t HW_MajRev = 0;
    uint8_t HW_MidRev = 0;
    uint8_t HW_MinRev = 0;
    uint8_t SW_MajRev = 0;
    uint8_t SW_MidRev = 0;
    uint8_t SW_MinRev = 0;
};

ControllerInfoType ControllerInfo = {};

//==============================================================================================

void setup() {

  ControllerInfo.HW_MajRev = 3;
  ControllerInfo.HW_MidRev = 6;
  ControllerInfo.HW_MinRev = 21;

  ControllerInfo.SW_MajRev = 0;
  ControllerInfo.SW_MidRev = 0;
  ControllerInfo.SW_MinRev = 13;

  sr1Port.setPins(SRX_LATCH_GPIO, SRX_CLK_GPIO, SR1_DATA_IN_GPIO);
  sr1Port.setHoldTime(sr1_holdTime);
  sr1Port.setHandle_ButPressed(ButPressed);
  sr1Port.setHandle_ButHeld(ButHeld);
  sr1Port.setHandle_ButReleased(ButRelease);
  sr1Port.setHandle_EncChanged(EncChanged);

  sr2Port.setPins(SRX_LATCH_GPIO, SRX_CLK_GPIO, SR2_DATA_IN_GPIO);
  sr2Port.setHoldTime(sr2_holdTime);
  sr2Port.setHandle_ButPressed(ButPressed);
  sr2Port.setHandle_ButHeld(ButHeld);
  sr2Port.setHandle_ButReleased(ButRelease);
  sr2Port.setHandle_EncChanged(EncChanged);

  gpio_init(HW_PROGRAM_BUTTON);
  gpio_set_function(HW_PROGRAM_BUTTON, GPIO_FUNC_SIO);
  gpio_set_dir(HW_PROGRAM_BUTTON, GPIO_IN);
  pinMode(HW_PROGRAM_BUTTON, INPUT_PULLUP);

} // end setup

//-----------------------------------------------

void setup1() {
  Serial0_USB.begin(115'200); // USB serial init w/defaults

  // make sure to set '#define CFG_TUD_CDC 2' and '#define CFG_TUD_MIDI 2' in file tusb_config_rp2040.h in dir...
  /* C:\.platformio\packages\framework-arduinopico\libraries\Adafruit_TinyUSB_Arduino\src\arduino\ports\rp2040\ */
  //Serial1_USB.begin(1'000'000);

  // Initialize UART1 (PICO_TO_CM) on GPIO8 (TX) and GPIO9 (RX)
  uart_init(uart1, 38'400); // tested at 2'000'000 and works
  gpio_set_function(8, GPIO_FUNC_UART);  // Set GPIO8 to UART function
  gpio_set_function(9, GPIO_FUNC_UART);  // Set GPIO9 to UART function

  MIDI_5_PIN_UART0.begin(MIDI_CHANNEL_OMNI);
  MIDI_5_PIN_UART0.turnThruOff();

  MIDI_PICO_TO_CM_UART1.begin(MIDI_CHANNEL_OMNI);
  MIDI_PICO_TO_CM_UART1.turnThruOff();

  TinyUSB_SerialMIDI.setStringDescriptor("WTS Serial MIDI");
  TinyUSB_SerialMIDI.begin(115'200);
  MIDI_USB_SERIAL_DEV.begin(MIDI_CHANNEL_OMNI);
  MIDI_USB_SERIAL_DEV.turnThruOff();

  //TinyUSB_MIDI.setStringDescriptor("WTS Controller");
  TinyUSB_MIDI.setStringDescriptor("Wave Terrain Synth");
  MIDI_USB_DEV.begin(MIDI_CHANNEL_OMNI);
  MIDI_USB_DEV.turnThruOff();
  while( !TinyUSBDevice.mounted() ) { delay(1); }  // wait until device mounted

  pinMode(NEOPIX_GPIO, OUTPUT);

  // this is a pin that was acedentailly used in the hardware which is now jumpered to MISO
  gpio_init(BAD_MISO);
  gpio_set_dir(BAD_MISO, GPIO_IN);

// ADC AD6886 SETUP...
  // SETUP: GPIO pins & SPI pins for ADC AD6886
  //Initialise GPIO pins for AD6886 ADC reset pin
  gpio_init(CV_RST); // Initialise CS Pin
  gpio_set_dir(CV_RST, GPIO_OUT); // Set CS as output

  //Initialise GPIO pins for SPI communication
  gpio_init(MISO);
  gpio_set_dir(MISO, GPIO_IN);
  gpio_set_function(MISO, GPIO_FUNC_SPI);

  gpio_init(SCLK);
  gpio_set_dir(SCLK, GPIO_OUT);
  gpio_set_function(SCLK, GPIO_FUNC_SPI);

  gpio_init(MOSI);
  gpio_set_dir(MOSI, GPIO_OUT);
  gpio_set_function(MOSI, GPIO_FUNC_SPI);

  // Configure Chip Select Pins
  gpio_init(CS_CV); // Initialise CS Pin
  gpio_set_dir(CS_CV, GPIO_OUT); // Set CS as output
  gpio_put(CS_CV, 1); // Set CS High to indicate no currect SPI communication

  gpio_init(CS_CG); // Initialise CS Pin
  gpio_set_dir(CS_CG, GPIO_OUT); // Set CS as output
  gpio_put(CS_CG, 1); // Set CS High to indicate no currect SPI communication

// DAC SETUP...
  gpio_init(DAC_MUTE_GPIO); // Initialise DAC MUTE Pin
  gpio_set_dir(DAC_MUTE_GPIO, GPIO_OUT); // Set DAC MUTE pin as output
  gpio_put(DAC_MUTE_GPIO, 0); // Set DAC MUTE to mute

  gpio_init(DAC_RESET_GPIO); // Initialise DAC MUTE Pin
  gpio_set_dir(DAC_RESET_GPIO, GPIO_OUT); // Set DAC MUTE pin as output
  gpio_put(DAC_RESET_GPIO, 0); // Set DAC MUTE to mute

  delay(10);
  gpio_put(DAC_RESET_GPIO, 1); // Set DAC MUTE to mute
  delay(10);
  gpio_put(DAC_MUTE_GPIO, 1); // Set DAC MUTE to mute

// HeadPhone Amp via I2C Setup
    i2c_init(i2c1, 400000);                  // Initialize I2C at 400 kHz
    gpio_set_function(I2C1_SDA, GPIO_FUNC_I2C);       // Set SDA pin (e.g., GP20)
    gpio_set_function(I2C1_SCL, GPIO_FUNC_I2C);       // Set SCL pin (e.g., GP21)
    gpio_pull_up(I2C1_SDA);                           // Pull-up resistor for SDA
    gpio_pull_up(I2C1_SCL);                           // Pull-up resistor for SCL

// write to Headphone Amp via i2c
    uint8_t slave_address = 0b110'0000;
    uint8_t numBytes = 3;
    uint8_t buffer[numBytes];
    buffer[0] = 0b0000'0001;   // Register address 1
    buffer[1] = 0b11'000000;   // HP_EN0&1(2bit) + MISC(6bit)
    buffer[2] = 0b00'110101;   // Mute(2bit) + Vol(6bit)
    int rtn = i2c_write_blocking(i2c1, slave_address, buffer, numBytes, false);

// // write to DAC via i2C - tested inbound I2S to DAC with Teensy WTS and it receives I2S samples & plays sound!
//     uint8_t slave_address = 0b1001110;
//     uint8_t numBytes = 1;
//     uint8_t buffer[numBytes];
//     buffer[0] = x;             // Register address
//     buffer[1] = 0bxxxx'xxxx;   // ????
//     i2c_write_blocking(i2c1, slave_address, buffer, numBytes, false);


  //Init the onboard PICO: DMA ADC MUX inputs - not used in this design
  // adc_4051_dma_init();
  // delay (100);


  // Clear LED data arrays
  memset(theLEDs, 0, sizeof(theLEDs));
  delay(1);
  memset(theTempLEDs, 0, sizeof(theTempLEDs));
  delay(1);
  memset(temporalLEDs, 0, sizeof(temporalLEDs));
  delay(1);

  //Init the DMA PIO LED refresh
  dma_ws2812_init(theLEDs, NUM_NEOPIX);
  delay (100);

  // Bootup Color Show
  BootUpLEDLightShow();
  delay(100);

  // blacken all LEDs
  memset(theLEDs, 0, NUM_NEOPIX * sizeof(theLEDs[0]));
  delay(1);
  dma_ws2812_leds_update();
  delay(250);

  // turn on the Power LED
  addAniLED(LED_POWER, theColWhite, 500, LED_LERP);

  pinMode(PICO_DEFAULT_LED_GPIO, OUTPUT);
  // BLINK Pico LED
  for (auto k=0; k<3; ++k) {
    digitalWrite(PICO_DEFAULT_LED_GPIO, HIGH);
    delay(300);
    digitalWrite(PICO_DEFAULT_LED_GPIO, LOW);
    delay(150);
  }

  //sr1Port.DebugDump(); // dumps what happened in the constructor

  // SET UP AD6886 via SPI...
    // GPIO 7 used for Logic Analyser debugging
    gpio_init(LOGIC_ANALYZER_GPIO);
    gpio_set_dir(LOGIC_ANALYZER_GPIO, GPIO_OUT);
    gpio_put(LOGIC_ANALYZER_GPIO, 0);

    // SPI SETUP
    spi_init(SPI_PORT, SPI_CLK_RATE); // Initialise spi0 at 10MHz; max clk = 17MhZ
    spi_set_slave(SPI_PORT, false); // set to master
    spi_set_format(SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST); // Set to Mode 1 (not default)

    gpio_put(CV_RST, 1); // Set CV_RST High to release chip reset
    delayMicroseconds(50);
    AD8668_SetCommandReg16(AD6886_RST);
    delayMicroseconds(50);

    // Set default AD8668 ADC input voltage ranges to 5.12v
    //Serial0_USB.println("SetVoltageInputRanges...");
    AD8668_SetVoltageInputRange(AD6886_ALL_INPUTS, UNI_5v12_RANGE);
    //AD8668_SetVoltageInputRange(0, UNI_10v24_RANGE);

// !!! these need to come back from where the PICO TO CM5 got to !!!
  // setup MIDI Call backs
  String PICO_TO_CM_Port_Name = "NOT ASSIGNED";

  if (PICO_TO_CM == MIDI_Destinations_Enums::UART1_Pico_To_CM_MIDI) {
    MIDI_PICO_TO_CM_UART1.setHandleControlChange(HandleCC);
    MIDI_PICO_TO_CM_UART1.setHandleSystemExclusive(HandleSysEx);
    PICO_TO_CM_Port_Name = "MIDI_PICO_TO_CM_UART1";
  }
  else if (PICO_TO_CM == MIDI_Destinations_Enums::USB_MIDI) {
    MIDI_USB_DEV.setHandleControlChange(HandleCC);
    MIDI_USB_DEV.setHandleSystemExclusive(HandleSysEx);
    PICO_TO_CM_Port_Name = "MIDI_USB_DEV";
  }
  else if (PICO_TO_CM == MIDI_Destinations_Enums::USB_Serial1) {
    MIDI_USB_SERIAL_DEV.setHandleControlChange(HandleCC);
    MIDI_USB_SERIAL_DEV.setHandleSystemExclusive(HandleSysEx);
    PICO_TO_CM_Port_Name = "MIDI_USB_SERIAL_DEV";
  }
  else if (PICO_TO_CM == MIDI_Destinations_Enums::UART0_5Pin_MIDI) {
    MIDI_5_PIN_UART0.setHandleControlChange(HandleCC);
    MIDI_5_PIN_UART0.setHandleSystemExclusive(HandleSysEx);
    PICO_TO_CM_Port_Name = "MIDI_5_PIN_UART0";
  }

  Serial0_USB.println("WTS Cntlr Starting ... PICO_TO_CM port name = " + PICO_TO_CM_Port_Name);
  //Serial1_USB.println("Serial1_USB Starting ...");

}  // end setup1

//-----------------------------------------------

void loop() {
  static uint32_t prevTime1 = 0;
  static uint32_t prevTime2 = 0;

  // Read all the inbound shift reg data for buttons and encoders
  if ( (micros() - prevTime1) > 800) { // 800us period. FYI the sr1&2 scans takes ~600us
    prevTime1 = micros();
    sr1Port.scanModules();
    sr2Port.scanModules();

    if (!gpio_get(HW_PROGRAM_BUTTON)) {
      //Serial0_USB.println("going into Mass Storage boot mode...");
      //delay (500);
      reset_usb_boot(0,0);
    }
  }

  // if ( (millis() - prevTime2) > 10000) {
  //     prevTime2 = millis();
  //     Serial0_USB.println("...Serial0_USB (debug) still alive!");
  //     TinyUSB_SerialMIDI.println("...TinyUSB_SerialMIDI still alive!");
  //     //Serial1_USB.println("...serial 1 still alive!");
  // }

} // end loop

//-----------------------------------------------

void loop1() {
  static uint32_t prevTime1 = 0;
  static uint32_t prevTime2 = 0;
  static uint32_t prevTime3 = 0;
  static uint32_t prevTime4 = 0;

  // Read USB Serial Port for MIDI data
  //if ( (millis() - prevTime4) > 1) { // 1ms
  if ( (micros() - prevTime4) > 800) { // 1000us 1ms
    //prevTime4 = millis();
    prevTime4 = micros();
    MIDI_USB_DEV.read();
    //MIDI_USB_Get();  // only using call-backs for MIDI_USB

    // FOR TESTING ONLY - Pass 5-Pin MIDI Msgs to WTS via MIDI_USB; filtering CH16 Msgs (reserved for HW Controller MIDI Msgs)
    // PICO will pass 5-Pin & USB_Dev MIDI Msgs to WTS via MIDI_PICO_TO_CM_UART1 on UART1 @ 1'000'000 baud
    MIDI_5_PIN_Get();

    spi_set_format(SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST); // Set to Mode 1 (not default)
    AD8668_GetAllVoltages(AD8668Voltages);

    // Convert and Send all CVs that changed to WTS Engine
    uint8_t bitTrunc = 0;
    for (auto i=0; i<8; ++i) { // read CVs 0-5 + Exp Ped 1 & 2
// TEMPORARY: ignor CVs 4 & 5 cause they aren't on the 1/2 I/O board
      if (!(i==4 || i==5)) {

        const uint8_t bitSmoother = 1;
        const uint8_t bitReduce = 2;
        cvSumVoltages[i] -= cvSumVoltages[i] >> bitSmoother;
        cvSumVoltages[i] += AD8668Voltages[i];
        uint16_t aveVoltage = cvSumVoltages[i] >> (bitSmoother + bitReduce);

        if (aveVoltage != cvPrevAveVoltages[i]) {
          if (aveVoltage != cvPrev2AveVoltages[i]) {
            uint8_t theMSB = (aveVoltage >> 7) & 0x7F;
            uint8_t theLSB = aveVoltage & 0x7F;

// !!! depricated code - test below code for Exp Ped and CV before deleting !!!

            // !!! TEMPORARY !!!  should send to CM5 via MIDI_PICO_TO_CM_UART1 (UART1)
            //MIDI_USB_DEV.sendControlChange((midi::DataByte)cvToMidiLUT[i*2], theMSB, theMIDIChan);
            //MIDI_USB_DEV.sendControlChange((midi::DataByte)cvToMidiLUT[i*2+1], theLSB, theMIDIChan);

            // // Send out UART1 (to go to the CM5) - this should be the "keeper" code
            // if (prevMidiMsgSrcPort != wtsSourcePort::SRC_MIDI_CV_GATE ) {
            //   Send_MIDI_To_Port(Serial1_USB_Raw, (uint8_t)ControllerMidiEnums::WTS_SRC_PORT, (uint8_t)wtsSourcePort::SRC_MIDI_CV_GATE, 16);
            //   delayMicroseconds(50);
            // }
            // send_USB_serial_raw_midi_cc(Serial0_USB, cvToMidiLUT[i*2], theMSB, theMIDIChan);
            // delayMicroseconds(50);
            // send_USB_serial_raw_midi_cc(Serial0_USB, cvToMidiLUT[i*2+1], theLSB, theMIDIChan);
            // delayMicroseconds(50);
            // prevMidiMsgSrcPort = wtsSourcePort::SRC_MIDI_CV_GATE;


            // Send out to the Synth Engine (Product - on the CM5) / (Dev - on the PC/MAC) - this should be the "keeper" code
            if (prevMidiMsgSrcPort != wtsSourcePort::SRC_MIDI_CV_GATE ) {
              Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, (uint8_t)ControllerMidiEnums::WTS_SRC_PORT, (uint8_t)wtsSourcePort::SRC_MIDI_CV_GATE, 16);
              delayMicroseconds(50);
            }
            Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, cvToMidiLUT[i*2], theMSB, theMIDIChan);
            delayMicroseconds(50);
            Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, cvToMidiLUT[i*2+1], theLSB, theMIDIChan);
            delayMicroseconds(50);
            prevMidiMsgSrcPort = wtsSourcePort::SRC_MIDI_CV_GATE;

            // for debugging: calc 7 bits only...
            //uint8_t only7Bits = aveVoltage >> (5 - bitReduce);
            //uint16_t recombined = (uint16_t)theMSB << 7 | theLSB;
            //Serial0_USB.println(String("CV") + i + " -> CC#" + cvToMidiLUT[i*2] + "&" + cvToMidiLUT[i*2+1]+ " val=" + aveVoltage + "  (MSB=" + theMSB + " LSB=" + theLSB + ")=" + recombined + "  7bits=" + only7Bits);

            cvPrev2AveVoltages[i] = cvPrevAveVoltages[i];
            cvPrevAveVoltages[i] = aveVoltage;
          }
        }
      }
    } // end for

    //Serial0_USB.printf("%5hu %5hu %5hu\n", AD8668Voltages[0], AD8668Voltages[6], AD8668Voltages[7]);
    // Serial0_USB.printf("%5hu %5hu %5hu %5hu %5hu %5hu %5hu %5hu\n",
    //        AD8668Voltages[0], AD8668Voltages[1], AD8668Voltages[2], AD8668Voltages[3],
    //        AD8668Voltages[4], AD8668Voltages[5], AD8668Voltages[6], AD8668Voltages[7]);


    // Convert and Send all Gates that changed to WTS Engine
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST); // Set to Mode 1 (not default)
    uint8_t PCA9702Gates = PCA9702_GatAllGates();
    for (auto i=0; i<7; ++i) {
        uint8_t theIthGate = (PCA9702Gates >> i) & 1;
        uint8_t theIthPrevGate = (PCA9702PrevGates >> i) & 1;
        if (theIthGate != theIthPrevGate) {

// !!! depricated code - test below code for Gates before deleting !!!

          // !!! TEMPORARY !!!  should send to CM5 via MIDI_PICO_TO_CM_UART1 (UART1)
          // MIDI_USB_DEV.sendControlChange((midi::DataByte)gateToMidiLUT[i], theIthGate * 127, theMIDIChan);

          // if (prevMidiMsgSrcPort != wtsSourcePort::SRC_MIDI_CV_GATE ) {
          //     //MIDI_PICO_TO_CM_UART1.sendControlChange((midi::DataByte)ControllerMidiEnums::WTS_SRC_PORT, (midi::DataByte)wtsSourcePort::SRC_MIDI_CV, (midi::Channel)16);
          //     send_USB_serial_raw_midi_cc(Serial0_USB, (uint8_t)ControllerMidiEnums::WTS_SRC_PORT, (uint8_t)wtsSourcePort::SRC_MIDI_CV_GATE, 15);
          //     delayMicroseconds(50);
          // }
          // //MIDI_PICO_TO_CM_UART1.sendControlChange((midi::DataByte)gateToMidiLUT[i], theIthGate * 127, theMIDIChan);
          // send_USB_serial_raw_midi_cc(Serial0_USB, gateToMidiLUT[i], theIthGate * 127, theMIDIChan);
          // delayMicroseconds(50);

          // Send out to the Synth Engine (Product - on the CM5) / (Dev - on the PC/MAC) - this should be the "keeper" code
          if (prevMidiMsgSrcPort != wtsSourcePort::SRC_MIDI_CV_GATE ) {
            Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, (uint8_t)ControllerMidiEnums::WTS_SRC_PORT, (uint8_t)wtsSourcePort::SRC_MIDI_CV_GATE, 16);
            delayMicroseconds(50);
          }
          Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, gateToMidiLUT[i], theIthGate * 127, theMIDIChan);
          delayMicroseconds(50);
          prevMidiMsgSrcPort = wtsSourcePort::SRC_MIDI_CV_GATE;
          PCA9702PrevGates = PCA9702Gates;
        }
    }

    auto i = 7; // Analog_Clock_Tic
    uint8_t theIthGate = (PCA9702Gates >> i) & 1;
    uint8_t theIthPrevGate = (PCA9702PrevGates >> i) & 1;
    if (theIthGate != theIthPrevGate) {
      if (theIthGate == 0) {  // 1 PPQ
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, (uint8_t)ControllerGateEnums::Analog_Clock_Tic, 0, theMIDIChan);
        delayMicroseconds(50);
        // Serial0_USB.println("Analog Clock Tic ");
        prevMidiMsgSrcPort = wtsSourcePort::SRC_MIDI_CV_GATE;
        PCA9702PrevGates = PCA9702Gates;
      }
    }
  }

  // Animate LEDs
  if ( (millis() - prevTime1) > 67) { // ~15fps (67ms)
    prevTime1 = millis();
    updateAniLEDs();

    // for debugging, steps thru the LEDs
    // if (nextLed) {
    //   nextLed = false;
    //   addAniLED(ledIndex, {0,10,10}, 500, LED_LERP);
    //   Serial0_USB.println(String("LED #") + ledIndex);
    //   ledIndex++;
    //   if (ledIndex > NUM_NEOPIX -1 ) {
    //       ledIndex = 0;
    //       memset(theLEDs, 0, NUM_NEOPIX * sizeof(theLEDs[0]));
    //       delay(1);
    //       dma_ws2812_leds_update();
    //       delay(250);
    //   }
    // }

  } // end Animate LEDs

  // HW Controller to WTS Synth Engine Handshake
  static bool blink_No_Handshake = false;
  static uint8_t keepAlive = 0;
  if ( (millis() - prevTime2) > 333) { // ~3/s
    prevTime2 = millis();
    if (!gMidiHandShakeComplete) {
      //Serial0_USB.println("...Sending CONTROLLER_INIT_REQUEST");
      Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, (uint8_t)ControllerMidiEnums::InitRequest, 0, theMIDIChan);
      if (blink_No_Handshake) {
        addAniLED(LED_POWER, {0, 16, 0}, 150, LED_LERP); /*GRB*/
        blink_No_Handshake = false;
        }
      else {
        addAniLED(LED_POWER, {0, 4, 0}, 150, LED_LERP); /*GRB*/
        blink_No_Handshake = true;
      }
    }
    else {
      //Serial0_USB.println("...in Proc1 loop - also alive!");

      keepAlive++;
      if (keepAlive > 5) {
        addAniLED(LED_POWER, {14, 14, 16}, 250, LED_LERP); /*GRB*/
        keepAlive = 0;
        }
      else {
        addAniLED(LED_POWER, {4, 4, 4}, 500, LED_LERP); /*GRB*/
      }
    }
  } // end Handshake

  // LED updates and ADC scanning
  if ( (millis() - prevTime3) > 10) { // 100/s (10ms)
    prevTime3 = millis();
    dma_ws2812_leds_update();
    //PICO_ADC_scanAll();
  }

} // end loop1

//-----------------------------------------------

// PICO on-board ADC, not used for this WTS project...
// void PICO_ADC_scanAll() {
//     static uint16_t prevAdcVals[8];
//     uint16_t adc7bitInput = 0;
//     static const uint8_t adcIndexToSynthIndex[8] = {
//       static_cast<uint8_t>(ParameterIndex::Index::ControlVoltage_1),
//       static_cast<uint8_t>(ParameterIndex::Index::ControlVoltage_2),
//       static_cast<uint8_t>(MidiControllerIndex::Expression_Pedal),
//       static_cast<uint8_t>(MidiControllerIndex::Foot_Pedal),
//       static_cast<uint8_t>(ParameterIndex::Index::ControlVoltage_1),
//       static_cast<uint8_t>(ParameterIndex::Index::ControlVoltage_2),
//       static_cast<uint8_t>(MidiControllerIndex::Expression_Pedal),
//       static_cast<uint8_t>(MidiControllerIndex::Foot_Pedal),
//     };

//     adc_4051_dma_get_all(adcInputs); // get all 8 ADC Mux'd inputs
//     // uint16_t adc0In = adc_4051_dma_get(0); // get 1 of 8 ADC Mux'd inputs

//     //for (auto i=0; i<8; ++i) {
//     for (auto i=0; i<1; ++i) { // only send ControlVoltage1
//       adc7bitInput = 0x0000'007F & (adcInputs[i] >> 9); // 7 bits [0-127] is stable
//       //adc7bitInput = 0x0000'00FF & (adcInputs[i] >> 8); // 8 bits [0-255] is stable
//       //adc7bitInput = 0x0000'01FF & (adcInputs[i] >> 7); // 9 bits [0-511] is stable
//       //adc7bitInput = 0x0000'03FF & (adcInputs[i] >> 6); // 10 bits [0-1023] is stable
//       //adc7bitInput = 0x0000'07FF & (adcInputs[i] >> 5); // 11 bits [0-2047] is NOT to stable

//       if (prevAdcVals[i] != adc7bitInput) {
//         //Serial0_USB.println(String("7bit ") + i + " = " + adc7bitInput + " =? " + prevAdcVals[i]);
//         prevAdcVals[i] = adc7bitInput;

//         // single CC for 7bit CC msg
//         MIDI_USB_DEV.sendControlChange((midi::DataByte)adcIndexToSynthIndex[i], static_cast<uint8_t>(prevAdcVals[i]), theMIDIChan);
//         MIDI_PICO_TO_CM_UART1.sendControlChange((midi::DataByte)adcIndexToSynthIndex[i], static_cast<uint8_t>(prevAdcVals[i]), theMIDIChan);

//         // MSB then LSB for 8-10bit dual CC msgs
//         //MIDI_UART2.sendControlChange((midi::DataByte)adcIdxToCC[i] + 1, static_cast<uint8_t>(0x0000'007F & (prevAdcVals[i] >> 7)), theMIDIChan);
//         //MIDI_UART2.sendControlChange((midi::DataByte)adcIdxToCC[i], static_cast<uint8_t>(0x0000'007F & prevAdcVals[i]), theMIDIChan);
//       } // end if
//     } // end for

// } // end PICO_ADC_scanAll()

//-----------------------------------------------

// HandleCC() is assigned as the call-back in Setup1 where PICO_TO_CM is defined:
//      - product release should be assigned to MIDI_PICO_TO_CM_UART1
//      - debug release should be assigned to MIDI_USB_DEV
void HandleCC(uint8_t channel, uint8_t theCC, uint8_t theCCVal) {
  //LOGLINE_MIDI_DEBUG(String("In CC Handler - ch=") + channel + " cc=" + theCC + " val=" + theCCVal);


    if (theCC == (uint8_t)ControllerMidiEnums::InitAcknowledge) {  // the CONTROLLER_INIT_REQUEST has beed received
        gMidiHandShakeComplete = true;
        addAniLED(LED_POWER, {0, 0, 0}, 150, LED_LERP); /*GRB*/
        SendControllerInfo();
    }

  //LOGLINE_MIDI_DEBUG("   ... out CC Handler");
} // end fcn HandleCC

// =======================================================

int whichLEDGroup(uint8_t hwLedNum) {
  for (auto j=0; j<NUM_LED_GROUPS; ++j) {
    for (auto i=0; i<MAX_NUM_LEDS_PER_GROUP; ++i) {
      if (hwLedNum == static_cast<uint8_t>(ledGroupInfo[j][i])) {
        return j;
        break;
      }
    }
  }
  return 0;
}

// =======================================================

// HandleSysEx() is assigned as the call-back in Setup1 where PICO_TO_CM is defined:
//      - product release should be assigned to MIDI_PICO_TO_CM_UART1
//      - debug release should be assigned to MIDI_USB_DEV
void HandleSysEx(byte *data, unsigned int length) {
  LOGLINE_MIDI_DEBUG(String("SysEx Message... ") + data[0] + " " + data[1] + " " + data[2] + " "+ data[3] + " "+ data[4] + " "+ data[5] + " "+ data[6] + " "+ data[7]);
  //Serial0_USB.println(String("SysEx Message type #") + data[1] + " len:" + length + " #of Pairs=" + data[2]);

  switch (static_cast<SysexMessageType>(data[1])) {  // byte 1 = SysEx Message ID, defined by Conductive Labs

    case SysexMessageType::SetAllLEDs:
      // Serial0_USB.println(String("allLeds   LED#=") + data[2] + " IsOn=" + data[3] + "  RGB=" + data[4] + "," + data[5] + "," + data[6]);
      // for (auto j=0; j<NUM_NEOPIX-5; ++j) { // -5 was legacy bug fix hack
      for (auto j=0; j<NUM_NEOPIX; ++j) {
          addAniLED(ledIndexLUT[j], {static_cast<uint8_t>(data[5] * dimLED_G),  // GREEN
                                     static_cast<uint8_t>(data[4] * dimLED_R),  // RED
                                     static_cast<uint8_t>(data[6] * dimLED_B)}, // BLUE
                                     150, LED_LERP);
      }
      break;

    case SysexMessageType::SetSingleLEDColor: {
      uint8_t hwLed = ledIndexLUT[data[2]];
      // Serial0_USB.println(String("oneLed   LED#=") + data[2] + " IsOn=" + data[3] + "  RGB=" + data[4] + "," + data[5] + "," + data[6]);
      if (hwLed < NUM_NEOPIX) {
          addAniLED(hwLed, {static_cast<uint8_t>(data[5] * dimLED_G),  // GREEN
                            static_cast<uint8_t>(data[4] * dimLED_R),  // RED
                            static_cast<uint8_t>(data[6] * dimLED_B)}, // BLUE
                            150, LED_LERP);
      }
      break;
    }

    case SysexMessageType::SetSingleLEDColorInGroup: {
      uint8_t hwLed = ledIndexLUT[data[2]];
      auto ledGroupNum = whichLEDGroup(hwLed);
      // Serial0_USB.println(String("oneLedInGroup   hwLED#=") + hwLed + "[" + data[2] + "] IsOn=" + data[3] + "  RGB=" + data[4] + "," + data[5] + "," + data[6] +
      //                       " LED Group " + ledGroupNum + " first:" + ledGroupInfo[ledGroupNum][0]);
      for (auto j = 0; j < MAX_NUM_LEDS_PER_GROUP; ++j) {
          if (ledGroupInfo[ledGroupNum][j] == NO_LED) { break; }
          addAniLED(ledGroupInfo[ledGroupNum][j], {0,0,0,}, 150, LED_LERP);
      }
      addAniLED(hwLed, {static_cast<uint8_t>(data[5] * dimLED_G),  // GREEN
                        static_cast<uint8_t>(data[4] * dimLED_R),  // RED
                        static_cast<uint8_t>(data[6] * dimLED_B)}, // BLUE
                        150, LED_LERP);
      break;
    }

    case SysexMessageType::SetColorOfLEDGroup: {
      uint8_t hwLed = ledIndexLUT[data[2]];
      auto ledGroupNum = whichLEDGroup(hwLed);
      // Serial0_USB.println(String("allLedsInGroup   hwLED#=") + hwLed + "[" + data[2] + "] IsOn=" + data[3] + "  RGB=" + data[4] + "," + data[5] + "," + data[6] +
      //                       " LED Group " + ledGroupNum + " first:" + ledGroupInfo[ledGroupNum][0]);
      for (auto j = 0; j < MAX_NUM_LEDS_PER_GROUP; ++j) {
          if (ledGroupInfo[ledGroupNum][j] == NO_LED) { break; }
          addAniLED(ledGroupInfo[ledGroupNum][j],
                       {static_cast<uint8_t>(data[5] * dimLED_G),  // GREEN
                        static_cast<uint8_t>(data[4] * dimLED_R),  // RED
                        static_cast<uint8_t>(data[6] * dimLED_B)}, // BLUE
                        150, LED_LERP);
      }
      break;
    }

    default:
      LOGLINE_MIDI_DEBUG(String("   !!! NOT Handled SysEx Msg - Type#") + data[1]);

  } // end switch

} // end fcn HandleSysEx

// =======================================================

void MIDI_USB_SERIAL_DEV_Get() {
    if (MIDI_USB_SERIAL_DEV.read()) {
    LOGLINE_MIDI_DEBUG("READ MIDI_USB_SERIAL_DEV (USB_Serial1) From PC (Serial)");

      if (PICO_TO_CM == MIDI_Destinations_Enums::USB_Serial1)  // don't cause a mid loop
      {
        // Handle inbound MIDI (non-SysEx) form the Synth Engine here
        LOGLINE_MIDI_DEBUG("MIDI rx'ed from WTS Synth Engine");
        return;
      }

    } // end if is there MIDI
} // end fcn MIDI_USB_Get

// =======================================================

void MIDI_USB_DEV_Get() {
    if (MIDI_USB_DEV.read()) {
    LOGLINE_MIDI_DEBUG("READ MIDI_USB_DEV (USB_MIDI) From PC (MIDI/DAW)");

      if (PICO_TO_CM == MIDI_Destinations_Enums::USB_MIDI)  // don't cause a mid loop
      {
        // Handle inbound MIDI (non-SysEx) form the Synth Engine here
        LOGLINE_MIDI_DEBUG("MIDI rx'ed from WTS Synth Engine");
        return;
      }

      // Send out to the Synth Engine (Product - on the CM5) / (Dev - on the PC/MAC) - this should be the "keeper" code
      if (prevMidiMsgSrcPort != wtsSourcePort::SRC_MIDI_USB_DEV ) { // if MIDI msg source changes notify WTS synth Engine
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, (uint8_t)ControllerMidiEnums::WTS_SRC_PORT, (uint8_t)wtsSourcePort::SRC_MIDI_USB_DEV, 16);
        delayMicroseconds(50);
      }
      Send_MIDI_To_Port(PICO_TO_CM, MIDI_USB_DEV.getType(), MIDI_USB_DEV.getData1(), MIDI_USB_DEV.getData2(), MIDI_USB_DEV.getChannel());
      delayMicroseconds(50);
      prevMidiMsgSrcPort = wtsSourcePort::SRC_MIDI_USB_DEV;

    } // end if is there MIDI
} // end fcn MIDI_USB_Get

// =======================================================

void MIDI_5_PIN_Get() {
    // LOGLINE_MIDI_DEBUG("READ MIDI_5_PIN...");
    if (MIDI_5_PIN_UART0.read()) {
        LOGLINE_MIDI_DEBUG(String("READ MIDI_5_PIN...  Type ") + MIDI_5_PIN_UART0.getType() + " Data " + MIDI_5_PIN_UART0.getData1() + "-" + MIDI_5_PIN_UART0.getData2()  + " Ch" + MIDI_5_PIN_UART0.getChannel());

      // highly unlikely that this port is ever used to connect the PICO to the CM5
      if (PICO_TO_CM == MIDI_Destinations_Enums::UART0_5Pin_MIDI)  // don't cause a mid loop
      {
        // Handle inbound MIDI (non-SysEx) form the Synth Engine here
        LOGLINE_MIDI_DEBUG("MIDI rx'ed from WTS Synth Engine");
        return;
      }

      // Send out to the Synth Engine (Product - on the CM5) / (Dev - on the PC/MAC) - this should be the "keeper" code
      if (prevMidiMsgSrcPort != wtsSourcePort::SRC_MIDI_5_PIN ) {  // if MIDI msg source changes notify WTS synth Engine
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, (uint8_t)ControllerMidiEnums::WTS_SRC_PORT, (uint8_t)wtsSourcePort::SRC_MIDI_5_PIN, 15);
        delayMicroseconds(50);
      }
      Send_MIDI_To_Port(PICO_TO_CM, MIDI_5_PIN_UART0.getType(), MIDI_5_PIN_UART0.getData1(), MIDI_5_PIN_UART0.getData2(), MIDI_5_PIN_UART0.getChannel());
      delayMicroseconds(50);
      prevMidiMsgSrcPort = wtsSourcePort::SRC_MIDI_5_PIN;

    } // end if is there MIDI
} // end fcn MIDI_5_PIN_Get

//=================== P A N I C  PORT ==================

 void MIDIPanic() {

    LOGLINE_MIDI_DEBUG(String("Panic To USB:"));
    for (auto channel=1; channel<=16; channel++) {
        delay(1);
        //MIDI_USB_DEV.sendControlChange((midi::DataByte)MidiControllerIndex::All_Notes_Mute, 0, theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, (uint8_t)MidiControllerIndex::All_Notes_Mute, 0, theMIDIChan);

    }

} // end fcn MIDIPanic

//====================================================================

void ButPressed(uint8_t srPortNum, butInfoType butInfo)  {
  //LOGLINE_BUT_ENC_DEBUG(String("Prs: ") + srPortNum + ":" + butInfo.modNum + ":" + butInfo.modButNum + "-" + butInfo.ordButNum);
  //LOGLINE_BUT_ENC_DEBUG(String("Button Prs: port=") + srPortNum + " but num=" + butInfo.modButNum + " ordinal but num=" + butInfo.ordButNum);

  if (srPortNum == SR0) {
    switch (butInfo.ordButNum) {

      case SR0_BUTNUM_LOW_INIT:
        //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::InitButtonPress, theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::InitButtonPress, theMIDIChan);
        break;

      case SR0_BUTNUM_COPY:
        //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::CopyButtonPress, theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::CopyButtonPress, theMIDIChan);
        break;
      case SR0_BUTNUM_PASTE:
        //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::PasteButtonPress, theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::PasteButtonPress, theMIDIChan);
        break;
      case SR0_BUTNUM_RND:
        //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::RndButtonPress, theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::RndButtonPress, theMIDIChan);
        break;
      case SR0_BUTNUM_UNDO:
        //MIDI_USB.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::UndoButtonPress, theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::UndoButtonPress, theMIDIChan);
        break;
      case SR0_BUTNUM_REDO:
        //MIDI_USB.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::RedoButtonPress, theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::RedoButtonPress, theMIDIChan);
        break;
    } // end switch
  } // end if

  else { // srPortNum == SR2
    switch (butInfo.ordButNum) {
      case SR1_BUTNUM_SHIFT:
        gShiftHeld = true;
        //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::ShiftButtonPress, theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::ShiftButtonPress, theMIDIChan);
        break;

      case SR1_BUTNUM_HIGH_INIT:
        //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::InitButtonPress, theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::InitButtonPress, theMIDIChan);
        break;

      case SR1_BUTNUM_LEARN:
        //MIDI_USB.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::LearnButtonPress, theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::LearnButtonPress, theMIDIChan);
        break;

      case SR1_BUTNUM_LOAD:
        //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::LoadButtonPress, theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::LoadButtonPress, theMIDIChan);
        break;
      case SR1_BUTNUM_SAVE:
        //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::SaveButtonPress, theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::SaveButtonPress, theMIDIChan);
        break;

      case SR1_BUTNUM_HELP:
      gHelpHeld = true;
        //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::HelpButtonPress, theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::HelpButtonPress, theMIDIChan);
        break;

      case SR1_BUTNUM_USER_NW:
        gUserNwHeld = true;
        // Serial0_USB.println("USER_NW was 1");
        break;
      case SR1_BUTNUM_USER_NE:
        gUserNeHeld = true;
        // Serial0_USB.println("USER_NE was 2");
        break;
      case SR1_BUTNUM_USER_SE:
        gUserSeHeld = true;
        // Serial0_USB.println("USER_SE was 3");
        break;
      case SR1_BUTNUM_USER_SW:
        // Serial0_USB.println("USER_SW was 4");
        gUserSwHeld = true;
        break;

      case SR1_BUTNUM_VOICE_MOD:
        gVoiceHeld = true;
        break;
      case SR1_BUTNUM_FILTER_MOD:
        gFilterHeld = true;
        break;

    } // end switch
  } // end else

} // end fcn BUT Pressed

//====================================================================

void ButHeld(uint8_t srPortNum, butInfoType butInfo) {
  //LOGLINE_BUT_ENC_DEBUG(String("Hld: ") + srPortNum + ":" + butInfo.modNum + ":" + butInfo.modButNum + "-" + butInfo.ordButNum);
  //LOGLINE_BUT_ENC_DEBUG(String("Button Held: port=") + srPortNum + " but num=" + butInfo.modButNum + " ordinal but num=" + butInfo.ordButNum);
}  // end fcn BUT Held

//====================================================================

void ButRelease(uint8_t srPortNum, butInfoType butInfo) {
  //LOGLINE_BUT_ENC_DEBUG(String("Rel: ") + srPortNum + ":" + butInfo.modNum + ":" + butInfo.modButNum + "-" + butInfo.ordButNum + " (" + butInfo.elpsTime + ")");
  LOGLINE_BUT_ENC_DEBUG(String("ButRel: port=") + srPortNum + " but=" + butInfo.modButNum + " ord=" + butInfo.ordButNum);

  // this helps keeps state for GlobalMod i.e. Mixer, Patch Lib, Settings
  if ((butInfo.ordButNum == SR1_BUTNUM_VOICE_MOD)  ||
      (butInfo.ordButNum == SR1_BUTNUM_FILTER_MOD) ||
      (butInfo.ordButNum == SR1_BUTNUM_PATH_MOD)   ||
      (butInfo.ordButNum == SR1_BUTNUM_TERRAIN_MOD)) {
      lastModuleWasGlobalMod = false;
  }

  // Sends the Button Release message to the Synth Engine
  if (srPortNum == SR0) {
    if (butInfo.ordButNum >= SR0_BUTNUM_FILTER_SHORTCUT && butInfo.ordButNum <= SR0_BUTNUM_PASTE) {
        //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)SR0_ControllerButtonToSynthEnumLUT[butInfo.ordButNum], theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)SR0_ControllerButtonToSynthEnumLUT[butInfo.ordButNum], theMIDIChan);
    }

    // TEMPORARY until the audition is implemented in the Synth engine
    if (butInfo.ordButNum ==  SR0_BUTNUM_PLAY) {
      if (gShiftHeld) {
        //MIDI_USB_DEV.sendNoteOn(50,127,1);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::NoteOn, 50,127,1);
        delay(500);
        //MIDI_USB_DEV.sendNoteOn(54,127,1);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::NoteOn, 54,127,1);
        delay(500);
        //MIDI_USB_DEV.sendNoteOn(57,127,1);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::NoteOn, 57,127,1);
        delay(500);
        //MIDI_USB_DEV.sendNoteOn(62,127,1);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::NoteOn, 62,127,1);
        delay(1500);
        // MIDI_USB_DEV.sendNoteOff(50,127,1);
        // MIDI_USB_DEV.sendNoteOff(54,127,1);
        // MIDI_USB_DEV.sendNoteOff(57,127,1);
        // MIDI_USB_DEV.sendNoteOff(62,127,1);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::NoteOff, 50,127,1);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::NoteOff, 54,127,1);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::NoteOff, 57,127,1);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::NoteOff, 62,127,1);
      }
      else {
        for (auto j=0; j<=20; ++j) {
          auto noteVal = j * 3 + 30;
          //MIDI_USB_DEV.sendNoteOn(noteVal,127,j%4+1);
          Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::NoteOn, noteVal, 127, j%4+1);
          delay(150);
          //MIDI_USB_DEV.sendNoteOff(noteVal,127,j%4+1);
          Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::NoteOff, noteVal, 127, j%4+1);
        }
      }
    }
  } // end if SR0

  else { // srPortNum == SR1
    switch (butInfo.ordButNum) {

      // For these But Rel - just send button release to Synth engine...
      case SR1_BUTNUM_SYNTH_A:
      case SR1_BUTNUM_SYNTH_B:
      case SR1_BUTNUM_SYNTH_C:
      case SR1_BUTNUM_SYNTH_D:
      case SR1_BUTNUM_HIGH_INIT:
      case SR1_BUTNUM_POV:
      case SR1_BUTNUM_GRID:
      case SR1_BUTNUM_VAL:
      case SR1_BUTNUM_LFO:
      case SR1_BUTNUM_ENV:
      case SR1_BUTNUM_EXP:
      case SR1_BUTNUM_MODULATE_ENC0:
      case SR1_BUTNUM_MODULATE_ENC1:
      case SR1_BUTNUM_MODULATE_ENC2:
      case SR1_BUTNUM_MODULATE_ENC3:
      case SR1_BUTNUM_ENTER:
      case SR1_BUTNUM_ROW_UP:
      case SR1_BUTNUM_ROW_DOWN:
      case SR1_BUTNUM_LEARN:
      case SR1_BUTNUM_LOAD:
      case SR1_BUTNUM_SAVE:
      case SR1_BUTNUM_PATH_MOD:
      case SR1_BUTNUM_TERRAIN_MOD:
      case SR1_BUTNUM_SEQ:
      case SR1_BUTNUM_ARP:
      case SR1_BUTNUM_REST:
      case SR1_BUTNUM_TIE:
      case SR1_BUTNUM_REC:
      case SR1_BUTNUM_STOP:
      case SR1_BUTNUM_PLAY_PAUSE:
        //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)SR1_ControllerButtonToSynthEnumLUT[butInfo.ordButNum], theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)SR1_ControllerButtonToSynthEnumLUT[butInfo.ordButNum], theMIDIChan);
        break;


      case SR1_BUTNUM_WTS: // Resets WTS/HW CNTLR handshake
        if (gShiftHeld & gHelpHeld) {
          gMidiHandShakeComplete = false;
          break;
        }
        if (gShiftHeld) { // Triggers Panic aka All notes off
          MIDIPanic();
          break;
        }

        else {
          if (butInfo.elpsTime > 6000) { // System Shut Down
            //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerMidiEnums::WTSButtonSystemOff, theMIDIChan);
            Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerMidiEnums::WTSButtonSystemOff, theMIDIChan);
          }
          else if (butInfo.elpsTime > 3000) { // App Shut Down
            //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerMidiEnums::WTSButtonAppOff, theMIDIChan);
            Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerMidiEnums::WTSButtonAppOff, theMIDIChan);
          }
          else { // Does nothing
            //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::WTSButton, theMIDIChan);
            Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::WTSButton, theMIDIChan);
          }
          break;
        }
      case SR1_BUTNUM_SHIFT:
        gShiftHeld = false;
        //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::ShiftButtonRelease, theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::ShiftButtonRelease, theMIDIChan);
        break;


      case SR1_BUTNUM_HELP:
        gHelpHeld = false;
        //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::HelpButtonRelease, theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::HelpButtonRelease, theMIDIChan);
        break;

  // The User Selector has 4 positions NW, NE, SW, SE & 2 combo-positions Top & Bottom - here is the mapping to UserButtonX
      // Clockwise...
      // TOP -> NW & NE    = UserButton0(37) - HW LED 35  - ledIndexLUT 12
      // NE                = UserButton1(38) - HW LED  6  - ledIndexLUT 13
      // SE                = UserButton2(39) - HW LED  7  - ledIndexLUT 14
      // Bottom -> SW & SE = UserButton3(40) - HW LED 36  - ledIndexLUT 15
      // SW                = UserButton4(41) - HW LED  8  - ledIndexLUT 16
      // NW                = UserButton5(42) - HW LED  5  - ledIndexLUT 17

      case SR1_BUTNUM_USER_NW: // NW
        if (gUserNeHeld && gUserNwHeld) { // Select "UserButton0", Top - inbetween NW and NE
          //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::UserButton0, theMIDIChan);
          Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::UserButton0, theMIDIChan);
          gUserNwHeld = gUserNeHeld = false;
        }
        else if (!gUserNeHeld && gUserNwHeld) { // Select "UserButton5", NW
          //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::UserButton5, theMIDIChan);
          Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::UserButton5, theMIDIChan);
          gUserNwHeld = false;
        }
        break;

      case SR1_BUTNUM_USER_NE: // NE
        if (gUserNeHeld && gUserNwHeld) { // Select "UserButton0", Top - inbetween NW and NE
          //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::UserButton0, theMIDIChan);
          Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::UserButton0, theMIDIChan);
          gUserNwHeld = gUserNeHeld = false;
        }
        else if (gUserNeHeld && !gUserNwHeld) { // Select "UserButton1", NE
          //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::UserButton1, theMIDIChan);
          Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::UserButton1, theMIDIChan);
          gUserNeHeld = false;
        }
        break;


      case SR1_BUTNUM_USER_SE: // SE
        if (gUserSwHeld && gUserSeHeld) { // Select "UserButton3", Bottomp - inbetween SW and SE
          //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::UserButton3, theMIDIChan);
          Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::UserButton3, theMIDIChan);
          gUserSeHeld = gUserSwHeld = false;

        }
        else if (!gUserSwHeld && gUserSeHeld) { // Select "UserButton2", SE
          //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::UserButton2, theMIDIChan);
          Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::UserButton2, theMIDIChan);
          gUserSeHeld = false;
        }
        break;

      case SR1_BUTNUM_USER_SW: // SW
        if (gUserSwHeld && gUserSeHeld) { // Select "UserButton3", Bottomp - inbetween SW and SE
          //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::UserButton3, theMIDIChan);
          Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::UserButton3, theMIDIChan);
          gUserSeHeld = gUserSwHeld = false;
        }
        else if (gUserSwHeld && !gUserSeHeld) { // Select "UserButton5", SW
          //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::UserButton4, theMIDIChan);
          Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::UserButton4, theMIDIChan);
          gUserSwHeld = false;
        }
        break;

  // The Module Selector has 4 positions NW, NE, SW, SE & 1 combo-position Top
      case SR1_BUTNUM_VOICE_MOD:
        if (gFilterHeld && gVoiceHeld) { // Select "FX", inbetween Voice and Filter
          //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::EffectsSelectButton, theMIDIChan);
          Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::EffectsSelectButton, theMIDIChan);
          gFilterHeld = gVoiceHeld = false;
        }
        else if (!gFilterHeld && gVoiceHeld) {
          //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::VoiceSelectButton, theMIDIChan);
          Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::VoiceSelectButton, theMIDIChan);
          gVoiceHeld = false;
        }
        break;

      case SR1_BUTNUM_FILTER_MOD:
        if (gFilterHeld && gVoiceHeld) { // Select "FX", inbetween Voice and Filter
          //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::EffectsSelectButton, theMIDIChan);
          Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::EffectsSelectButton, theMIDIChan);
          gFilterHeld = gVoiceHeld = false;
        }
        else if (gFilterHeld && !gVoiceHeld) {
          //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::FilterSelectButton, theMIDIChan);
          Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::FilterSelectButton, theMIDIChan);
          gFilterHeld = false;
        }
        break;


      case SR1_BUTNUM_GLOBAL_MOD:
        // Toggle State: Settings/PatchLib/Mixer - The WTS engine should keep state of this!!!  Just a hack until we sync up with the Engine!
        if (lastModuleWasGlobalMod) {
          globalModule = (globalModule + 1) % 3;
        }
        else {
          lastModuleWasGlobalMod = true;
        }


        if (globalModule == SR1_BUTNUM_GLOBAL_MOD_SETTINGS) {
          //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::SettingsSelectButton, theMIDIChan);
          Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::SettingsSelectButton, theMIDIChan);
        }
        else if (globalModule == SR1_BUTNUM_GLOBAL_MOD_PATCHLIB) {
          //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::PatchesSelectButton, theMIDIChan);
          Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::PatchesSelectButton, theMIDIChan);
        }
        else if (globalModule == SR1_BUTNUM_GLOBAL_MOD_MIXER) {
          //MIDI_USB_DEV.sendControlChange((midi::DataByte)CCButtonPress, (uint8_t)ControllerButtonEnums::MixerSelectButton, theMIDIChan);
          Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, CCButtonPress, (uint8_t)ControllerButtonEnums::MixerSelectButton, theMIDIChan);
        }
        break;

    } // end switch
  } // end else

} // end fcn BUT Release

//====================================================================

void EncChanged(uint8_t srPortNum, encInfoType encInfo) { // encInfoType = .modNum .modEncNum .ordEncNum .encRotDir
  //LOGLINE_BUT_ENC_DEBUG(String("Enc: ") + srPortNum + ":" + encInfo.modNum + ":" + encInfo.modEncNum + "-" + encInfo.ordEncNum + " -> " + encInfo.encRotDir + " accel = " + encInfo.encRotVel);
  //LOGLINE_BUT_ENC_DEBUG(String("Enc: port=") + srPortNum + " num=" + encInfo.modEncNum + " ord=" + encInfo.ordEncNum + "      dir=" + encInfo.encRotDir + " accel=" + encInfo.encRotVel);

  uint8_t ccEncVal = 0;

  if (encInfo.encRotDir == ROTATION_INC) { ccEncVal = 65 + encInfo.encRotVel; }
  else { ccEncVal = 63 - encInfo.encRotVel; }
  LOGLINE_BUT_ENC_DEBUG(String("Enc:") + encInfo.ordEncNum + " dir=" + encInfo.encRotDir + "  vel=" + encInfo.encRotVel + " ccEncVal=" + ccEncVal);
  if (encInfo.encRotVel == 4) { LOGLINE_BUT_ENC_DEBUG("4444444444444444444"); }

  if (srPortNum == SR0) {
    if (encInfo.ordEncNum >= SR0_ENCNUM_MAIN_VOLUME && encInfo.ordEncNum <= SR0_ENCNUM_MENU5) {
        //MIDI_USB_DEV.sendControlChange(static_cast<unsigned>(SR0_ControllerEncToSynthEnumLUT[encInfo.ordEncNum]), ccEncVal, theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, static_cast<unsigned>(SR0_ControllerEncToSynthEnumLUT[encInfo.ordEncNum]), ccEncVal, theMIDIChan);
    }
  }

  else { // srPortNum == SR1
    if (encInfo.ordEncNum >= SR1_ENCNUM_MODULATION_ENC0 && encInfo.ordEncNum <= SR1_ENCNUM_USER_ENC1) {
        //MIDI_USB_DEV.sendControlChange(static_cast<unsigned>(SR1_ControllerEncToSynthEnumLUT[encInfo.ordEncNum]), ccEncVal, theMIDIChan);
        Send_MIDI_To_Port(PICO_TO_CM, (uint8_t)midi::ControlChange, static_cast<unsigned>(SR1_ControllerEncToSynthEnumLUT[encInfo.ordEncNum]), ccEncVal, theMIDIChan);
    }
  }

  delayMicroseconds(100);

} // end fcn ENC Changed

//====================================================================

GRBPixelType LEDLERP(GRBPixelType theRGBStart, GRBPixelType theRGBEnd, float lerpPos) {

return { theRGBEnd.r + static_cast<float>(theRGBStart.r - theRGBEnd.r) * lerpPos,
         theRGBEnd.g + static_cast<float>(theRGBStart.g - theRGBEnd.g) * lerpPos,
         theRGBEnd.b + static_cast<float>(theRGBStart.b - theRGBEnd.b) * lerpPos};
}

//====================================================================

void BootUpLEDLightShow() {
    //for (auto i=LED_VOICE_PATCH; i<LED_MIXER_ENV; ++i) { theLEDs[i] = theColWhite; } dma_ws2812_leds_update(); delay(500);

    for (auto j=0; j<=NUM_NEOPIX; ++j) {
      uint8_t pixStep = (uint8_t)((float)j * 127.f / NUM_NEOPIX);
      GRBPixelType gradCol = GetGradVal( pixStep, theColTeal, theColPurple, theColYellow);
      addAniLED(j, gradCol, 500, LED_LERP);
      for (auto i=0; i<10; ++i) { updateAniLEDs(); dma_ws2812_leds_update(); delay(8); }
    }
    delay(200);
    for (auto j=0; j<NUM_NEOPIX; ++j) {
      addAniLED(ledIndexLUT[j], {0,0,0}, 150, LED_LERP);
    }
    return;


    // addAniLED(LED_MAIN_POWER, theColGreen, 500, LED_LERP);
    // for (auto i=0; i<10; ++i) { updateAniLEDs(); dma_ws2812_leds_update(); delay(60); }

    // for (auto i=LED_VOICE_PATCH; i<=LED_MIXER_ENV; ++i) { addAniLED(i, theColWhite, 500, LED_LERP); }
    // for (auto i=0; i<10; ++i) { updateAniLEDs(); dma_ws2812_leds_update(); delay(60); }

    // addAniLED(LED_VOL_MACRO, theColGreen, 500, LED_LERP);
    // for (auto i=0; i<4; ++i) { updateAniLEDs(); dma_ws2812_leds_update(); delay(60); }
    // addAniLED(LED_SHIFT, theColRed, 500, LED_LERP);
    // for (auto i=0; i<4; ++i) { updateAniLEDs(); dma_ws2812_leds_update(); delay(60); }
    // addAniLED(LED_SUSTAIN, theColGreen, 500, LED_LERP);
    // for (auto i=0; i<4; ++i) { updateAniLEDs(); dma_ws2812_leds_update(); delay(60); }

    // addAniLED(LED_SYNTH_A, theColBlue, 500, LED_LERP);
    // addAniLED(LED_SYNTH_B, theColGreen, 500, LED_LERP);
    // addAniLED(LED_SYNTH_C, theColPurple, 500, LED_LERP);
    // addAniLED(LED_SYNTH_D, theColYellow, 500, LED_LERP);
    // for (auto i=0; i<10; ++i) { updateAniLEDs(); dma_ws2812_leds_update(); delay(60); }

    // addAniLED(LED_PATCH, theColBlue, 500, LED_LERP);
    // addAniLED(LED_BANK, theColRed, 500, LED_LERP);
    // addAniLED(LED_KB, theColPurple, 500, LED_LERP);
    // addAniLED(LED_ARP, theColGreen, 500, LED_LERP);
    // for (auto i=0; i<10; ++i) { updateAniLEDs(); dma_ws2812_leds_update(); delay(60); }

    // for (auto i=LED_GRID_BUT_0; i<LED_GRID_BUT_11; ++i) { addAniLED(i, theColTeal, 500, LED_LERP); }
    // for (auto i=0; i<10; ++i) { updateAniLEDs(); dma_ws2812_leds_update(); delay(60); }

    // addAniLED(LED_FILTER_ENC_SELECT, theColPurple, 500, LED_LERP);
    // for (auto i=0; i<10; ++i) { updateAniLEDs(); dma_ws2812_leds_update(); delay(60); }

    // theLEDs[LED_VOL_MACRO]=theColGreen; dma_ws2812_leds_update(); delay(250);
    // theLEDs[LED_SHIFT]=theColRed; dma_ws2812_leds_update(); delay(250);
    // theLEDs[LED_SUSTAIN]=theColGreen; dma_ws2812_leds_update(); delay(250);
    // theLEDs[LED_SYNTH_A]=theColBlue; theLEDs[LED_SYNTH_B]=theColGreen; theLEDs[LED_SYNTH_C]=theColPurple; theLEDs[LED_SYNTH_D]=theColYellow; dma_ws2812_leds_update(); delay(500);
    //theLEDs[LED_PATCH]=theColBlue; theLEDs[LED_BANK]=theColRed; theLEDs[LED_KB]=theColPurple; theLEDs[LED_ARP]=theColGreen; dma_ws2812_leds_update(); delay(500);
    //for (auto i=LED_GRID_BUT_0; i<LED_GRID_BUT_11; ++i) { theLEDs[i] = theColTeal; dma_ws2812_leds_update(); delay(200); }
    //delay(500);
    //theLEDs[LED_FILTER_ENC_SELECT]=theColPurple; dma_ws2812_leds_update(); delay(100);
}

//====================================================================

void addAniLED(uint8_t ledNum, GRBPixelType theRGBEnd, uint16_t duration, uint8_t theAniType) {
  // this is a one-per-LED task list, means that LED animations can't overlap and get replaced!
  if (ledNum > NUM_NEOPIX -1) {
    //Serial0_USB.println("In addAniLED(): ledNum > NUM_NEOPIX");
    return;
  }
  temporalLEDs[ledNum].theRGBStart = theLEDs[ledNum];
  temporalLEDs[ledNum].theRGBEnd = theRGBEnd;
  temporalLEDs[ledNum].duration = duration;
  temporalLEDs[ledNum].elpTime = duration;
  temporalLEDs[ledNum].theAniType = theAniType;
}

//====================================================================

void updateAniLEDs() {
  static uint32_t lastTime = 0;
  uint32_t deltTime = millis() - lastTime;

  for (auto i=0; i<NUM_NEOPIX; ++i) {
    if (temporalLEDs[i].elpTime != 0) {

      // countdown elapsed time to 0
      if (temporalLEDs[i].elpTime < deltTime) { temporalLEDs[i].elpTime = 0; }
      else { temporalLEDs[i].elpTime -= deltTime; }

      switch (temporalLEDs[i].theAniType) {
        case LED_LERP: {
          float lerpPos = (float)temporalLEDs[i].elpTime / (float)temporalLEDs[i].duration;
          //Serial0_USB.printf("lerpPos = %f\n", lerpPos);
          theLEDs[i] = LEDLERP(temporalLEDs[i].theRGBStart, temporalLEDs[i].theRGBEnd, lerpPos);
          } break;
        case LED_BLINK:
          break;
      } // end switch

    } // end if not 0
  } // end for

  lastTime = millis();

} // end fcn updateAniLEDs

//====================================================================

void cancelAniLEDs() {
  for (auto i=0; i<NUM_NEOPIX; ++i) {
    temporalLEDs[i].elpTime = 0;
  }
} // end fcn cancelAniLEDs

//====================================================================

void tempLightInitLEDs(boolean turnOn) {
  // if (turnOn) {
  //   memcpy(theTempLEDs, theLEDs, sizeof(theLEDs)); // save LEDs state
  //   for (auto i=LED_VOICE_PATCH; i<=LED_MIXER_ENV; ++i) {
  //     //theLEDs[i] = theColRed;
  //     addAniLED(i, theColRed, buttonDimONTime, LED_LERP);
  //   }
  //   for (auto i=LED_SYNTH_A; i<=LED_SYNTH_D; ++i) {
  //     //theLEDs[i] = theColRed;
  //     addAniLED(i, theColRed, buttonDimONTime, LED_LERP);
  //   }
  // }
  // else { // turnOff
  //   cancelAniLEDs();
  //   memcpy(theLEDs, theTempLEDs, sizeof(theLEDs)); // restore LEDs state
  // }
}

//====================================================================

void tempLightLoadLEDs(boolean turnOn) {
  // if (turnOn) {
  //   memcpy(theTempLEDs, theLEDs, sizeof(theLEDs)); // save LEDs state
  //   for (auto i=LED_VOICE_PATCH; i<=LED_MIXER_ENV; ++i) {
  //     addAniLED(i, theColYellow, buttonDimONTime, LED_LERP);
  //   }
  //   for (auto i=LED_SYNTH_A; i<=LED_SYNTH_D; ++i) {
  //     addAniLED(i, theColYellow, buttonDimONTime, LED_LERP);
  //   }
  // }
  // else { // turnOff
  //   cancelAniLEDs();
  //   memcpy(theLEDs, theTempLEDs, sizeof(theLEDs)); // restore LEDs state
  // }
}

//====================================================================

void tempLightSaveLEDs(boolean turnOn) {
  // if (turnOn) {
  //   memcpy(theTempLEDs, theLEDs, sizeof(theLEDs)); // save LEDs state
  //   for (auto i=LED_VOICE_PATCH; i<=LED_MIXER_ENV; ++i) {
  //     addAniLED(i, theColGreen, buttonDimONTime, LED_LERP);
  //   }
  //   for (auto i=LED_SYNTH_A; i<=LED_SYNTH_D; ++i) {
  //     addAniLED(i, theColGreen, buttonDimONTime, LED_LERP);
  //   }
  // }
  // else { // turnOff
  //   cancelAniLEDs();
  //   memcpy(theLEDs, theTempLEDs, sizeof(theLEDs)); // restore LEDs state
  // }
}

//====================================================================

void FillColSolidLEDarray(GRBPixelType theCol) {
  for (auto i=0; i<NUM_NEOPIX; ++i) {
    theLEDs[i].r = theCol.r;
    theLEDs[i].g = theCol.g;
    theLEDs[i].b = theCol.b;
  }
} // end fcn ClearAllNeoPix

//====================================================================

void FillColWashLEDarray(GRBPixelType theLEDArray[], uint8_t theLEDArraySize) {
  static uint32_t ledR=0, ledG=0, ledB=0;
  static float sinOffset1=0, sinOffset2=0, sinOffset3=0;
  //float ledMax=254, ledMin=20;
  float ledMax=35, ledMin=5;

    ledMax-=ledMin;

    sinOffset1+=0.027; if (sinOffset1>3.14) {sinOffset1=0.0;}
    sinOffset2+=0.035; if (sinOffset2>3.14) {sinOffset2=0.0;}
    sinOffset3+=0.019; if (sinOffset3>3.14) {sinOffset3=0.0;}

    ledR = (uint8_t)(sin(sinOffset1) * ledMax + ledMin);
    ledG = (uint8_t)(sin(sinOffset2) * ledMax + ledMin);
    ledB = (uint8_t)(sin(sinOffset3) * ledMax + ledMin);

    for (uint8_t i=0; i<theLEDArraySize; ++i) {
      theLEDArray[i].r = ledR;
      theLEDArray[i].g = ledG;
      theLEDArray[i].b = ledB;
    }

} // end fcn ColWash

//====================================================================

GRBPixelType GetGradVal(uint8_t theGradPosition, GRBPixelType theMinCol, GRBPixelType theMidCol, GRBPixelType theMaxCol) {
  // theGradPosition expected val range 0-127
  GRBPixelType rtnVal ={0,0,0};
  if (theGradPosition < 64) {
    rtnVal.r = ((64-theGradPosition) * (theMinCol.r - theMidCol.r) )/64 + theMidCol.r;
    rtnVal.g = ((64-theGradPosition) * (theMinCol.g - theMidCol.g) )/64 + theMidCol.g;
    rtnVal.b = ((64-theGradPosition) * (theMinCol.b - theMidCol.b) )/64 + theMidCol.b;
  }
  else { //if (theGradPosition >= 64) {
    rtnVal.r = ((theGradPosition-64) * (theMaxCol.r - theMidCol.r) )/64 + theMidCol.r;
    rtnVal.g = ((theGradPosition-64) * (theMaxCol.g - theMidCol.g) )/64 + theMidCol.g;
    rtnVal.b = ((theGradPosition-64) * (theMaxCol.b - theMidCol.b) )/64 + theMidCol.b;
  }

  return {rtnVal.r/2, rtnVal.g/2, rtnVal.b/2};
} // end fcn GetGradVal

//====================================================================

float fmap(float x, float minIn, float maxIn, float minOut, float maxOut) {
      float f = (x - minIn) * (maxOut - minOut) / (maxIn - minIn) + minOut;
      return f;
}

//====================================================================

void SendControllerInfo() {

  uint8_t SysExData[6] = {
      ControllerInfo.HW_MajRev,
      ControllerInfo.HW_MidRev,
      ControllerInfo.HW_MinRev,
      ControllerInfo.SW_MajRev,
      ControllerInfo.SW_MidRev,
      ControllerInfo.SW_MinRev
  };

  MIDI_USB_DEV.sendSysEx(sizeof(SysExData), SysExData);
}

//==============================================================================================================================

// void Send_MIDI_Raw(uint8_t midi_msg_type, uint8_t data1, uint8_t data2, uint8_t channel) {

//     // don't handle SysEx or F4/F5
//     if (midi_msg_type == midi::SystemExclusive ||     // SystemExclusive (0xF0)
//         midi_msg_type == midi::SystemExclusiveEnd ||  // SystemExclusiveEnd (0xF7)
//         midi_msg_type == 0xF4 ||                      // Reserved
//         midi_msg_type == 0xF5                         // Reserved
//         ) {
//       return;
//     }

//     uint8_t SIZE_OF_MSG = 3;

//     if (midi_msg_type == midi::ProgramChange || midi_msg_type == midi::AfterTouchChannel) { // Program Change (0xE0) || AfterTouchChannel (0xD0)
//       SIZE_OF_MSG = 2;
//       uint8_t midi_msg[SIZE_OF_MSG];
// /*verify*/ midi_msg[0] = midi_msg_type | channel - 1;
//       midi_msg[1] = data1;
//       Serial1_USB.write(midi_msg, SIZE_OF_MSG);
//     }

//     else if (midi_msg_type <= 0xE0) { // NoteOff, NoteOn, PolyphonicKeyPressure, ControlChange, PitchBend
//       uint8_t midi_msg[SIZE_OF_MSG]; //3
//       midi_msg[0] = midi_msg_type | channel - 1;
//       midi_msg[1] = data1;
//       midi_msg[2] = data2;
//       Serial1_USB.write(midi_msg, SIZE_OF_MSG);
//     }

//     else if (midi_msg_type == midi::TimeCodeQuarterFrame || midi_msg_type == midi::SongSelect) { // TimeCodeQuarterFrame (0xF1) || SongSelect (0xF3)
//       SIZE_OF_MSG = 2;
//       uint8_t midi_msg[SIZE_OF_MSG];
// /*verify*/ midi_msg[0] = midi_msg_type; // no channel info
//       midi_msg[1] = data1;
//       Serial1_USB.write(midi_msg, SIZE_OF_MSG);
//     }

//     else if (midi_msg_type == 0xF2) { // SongPositionPointer 3 bytes
//       uint8_t midi_msg[SIZE_OF_MSG]; //3
//       midi_msg[0] = midi_msg_type;
//       midi_msg[1] = data1;
//       midi_msg[2] = data2;
//       Serial1_USB.write(midi_msg, SIZE_OF_MSG);
//     }

//     else {
//       // Tune Request (0xF6), Timing Clock (0xF8), Reserved (Active Sensing) (0xF9),
//       // Start (0xFA), Continue (0xFB), Stop (0xFC), Reserved (Active Sensing) (0xFD),
//       // Reserved (System Reset) (0xFE), Reserved (System Reset) (0xFF)
//       Serial1_USB.write(midi_msg_type);
//     }

//     Serial1_USB.flush();
// }

void Send_MIDI_To_Port(MIDI_Destinations_Enums thePort, uint8_t midi_msg_type, uint8_t data1, uint8_t data2, uint8_t channel) {

    //Serial0_USB.println(String("In Send_MIDI_To_Port  port = ") + (uint8_t)thePort);

    switch (thePort) {
      case MIDI_Destinations_Enums::UART0_5Pin_MIDI:
        MIDI_5_PIN_UART0.send((midi::MidiType)midi_msg_type, (midi::DataByte)data1, (midi::DataByte)data2, (midi::Channel)channel);
        //Serial0_USB.println(String("Sending MIDI to UART0_5Pin_MIDI"));
        break;

      case MIDI_Destinations_Enums::UART1_Pico_To_CM_MIDI:
        MIDI_PICO_TO_CM_UART1.send((midi::MidiType)midi_msg_type, (midi::DataByte)data1, (midi::DataByte)data2, (midi::Channel)channel);
        //Serial0_USB.println(String("Sending MIDI to UART1_Pico_To_CM_MIDI"));
        break;

      case MIDI_Destinations_Enums::USB_MIDI:
        MIDI_USB_DEV.send((midi::MidiType)midi_msg_type, (midi::DataByte)data1, (midi::DataByte)data2, (midi::Channel)channel);
        //Serial0_USB.println(String("Sending MIDI to USB_Serial0_MIDI"));
        break;

      case MIDI_Destinations_Enums::USB_Serial1:
        MIDI_USB_SERIAL_DEV.send((midi::MidiType)midi_msg_type, (midi::DataByte)data1, (midi::DataByte)data2, (midi::Channel)channel);
        //Send_MIDI_Raw((midi::MidiType)midi_msg_type, (midi::DataByte)data1, (midi::DataByte)data2, (midi::Channel)channel);
        //Serial0_USB.println(String("Sending MIDI to USB_Serial1_Raw"));
        break;
    }
}