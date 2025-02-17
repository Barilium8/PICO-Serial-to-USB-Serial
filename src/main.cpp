
#include <Arduino.h>
#include <pico/stdlib.h> // added to make UF2 button work, see SR1_BUTNUM_HELP
#include <pico/bootrom.h> // added to make UF2 button work, see SR1_BUTNUM_HELP

#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"

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


#define PRINTB32(Num) for(int i=0; i<32; i++) { if(i%8==0){Serial0_USB.print("    ");} if(i%2Serial0_USBerial.prinSerial0_USB;}  Serial0_USB.write(((Num >> i) & 1) == 1 ?Serial0_USB '0'); } Serial0_USB0_USB.println();
#define PRINTB8(Num) for(int i=0; i<8; i++) { Serial0_USB.write(((Num >> i) & 1) == 1 ? '1' : '0'Serial0_USBerial.print(" ");

#define PICO_DEFAULT_LED_GPIO 25
#define HW_PROGRAM_BUTTON 26 // 26 new hw // was 2 // was 9

// MIDI PIN def
#define MIDI_LED_GPIO 22  // pin29 GPIO 22
//#define MIDI_RX1_GPIO  1  // pin2 GPIO 1
//#define MIDI_TX1_GPIO  0  // pin1 GPIO 0

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

uint8_t theMIDIChan = 16;


// function prototypes
void PICO_ADC_scanAll();

void MIDI_USB_DEV_Get();
void MIDI_5_PIN_Get();
void MIDIPanic();

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

  gpio_init(HW_PROGRAM_BUTTON);
  gpio_set_function(HW_PROGRAM_BUTTON, GPIO_FUNC_SIO);
  gpio_set_dir(HW_PROGRAM_BUTTON, GPIO_IN);
  pinMode(HW_PROGRAM_BUTTON, INPUT_PULLUP);

} // end setup

//-----------------------------------------------

void setup1() {
  Serial.begin(115'200); // USB serial init w/defaults

  // make sure to set '#define CFG_TUD_CDC 2' and '#define CFG_TUD_MIDI 2' in file tusb_config_rp2040.h in dir...
  /* C:\.platformio\packages\framework-arduinopico\libraries\Adafruit_TinyUSB_Arduino\src\arduino\ports\rp2040\ */
  //Serial.begin(1'000'000);

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

  // MIDI_PICO_TO_CM_UART1.setHandleControlChange(HandleCC);
  // MIDI_USB_DEV.setHandleControlChange(HandleCC);
  // MIDI_USB_SERIAL_DEV.setHandleControlChange(HandleCC);
  // MIDI_5_PIN_UART0.setHandleControlChange(HandleCC);

  Serial.println("PICO Serial-to-USB-Serial Starting... ");
  //Serial.println("Serial1_USB Starting ...");

}  // end setup1

//-----------------------------------------------

void loop() {
  static uint32_t prevTime1 = 0;
  static uint32_t prevTime2 = 0;

  // Read all the inbound shift reg data for buttons and encoders
  if ( (micros() - prevTime1) > 800) { // 800us period. FYI the sr1&2 scans takes ~600us
    prevTime1 = micros();

    if (!gpio_get(HW_PROGRAM_BUTTON)) {
      //Serial.println("going into Mass Storage boot mode...");
      //delay (500);
      reset_usb_boot(0,0);
    }
  }

  // if ( (millis() - prevTime2) > 10000) {
  //     prevTime2 = millis();
  //     Serial.println("...serial 1 still alive!");
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

    MIDI_5_PIN_Get();
  }
} // end loop1

// =======================================================

void HandleCC(uint8_t channel, uint8_t theCC, uint8_t theCCVal) {
  //Serial.println(String("In CC Handler - ch=") + channel + " cc=" + theCC + " val=" + theCCVal);

  //SendControllerInfo();

} // end fcn HandleCC

// =======================================================

void HandleSysEx(byte *data, unsigned int length) {
  //Serial.println(String("SysEx Message... ") + data[0] + " " + data[1] + " " + data[2] + " "+ data[3] + " "+ data[4] + " "+ data[5] + " "+ data[6] + " "+ data[7]);

} // end fcn HandleSysEx

// =======================================================

void MIDI_USB_SERIAL_DEV_Get() {
    if (MIDI_USB_SERIAL_DEV.read()) {
    } // end if is there MIDI
} // end fcn MIDI_USB_Get

// =======================================================

void MIDI_USB_DEV_Get() {
    if (MIDI_USB_DEV.read()) {
    } // end if is there MIDI
} // end fcn MIDI_USB_Get

// =======================================================

void MIDI_5_PIN_Get() {
    if (MIDI_5_PIN_UART0.read()) {
    } // end if is there MIDI
} // end fcn MIDI_5_PIN_Get

// =======================================================


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


void Send_MIDI_To_Port(MIDI_Destinations_Enums thePort, uint8_t midi_msg_type, uint8_t data1, uint8_t data2, uint8_t channel) {

    //Serial.println(String("In Send_MIDI_To_Port  port = ") + (uint8_t)thePort);

    switch (thePort) {
      case MIDI_Destinations_Enums::UART0_5Pin_MIDI:
        MIDI_5_PIN_UART0.send((midi::MidiType)midi_msg_type, (midi::DataByte)data1, (midi::DataByte)data2, (midi::Channel)channel);
        //Serial.println(String("Sending MIDI to UART0_5Pin_MIDI"));
        break;

      case MIDI_Destinations_Enums::UART1_Pico_To_CM_MIDI:
        MIDI_PICO_TO_CM_UART1.send((midi::MidiType)midi_msg_type, (midi::DataByte)data1, (midi::DataByte)data2, (midi::Channel)channel);
        //Serial.println(String("Sending MIDI to UART1_Pico_To_CM_MIDI"));
        break;

      case MIDI_Destinations_Enums::USB_MIDI:
        MIDI_USB_DEV.send((midi::MidiType)midi_msg_type, (midi::DataByte)data1, (midi::DataByte)data2, (midi::Channel)channel);
        //Serial.println(String("Sending MIDI to USB_Serial0_MIDI"));
        break;

      case MIDI_Destinations_Enums::USB_Serial1:
        MIDI_USB_SERIAL_DEV.send((midi::MidiType)midi_msg_type, (midi::DataByte)data1, (midi::DataByte)data2, (midi::Channel)channel);
        //Serial.println(String("Sending MIDI to USB_Serial1_Raw"));
        break;
    }
}