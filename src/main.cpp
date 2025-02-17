
#include <Arduino.h>
#include <pico/stdlib.h> // added to make UF2 button work, see SR1_BUTNUM_HELP
#include <pico/bootrom.h> // added to make UF2 button work, see SR1_BUTNUM_HELP

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

#define PICO_DEFAULT_LED_GPIO 25
#define HW_PROGRAM_BUTTON 26 // 26 new hw // was 2 // was 9

// MIDI PIN def
#define MIDI_LED_GPIO 22  // pin29 GPIO 22
//#define MIDI_RX1_GPIO  1  // pin2 GPIO 1
//#define MIDI_TX1_GPIO  0  // pin1 GPIO 0

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

uint8_t theMIDIChan = 16;


// function prototypes
void MIDI_PICO_TO_CM_UART1_Get();
void MIDI_USB_DEV_Get();
void MIDIPanic();

float fmap(float x, float a, float b, float c, float d);
void SendControllerInfo();

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

  ControllerInfo.HW_MajRev = 0;
  ControllerInfo.HW_MidRev = 0;
  ControllerInfo.HW_MinRev = 1;

  ControllerInfo.SW_MajRev = 0;
  ControllerInfo.SW_MidRev = 0;
  ControllerInfo.SW_MinRev = 1;

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

  MIDI_PICO_TO_CM_UART1.begin(MIDI_CHANNEL_OMNI);
  MIDI_PICO_TO_CM_UART1.turnThruOff();

  TinyUSB_MIDI.setStringDescriptor("SerialToUSBSerial");
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

  // Read all the inbound shift reg data for buttons and encoders
  if ( (micros() - prevTime1) > 800) { // 800us period. FYI the sr1&2 scans takes ~600us
    prevTime1 = micros();

    if (!gpio_get(HW_PROGRAM_BUTTON)) {
      //Serial.println("going into Mass Storage boot mode...");
      reset_usb_boot(0,0);
    }
  }

} // end loop

//-----------------------------------------------

void loop1() {
  static uint32_t prevTime1 = 0;

  // Read USB Serial Port for MIDI data
  if ( (micros() - prevTime1) > 800) { // 1000us 1ms
    prevTime1 = micros();

    MIDI_PICO_TO_CM_UART1_Get();
    MIDI_USB_DEV_Get();
  }
} // end loop1

// =======================================================

void HandleSysEx(byte *data, unsigned int length) {
  //Serial.println(String("SysEx Message... ") + data[0] + " " + data[1] + " " + data[2] + " "+ data[3] + " "+ data[4] + " "+ data[5] + " "+ data[6] + " "+ data[7]);

} // end fcn HandleSysEx

// =======================================================

void MIDI_PICO_TO_CM_UART1_Get() {
    if (MIDI_PICO_TO_CM_UART1.read()) {
      MIDI_USB_DEV.send(MIDI_PICO_TO_CM_UART1.getType(), MIDI_PICO_TO_CM_UART1.getData1(), MIDI_PICO_TO_CM_UART1.getData2(), MIDI_PICO_TO_CM_UART1.getChannel());
    }
}

// =======================================================

void MIDI_USB_DEV_Get() {
    if (MIDI_PICO_TO_CM_UART1.read()) {
      MIDI_USB_DEV.send(MIDI_USB_DEV.getType(), MIDI_USB_DEV.getData1(), MIDI_USB_DEV.getData2(), MIDI_USB_DEV.getChannel());
    }
}

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
