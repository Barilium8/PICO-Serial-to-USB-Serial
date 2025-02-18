
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

// create USB_SERIAL (CDC) object
Adafruit_USBD_CDC TinyUSB_SerialMIDI; // declare a USB 'Serial' port and wrap it in a MIDI intrerface
struct USB_SM_Dev_Settings : public midi::DefaultSettings{
    static const bool UseRunningStatus = false;
    static const bool HandleNullVelocityNoteOnAsNoteOff = true;
    static const bool Use1ByteParsing = true;
    static const long BaudRate = 460'800; // 31250;
    static const unsigned SysExMaxSize = 128;
};
MIDI_CREATE_CUSTOM_INSTANCE(Adafruit_USBD_CDC, TinyUSB_SerialMIDI, MIDI_USB_SERIAL_DEV, USB_SM_Dev_Settings);

// Creates MIDI on Serial1 aka UART0 (GPIO 0/1) to 'Loopback' MIDI to CM5
struct MIDI_LoopBack_Settings : public midi::DefaultSettings{
  static const bool UseRunningStatus = false;
  static const bool HandleNullVelocityNoteOnAsNoteOff = true;
  static const bool Use1ByteParsing = true;
  static const long BaudRate = 460'800;
  static const unsigned SysExMaxSize = 128;
};
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial1, MIDI_CM5_UART0, MIDI_LoopBack_Settings);

// Creates MIDI on Serial2 aka UART1 (GPIO 8/9) for PICO to ComputeModule MIDI "bus"
struct MIDI_UART1_Settings : public midi::DefaultSettings{
  static const bool UseRunningStatus = false;
  static const bool HandleNullVelocityNoteOnAsNoteOff = true;
  static const bool Use1ByteParsing = true;
   // tested at 2'000'000 and works
  static const long BaudRate = 460'800; //38'400; //31'250; //460'800; // 460800 baud tested output on PICO pin via analyser. Reqs 2MHz scan on the analyser! :-)
  static const unsigned SysExMaxSize = 128;
};
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial2, MIDI_PICO_UART1, MIDI_UART1_Settings);

// create USB_MIDI object
Adafruit_USBD_MIDI TinyUSB_MIDI; // declare a USB 'MIDI' port and wrap it in a MIDI intrerface
struct USBDev_Settings : public midi::DefaultSettings{
    static const bool UseRunningStatus = false;
    static const bool HandleNullVelocityNoteOnAsNoteOff = true;
    static const bool Use1ByteParsing = true;
    static const long BaudRate = 460'800; // 31250;
    static const unsigned SysExMaxSize = 128;
};
MIDI_CREATE_CUSTOM_INSTANCE(Adafruit_USBD_MIDI, TinyUSB_MIDI, MIDI_USB_DEV, USBDev_Settings);


uint8_t theMIDIChan = 16;


// function prototypes
void MIDI_PICO_UART1_Get(); // UART1 Serial2 port
void MIDI_USB_SERIAL_DEV_Get();   // PC/MAC Serial port
void MIDI_USB_DEV_Get();          // PC/MAC MIDI port

void MIDIPanic();
float fmap(float x, float a, float b, float c, float d);
void SendControllerInfo();
void BlinkLED(uint8_t times);

struct ControllerInfoType {
    uint8_t HW_MajRev = 0;
    uint8_t HW_MidRev = 0;
    uint8_t HW_MinRev = 0;
    uint8_t SW_MajRev = 0;
    uint8_t SW_MidRev = 0;
    uint8_t SW_MinRev = 0;
};
ControllerInfoType ControllerInfo = {};

bool gLoopBack = false;

//==============================================================================================

void setup() {

  pinMode(PICO_DEFAULT_LED_GPIO, OUTPUT);

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
  Serial.setStringDescriptor("DEBUG for SerialToUSBSerial");

  // make sure to set '#define CFG_TUD_CDC 2' and '#define CFG_TUD_MIDI 2' in file tusb_config_rp2040.h in dir...
  /* C:\Users\Steve\.platformio\packages\framework-arduinopico\libraries\Adafruit_TinyUSB_Arduino\src\arduino\ports\rp2040 */

  // Initialize Serial2 (UART1) for PICO-CM5 comms on GPIO8 (TX) and GPIO9 (RX)
  uart_init(uart0, 460'800); // tested at 2'000'000 and works
  gpio_set_function(0, GPIO_FUNC_UART);  // Set GPIO0 to UART function
  gpio_set_function(1, GPIO_FUNC_UART);  // Set GPIO1 to UART function

  // Initialize Serial2 (UART1) for PICO-CM5 comms on GPIO8 (TX) and GPIO9 (RX)
  uart_init(uart1, 460'800); // tested at 2'000'000 and works
  gpio_set_function(8, GPIO_FUNC_UART);  // Set GPIO8 to UART function
  gpio_set_function(9, GPIO_FUNC_UART);  // Set GPIO9 to UART function

  MIDI_PICO_UART1.begin(MIDI_CHANNEL_OMNI);
  MIDI_PICO_UART1.turnThruOff();

  TinyUSB_SerialMIDI.setStringDescriptor("SerialToUSBSerial");
  MIDI_USB_SERIAL_DEV.begin(MIDI_CHANNEL_OMNI);
  MIDI_USB_SERIAL_DEV.turnThruOff();

  TinyUSB_MIDI.setStringDescriptor("SerialToUSBSerial MIDI");
  MIDI_USB_DEV.begin(MIDI_CHANNEL_OMNI);
  MIDI_USB_DEV.turnThruOff();

  while( !TinyUSBDevice.mounted() ) { delay(1); }  // wait until device mounted


  delay (500);
  Serial.println(" PICO Serial-to-USB-Serial Starting... ");


}  // end setup1

//-----------------------------------------------

void loop() {
  static uint32_t prevTime1 = 0;
  static uint32_t prevTime2 = 0;
  static uint32_t prevTime3 = 0;

  if ( (micros() - prevTime1) > 800) {
    prevTime1 = micros();

    if (!gpio_get(HW_PROGRAM_BUTTON)) {
      Serial.println(" Going into Mass Storage boot mode...");
      reset_usb_boot(0,0);
    }
  }


  // Read all the inbound shift reg data for buttons and encoders
  if ( (millis() - prevTime2) > 10000) {
    prevTime2 = millis();
    BlinkLED(1);
  }

  static uint8_t tenTimes = 0;
  // Read all the inbound shift reg data for buttons and encoders
  if ( (millis() - prevTime3) > 500) {
    prevTime3 = millis();

    if (tenTimes < 10) {
      Serial.println(" Serial - Redirecting MIDI to MAC/PC (Serial).  Type '?' to see menu.");
      tenTimes++;
    }

    // TinyUSB_SerialMIDI.println("Serial - TinyUSB_SerialMIDI");
    // MIDI_USB_SERIAL_DEV.sendNoteOn(60,127,16);
    // MIDI_USB_SERIAL_DEV.sendNoteOff(60,127,16);
    // TinyUSB_SerialMIDI.println();
    String serialData = Serial.readString();
    //Serial.println(String("Echo: ") + serialData);

    if (serialData == "?") {
      Serial.println("  o---------------------------------------------o");
      Serial.println("  |  C = PICO <-> CM5                           |");
      Serial.println("  |  M = PICO <-> MAC/PC (Serial)               |");
      Serial.println("  |  D = Print Debug message 10 more times      |");
      Serial.println("  |  ? = this menu                              |");
      Serial.println("  o---------------------------------------------o");
      tenTimes = 10;
    }
    else if (serialData == "C") { gLoopBack = true;  Serial.println(" MIDI: PICO <-> CM5");}
    else if (serialData == "M") { gLoopBack = false; Serial.println(" MIDI: PICO <-> MAC/PC (Serial)");}
    else if (serialData == "D") { tenTimes = 0; }

  }

} // end loop

//-----------------------------------------------

void loop1() {
  static uint32_t prevTime1 = 0;

  // Read USB Serial Port for MIDI data
  if ( (micros() - prevTime1) > 800) { // 1000us 1ms
    prevTime1 = micros();

    MIDI_PICO_UART1_Get(); // UART1 Serial2 port
    MIDI_USB_SERIAL_DEV_Get(); // PC/MAC Serial port
    //MIDI_USB_DEV_Get();        // PC/MAC MIDI port
  }
} // end loop1

// =======================================================

void HandleSysEx(byte *data, unsigned int length) {
  Serial.println(String("SysEx Message: 1st 7 byts... ") + data[0] + " " + data[1] + " " + data[2] + " "+ data[3] + " "+ data[4] + " "+ data[5] + " "+ data[6] + " "+ data[7]);

} // end fcn HandleSysEx

// =======================================================

void MIDI_PICO_UART1_Get() {
    if (MIDI_PICO_UART1.read()) {
      if (gLoopBack) { // send to CM5
        MIDI_CM5_UART0.send(MIDI_PICO_UART1.getType(), MIDI_PICO_UART1.getData1(), MIDI_PICO_UART1.getData2(), MIDI_PICO_UART1.getChannel()); // outbound to PC/MAC Serial port
      }
      else { // send to PC/MAC
        //MIDI_USB_SERIAL_DEV.send(MIDI_PICO_UART1.getType(), MIDI_PICO_UART1.getData1(), MIDI_PICO_UART1.getData2(), MIDI_PICO_UART1.getChannel()); // outbound to PC/MAC Serial port
        //TinyUSB_SerialMIDI.flush();
        MIDI_USB_DEV.send(MIDI_PICO_UART1.getType(), MIDI_PICO_UART1.getData1(), MIDI_PICO_UART1.getData2(), MIDI_PICO_UART1.getChannel());        // outbound to PC/MAC MIDI port
      }

      BlinkLED(2);
    }
}

// =======================================================

void MIDI_USB_SERIAL_DEV_Get() { // inbound from PC/MAC Serial port (UART1)
  if (!gLoopBack) {
    if (MIDI_USB_SERIAL_DEV.read()) {
      MIDI_PICO_UART1.send(MIDI_USB_SERIAL_DEV.getType(), MIDI_USB_SERIAL_DEV.getData1(), MIDI_USB_SERIAL_DEV.getData2(), MIDI_USB_SERIAL_DEV.getChannel()); // outbound to Controller
      BlinkLED(3);
    }
  }
}

// =======================================================

void MIDI_UART0_CM5_Get() { // inbound from CM5 Serial port (UART0)
  if (gLoopBack) {
    if (MIDI_CM5_UART0.read()) {
      MIDI_PICO_UART1.send(MIDI_CM5_UART0.getType(), MIDI_CM5_UART0.getData1(), MIDI_CM5_UART0.getData2(), MIDI_CM5_UART0.getChannel()); // outbound to Controller
      BlinkLED(3);
    }
  }
}

// =======================================================

void MIDI_USB_DEV_Get() { // inbound from PC/MAC MIDI port
  // if (MIDI_USB_DEV.read()) {
  //   MIDI_PICO_CM5_UART1.send(MIDI_USB_DEV.getType(), MIDI_USB_DEV.getData1(), MIDI_USB_DEV.getData2(), MIDI_USB_DEV.getChannel()); // outbound to Controller
  //   BlinkLED(4);
  // }
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

  //MIDI_USB_SERIAL_DEV.sendSysEx(sizeof(SysExData), SysExData);
}

//====================================================================

void BlinkLED(uint8_t times) {

      for (auto k=0; k<times; ++k) {
        digitalWrite(PICO_DEFAULT_LED_GPIO, HIGH);
        delay(250);
        digitalWrite(PICO_DEFAULT_LED_GPIO, LOW);
        delay(50);
      }

} // end Blink LED