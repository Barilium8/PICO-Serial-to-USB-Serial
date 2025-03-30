
#include <Arduino.h>
#include <pico/stdlib.h> // added to make UF2 button work, see SR1_BUTNUM_HELP
#include <pico/bootrom.h> // added to make UF2 button work, see SR1_BUTNUM_HELP
#include <EEPROM.h>

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

#define ALL_BAUD 460'800

// create USB_SERIAL (CDC) object
Adafruit_USBD_CDC TinyUSB_SerialMIDI; // declare a USB 'Serial' port and wrap it in a MIDI intrerface
struct USB_SM_Dev_Settings : public midi::DefaultSettings{
    static const bool UseRunningStatus = false;
    static const bool HandleNullVelocityNoteOnAsNoteOff = true;
    static const bool Use1ByteParsing = true;
    static const long BaudRate = ALL_BAUD; //460'800; // 31250;
    static const unsigned SysExMaxSize = 64;
};
MIDI_CREATE_CUSTOM_INSTANCE(Adafruit_USBD_CDC, TinyUSB_SerialMIDI, MIDI_USB_SERIAL_DEV, USB_SM_Dev_Settings);

// create USB_MIDI object
Adafruit_USBD_MIDI TinyUSB_MIDI; // declare a USB 'MIDI' port and wrap it in a MIDI intrerface
struct USBDev_Settings : public midi::DefaultSettings{
    static const bool UseRunningStatus = false;
    static const bool HandleNullVelocityNoteOnAsNoteOff = true;
    static const bool Use1ByteParsing = true;
    static const long BaudRate = ALL_BAUD; // 460'800; // 31250;
    static const unsigned SysExMaxSize = 64;
};
MIDI_CREATE_CUSTOM_INSTANCE(Adafruit_USBD_MIDI, TinyUSB_MIDI, MIDI_USB_DEV, USBDev_Settings);


// Creates MIDI on Serial1 aka UART0 (GPIO 0/1) connects to The WTS Controller (RP2040 on IO board)
struct MIDI_CNTLR_Settings : public midi::DefaultSettings{
  static const bool UseRunningStatus = false;
  static const bool HandleNullVelocityNoteOnAsNoteOff = true;
  static const bool Use1ByteParsing = true;
  static const long BaudRate = ALL_BAUD; // tested at 2'000'000 and works
  static const unsigned SysExMaxSize = 64;
};
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial1, MIDI_PICO_UART0, MIDI_CNTLR_Settings);


// Creates MIDI on Serial2 aka UART1 (GPIO 4/5) connects to CM5 (RaspPi Compute Module)
struct MIDI_CM5_Settings : public midi::DefaultSettings{
  static const bool UseRunningStatus = false;
  static const bool HandleNullVelocityNoteOnAsNoteOff = true;
  static const bool Use1ByteParsing = true;
  static const long BaudRate = ALL_BAUD;
  static const unsigned SysExMaxSize = 64;
};
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial2, MIDI_CM5_UART1, MIDI_CM5_Settings);


uint8_t theMIDIChan = 16;


// function prototypes
void MIDI_PICO_UART0_Get();
void MIDI_CM5_UART1_Get();
void MIDI_USB_SERIAL_DEV_Get();
void MIDI_USB_DEV_Get();

void SendControllerInfo();
void BlinkLED(uint8_t times);
void DrawMenu();
void EEPromUpdate(uint8_t addr, uint8_t val);
void PrintNotes();

bool gSend_PC_MIDI_As_MIDI = true;
bool gPrevSend_PC_MIDI_As_MIDI = true;
const uint8_t Addr_Send_PC_MIDI_As_MIDI = 1;

bool gSend_PC_MIDI_As_SERIAL = true;
bool gPrevSend_PC_MIDI_As_SERIAL = true;
const uint8_t Addr_Send_PC_MIDI_As_SERIAL = 2;

bool gSend_CM5_MIDI_As_SERIAL = false;
bool gPrevSend_CM5_MIDI_As_SERIAL = false;
const uint8_t Addr_Send_CM5_MIDI_As_SERIAL = 3;

String version = "1.2";
char formattedVer[46];

//==============================================================================================

void setup() {
  Serial.begin(115'200); // USB serial init w/defaults
  Serial.setStringDescriptor("DEBUG for SerialToUSBSerial");

  pinMode(PICO_DEFAULT_LED_GPIO, OUTPUT);

  // gpio_init(HW_PROGRAM_BUTTON);
  // gpio_set_function(HW_PROGRAM_BUTTON, GPIO_FUNC_SIO);
  // gpio_set_dir(HW_PROGRAM_BUTTON, GPIO_IN);
  // pinMode(HW_PROGRAM_BUTTON, INPUT_PULLUP);

  //========= To Light WS2812 via PIO ============================

  // stdio_init_all();

  // PIO pio = pio0;
  // int sm = 0;
  // uint offset = pio_add_program(pio, &ws2812_program);
  // uint8_t cnt = 0;

  // puts("RP2040-Zero WS2812 Test");

  // ws2812_program_init(pio, sm, offset, 16, 800000, true);

  //====================================

  delay (1000);

  // makes 45 wide string padded
  //sprintf(formattedVer, " Ver %-40s", version.c_str());
  sprintf(formattedVer, " ver %-20s", version.c_str());

  #define EEPROM_SIZE 256
  EEPROM.begin(EEPROM_SIZE);
  gSend_PC_MIDI_As_MIDI = EEPROM.read(Addr_Send_PC_MIDI_As_MIDI);
  gSend_PC_MIDI_As_SERIAL = EEPROM.read(Addr_Send_PC_MIDI_As_SERIAL);
  gSend_CM5_MIDI_As_SERIAL = EEPROM.read(Addr_Send_CM5_MIDI_As_SERIAL);

  gPrevSend_PC_MIDI_As_MIDI = gSend_PC_MIDI_As_MIDI;
  gPrevSend_PC_MIDI_As_SERIAL = gSend_PC_MIDI_As_SERIAL;
  gPrevSend_CM5_MIDI_As_SERIAL= gSend_CM5_MIDI_As_SERIAL;

  DrawMenu();
  PrintNotes();

} // end setup

//-----------------------------------------------

void setup1() {

  // !!!!! MAKE SURE to set '#define CFG_TUD_CDC 2' and '#define CFG_TUD_MIDI 2' in file tusb_config_rp2040.h in dir...  !!!!
  /* C:\Users\Steve\.platformio\packages\framework-arduinopico\libraries\Adafruit_TinyUSB_Arduino\src\arduino\ports\rp2040 */

  // Initialize Serial2 (UART1) for PICO-CM5 comms on GPIO8 (TX) and GPIO9 (RX)
  uart_init(uart0, ALL_BAUD); // tested at 2'000'000 and works
  gpio_set_function(0, GPIO_FUNC_UART);  // Set GPIO0 to UART function
  gpio_set_function(1, GPIO_FUNC_UART);  // Set GPIO1 to UART function

  // Initialize Serial2 (UART1) for PICO-CM5 comms on GPIO8 (TX) and GPIO9 (RX)
  uart_init(uart1, ALL_BAUD); // tested at 2'000'000 and works
  gpio_set_function(4, GPIO_FUNC_UART);  // Set GPIO8 to UART function
  gpio_set_function(5, GPIO_FUNC_UART);  // Set GPIO9 to UART function

  MIDI_PICO_UART0.begin(MIDI_CHANNEL_OMNI);
  MIDI_PICO_UART0.turnThruOff();

  TinyUSB_SerialMIDI.setStringDescriptor("WTS Serial MIDI");
  MIDI_USB_SERIAL_DEV.begin(MIDI_CHANNEL_OMNI);
  MIDI_USB_SERIAL_DEV.turnThruOff();

  TinyUSB_MIDI.setStringDescriptor("Wave Terrain Synth");
  MIDI_USB_DEV.begin(MIDI_CHANNEL_OMNI);
  MIDI_USB_DEV.turnThruOff();

  while( !TinyUSBDevice.mounted() ) { delay(1); }  // wait until device mounted


  delay (500);
  //Serial.println(" PICO Serial-to-USB-Serial Starting... ");


}  // end setup1

//-----------------------------------------------

void loop() {
  static uint32_t prevTime1 = 0;
  static uint32_t prevTime2 = 0;
  static uint32_t prevTime3 = 0;

  // if ( (micros() - prevTime1) > 800) {
  //   prevTime1 = micros();
  //   if (!gpio_get(HW_PROGRAM_BUTTON)) {
  //     Serial.println(" Going into Mass Storage boot mode...");
  //     reset_usb_boot(0,0);
  //   }
  // }

  auto now = millis();

  // Read all the inbound shift reg data for buttons and encoders
  if ( (now - prevTime2) > 5000) {
    prevTime2 = millis();
    BlinkLED(1);
  }

  static uint8_t tenTimes = 0;
  // Read all the inbound shift reg data for buttons and encoders
  if ( (now - prevTime3) > 500) {
    prevTime3 = millis();

    // TinyUSB_SerialMIDI.println("Serial - TinyUSB_SerialMIDI");
    // MIDI_USB_SERIAL_DEV.sendNoteOn(60,127,16);
    // MIDI_USB_SERIAL_DEV.sendNoteOff(60,127,16);
    // TinyUSB_SerialMIDI.println();
    String serialData = Serial.readString();
    if (serialData == "M" || serialData == "m") {
      DrawMenu();
    }
    else if (serialData == "1") {
      gPrevSend_PC_MIDI_As_MIDI = gSend_PC_MIDI_As_MIDI;
      gSend_PC_MIDI_As_MIDI = !gSend_PC_MIDI_As_MIDI;
      EEPromUpdate(Addr_Send_PC_MIDI_As_MIDI, gSend_PC_MIDI_As_MIDI);
      DrawMenu();
      Serial.println("  ...toggled Send PC MIDI As MIDI");
    }
    else if (serialData == "2") {
      gPrevSend_PC_MIDI_As_SERIAL = gSend_PC_MIDI_As_SERIAL;
      gSend_PC_MIDI_As_SERIAL = !gSend_PC_MIDI_As_SERIAL;
      EEPromUpdate(Addr_Send_PC_MIDI_As_SERIAL, gSend_PC_MIDI_As_SERIAL);
      DrawMenu();
      Serial.println("  ...toggled Send PC MIDI As Serial Data");
    }
    else if (serialData == "3") {
      gPrevSend_CM5_MIDI_As_SERIAL = gSend_CM5_MIDI_As_SERIAL;
      gSend_CM5_MIDI_As_SERIAL = !gSend_CM5_MIDI_As_SERIAL;
      EEPromUpdate(Addr_Send_CM5_MIDI_As_SERIAL, gSend_CM5_MIDI_As_SERIAL);
      DrawMenu();
      Serial.println("  ...toggled Send CM5 MIDI As Serial Data");
    }
  }

    //=========== To Light WS2812 via PIO =========

      // for (cnt = 0; cnt < 0xff; cnt++)
      // {
      //     put_rgb(cnt, 0xff - cnt, 0);
      //     sleep_ms(3);
      // }
      // for (cnt = 0; cnt < 0xff; cnt++)
      // {
      //     put_rgb(0xff - cnt, 0, cnt);
      //     sleep_ms(3);
      // }
      // for (cnt = 0; cnt < 0xff; cnt++)
      // {
      //     put_rgb(0, cnt, 0xff - cnt);
      //     sleep_ms(3);
      // }
    //============================================

} // end loop

//-----------------------------------------------

void loop1() {
  static uint32_t prevTime1 = 0;

  // Read All MIDI ports
  if ( (micros() - prevTime1) > 800) { // 1000us 1ms
    prevTime1 = micros();
    MIDI_PICO_UART0_Get();     // read from WTS Controller (UART0/Serial1 port)
    MIDI_CM5_UART1_Get();      // read from CM5 Compute Module (UART1/Serial2 port)
    MIDI_USB_SERIAL_DEV_Get(); // read from PC/MAC MIDI (as Serial Data) (via USB)
    MIDI_USB_DEV_Get();        // read from PC/MAC MIDI (via USB)
  }

} // end loop1

// =======================================================

void HandleSysEx(byte *data, unsigned int length) {
  Serial.println(String("SysEx Message: 1st 7 byts... ") + data[0] + " " + data[1] + " " + data[2] + " "+ data[3] + " "+ data[4] + " "+ data[5] + " "+ data[6] + " "+ data[7]);

} // end fcn HandleSysEx

// =======================================================

void MIDI_PICO_UART0_Get() { // inbound from PICO Controller (via UART0)
    if (MIDI_PICO_UART0.read()) {
      if (gSend_CM5_MIDI_As_SERIAL)  MIDI_CM5_UART1.send(MIDI_PICO_UART0.getType(), MIDI_PICO_UART0.getData1(), MIDI_PICO_UART0.getData2(), MIDI_PICO_UART0.getChannel());      // outbound to PC/MAC Serial port
      if (gSend_PC_MIDI_As_SERIAL)   MIDI_USB_SERIAL_DEV.send(MIDI_PICO_UART0.getType(), MIDI_PICO_UART0.getData1(), MIDI_PICO_UART0.getData2(), MIDI_PICO_UART0.getChannel()); // outbound to PC/MAC Serial port
      if (gSend_PC_MIDI_As_MIDI)     MIDI_USB_DEV.send(MIDI_PICO_UART0.getType(), MIDI_PICO_UART0.getData1(), MIDI_PICO_UART0.getData2(), MIDI_PICO_UART0.getChannel());        // outbound to PC/MAC MIDI port
    }
}

// =======================================================

void MIDI_CM5_UART1_Get() { // inbound from CM5 (via UART1)
    if (MIDI_CM5_UART1.read() && gSend_CM5_MIDI_As_SERIAL) {
      MIDI_PICO_UART0.send(MIDI_CM5_UART1.getType(), MIDI_CM5_UART1.getData1(), MIDI_CM5_UART1.getData2(), MIDI_CM5_UART1.getChannel()); // outbound to PICO Controller
    }
}

// =======================================================

void MIDI_USB_SERIAL_DEV_Get() { // inbound MIDI (as Serial Data) from PC/MAC (via USB)
    if (MIDI_USB_SERIAL_DEV.read() && gSend_PC_MIDI_As_SERIAL) {
      MIDI_PICO_UART0.send(MIDI_USB_SERIAL_DEV.getType(), MIDI_USB_SERIAL_DEV.getData1(), MIDI_USB_SERIAL_DEV.getData2(), MIDI_USB_SERIAL_DEV.getChannel()); // outbound to Controller
    }
}

// =======================================================

void MIDI_USB_DEV_Get() { // inbound MIDI from PC/MAC (via USB)
    if (MIDI_USB_DEV.read() && gSend_PC_MIDI_As_MIDI) {
      MIDI_PICO_UART0.send(MIDI_USB_DEV.getType(), MIDI_USB_DEV.getData1(), MIDI_USB_DEV.getData2(), MIDI_USB_DEV.getChannel()); // outbound to Controller
    }
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

//====================================================================

void DrawMenu() {

  Serial.print('\f'); // Clears screen
  Serial.print("  WTS DEBUGGER (PICO) ");
  Serial.println(formattedVer);
  Serial.println();
  Serial.println("  o---------------------------------------------o");
  Serial.println("  |                                             |");

  if (gSend_PC_MIDI_As_MIDI) {
    Serial.println("  |  1 = Send PC MIDI as MIDI (is on)           |");
  }
  else{
    Serial.println("  |  1 = Send PC MIDI as MIDI (is off)          |");
  }

  if (gSend_PC_MIDI_As_SERIAL) {
    Serial.println("  |  2 = Send PC MIDI as Serial Data (is on)    |");
  }
  else{
    Serial.println("  |  2 = Send PC MIDI as Serial Data (is off)   |");
  }

  Serial.println("  |                                             |");

  if (gSend_CM5_MIDI_As_SERIAL) {
    Serial.println("  |  3 = Send CM5 MIDI as Serial Data (is on)   |");
  }
  else{
    Serial.println("  |  3 = Send CM5 MIDI as Serial Data (is off)  |");
  }

  Serial.println("  |                                             |");
  Serial.println("  |  m = this menu                              |");
  Serial.println("  |                                             |");
  Serial.println("  o---------------------------------------------o");

}

//====================================================================

void EEPromUpdate(uint8_t addr, uint8_t val) {
  if (addr >= EEPROM_SIZE) {
    return;
  }

  uint8_t tempVal = EEPROM.read(addr);
  if (tempVal != val) {
    EEPROM.write(addr, val);
    EEPROM.commit();
  }

}

//====================================================================

void PrintNotes() {
    Serial.println("");
    Serial.println("");
    Serial.println("  NOTES: This is a RS232 Serial to USB bridge and loopback debugger)");
    Serial.println("    It inserts between the serial ports of the WTS Controller PICO and WTS CM5,");
    Serial.println("    so that MIDI traffic can be intercepted and rerouted to a PC,");
    Serial.println("    as well as looped back for 'normmal' WTS Synth mode.");
    Serial.println("    It can (full duplex) send MIDI and/or Serial MIDI data between WTS Controller PICO and WTS CM5");
    Serial.println("    This is code for a PICO 'RP2040 ZERO', although can be used on any PICO (pin#'s may differ)");
    Serial.println("");
    Serial.println("  Pin Assignments:");
    Serial.println("    PICO: GPIO0 - UART0 'TX' (Pin0) to WTS IO Board JP30 Pin4");
    Serial.println("    PICO: GPIO1 - UART0 'RX' (Pin1) to WTS IO Board JP30 Pin6");
    Serial.println("    PICO: GPIO4 - UART1 'TX' (Pin4) to WTS IO Board JP30 Pin3");
    Serial.println("    PICO: GPIO5 - UART1 'RX' (Pin5) to WTS IO Board JP30 Pin5");
}

//====================================================================