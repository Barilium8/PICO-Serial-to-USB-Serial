
#include <Arduino.h>
#include <pico/stdlib.h> // added to make UF2 button work, see SR1_BUTNUM_HELP
#include <pico/time.h>
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
#define MIDI_SYSEX_ARRAY_SIZE 255
#include <MIDI.h> // by Francois Best
#include <Adafruit_TinyUSB.h>

#define PICO_DEFAULT_LED_GPIO 25
#define HW_PROGRAM_BUTTON 26 // 26 new hw // was 2 // was 9

#define USB_BAUD 460'800
#define UART_BAUD 115'200 //460'800 // 1'000'000

// create USB_MIDI object
Adafruit_USBD_MIDI TinyUSB_MIDI; // declare a USB 'MIDI' port and wrap it in a MIDI intrerface
struct USBDev_Settings : public midi::DefaultSettings{
    static const bool UseRunningStatus = false;
    static const bool HandleNullVelocityNoteOnAsNoteOff = true;
    static const bool Use1ByteParsing = true;
    static const long BaudRate = USB_BAUD;
    static const unsigned SysExMaxSize = 255;
};
MIDI_CREATE_CUSTOM_INSTANCE(Adafruit_USBD_MIDI, TinyUSB_MIDI, MIDI_USB_DEV, USBDev_Settings);


// Creates MIDI on Serial1 aka UART0 (GPIO 0/1) connects to The WTS Controller (RP2040 on IO board)
struct MIDI_CNTLR_Settings : public midi::DefaultSettings{
  static const bool UseRunningStatus = false;
  static const bool HandleNullVelocityNoteOnAsNoteOff = true;
  static const bool Use1ByteParsing = true;
  static const long BaudRate = UART_BAUD; // tested at 2'000'000 and works
  static const unsigned SysExMaxSize = 255;
};
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial1, MIDI_PICO_UART0, MIDI_CNTLR_Settings);


// Creates MIDI on Serial2 aka UART1 (GPIO 4/5) connects to CM5 (RaspPi Compute Module)
struct MIDI_CM5_Settings : public midi::DefaultSettings{
  static const bool UseRunningStatus = false;
  static const bool HandleNullVelocityNoteOnAsNoteOff = true;
  static const bool Use1ByteParsing = true;
  static const long BaudRate = UART_BAUD;
  static const unsigned SysExMaxSize = 255;
};
MIDI_CREATE_CUSTOM_INSTANCE(HardwareSerial, Serial2, MIDI_CM5_UART1, MIDI_CM5_Settings);


uint8_t theMIDIChan = 16;

#define toggleCtrlrMode 14
#define toggleSynthMode 15

const uint blink_pattern_SendBoth[] = {1, 0, 1, 0, 1, 0, 1, 0, 1, 0};
const uint blink_pattern_SendPC[] =   {1, 0, 1, 0, 1, 0, 0, 0, 0, 0};
const uint blink_pattern_SendCM5[] =  {1, 1, 1, 1, 1, 1, 1, 1, 0, 0};
const uint blink_pattern_None[] =     {1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const uint blink_count = 10;
const uint interval_ms = 200;

volatile uint pattern_index = 0;



// function prototypes

//void HandleSysEx_MIDI_CM5_UART1(byte *data, unsigned int length);
//void HandleSysEx_MIDI_USB_DEV(byte *data, unsigned int length);

void GPIO_interrupt_handler(uint gpio, uint32_t events);

void MIDI_PICO_UART0_Get();
void MIDI_CM5_UART1_Get();
void MIDI_USB_DEV_Get();

void SendControllerInfo();
void BlinkLED(uint8_t times);
int64_t blinkCode_callback(alarm_id_t id, void *user_data);
void DrawMenu();
void EEPromUpdate(uint8_t addr, uint8_t val);
void PrintNotes();

bool gSend_PC_MIDI = true;
bool gPrevSend_PC_MIDI = true;
const uint8_t Addr_Send_PC_MIDI = 1;

bool gSend_CM5_MIDI = false;
bool gPrevSend_CM5_MIDI = false;
const uint8_t Addr_Send_CM5_MIDI = 2;

bool gSend_A_Note = false;
bool gPrevSend_A_Note = false;
const uint8_t Addr_Send_A_Note = 3;
bool gA_Note_Is_Playing = false;

bool gForm_Feed = false;
bool gPrevForm_Feed = false;
const uint8_t Addr_Form_Feed = 4;

String version = "1.5";
char formattedVer[46];

//==============================================================================================

void setup() {
  Serial.setStringDescriptor("DEBUG for SerialToUSBSerial");
  Serial.begin(USB_BAUD); // USB serial init w/defaults

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
  gSend_PC_MIDI = EEPROM.read(Addr_Send_PC_MIDI);
  gSend_CM5_MIDI = EEPROM.read(Addr_Send_CM5_MIDI);
  gSend_A_Note = EEPROM.read(Addr_Send_A_Note);
  gForm_Feed = EEPROM.read(Addr_Form_Feed);

  gPrevSend_PC_MIDI = gSend_PC_MIDI;
  gPrevSend_CM5_MIDI= gSend_CM5_MIDI;
  gPrevSend_A_Note = gSend_A_Note;
  gPrevForm_Feed = gForm_Feed;

  DrawMenu();
  PrintNotes();

} // end setup

//-----------------------------------------------

void setup1() {

  // !!!!! MAKE SURE to set '#define CFG_TUD_CDC 2' and '#define CFG_TUD_MIDI 2' in file tusb_config_rp2040.h in dir...  !!!!
  /* C:\Users\Steve\.platformio\packages\framework-arduinopico\libraries\Adafruit_TinyUSB_Arduino\src\arduino\ports\rp2040 */

  // Initialize Serial1 (UART0) for PICO-CM5 comms on GPIO8 (TX) and GPIO9 (RX)
  uart_init(uart0, UART_BAUD); // USB_BAUD tested at 2'000'000 and works
  gpio_set_function(0, GPIO_FUNC_UART);  // Set GPIO0 to UART function
  gpio_set_function(1, GPIO_FUNC_UART);  // Set GPIO1 to UART function

  // Initialize Serial2 (UART1) for PICO-CM5 comms on GPIO8 (TX) and GPIO9 (RX)
  uart_init(uart1, UART_BAUD); // USB_BAUD tested at 2'000'000 and works
  gpio_set_function(4, GPIO_FUNC_UART);  // Set GPIO8 to UART function
  gpio_set_function(5, GPIO_FUNC_UART);  // Set GPIO9 to UART function

  MIDI_PICO_UART0.begin(MIDI_CHANNEL_OMNI);
  MIDI_PICO_UART0.turnThruOff();

  MIDI_CM5_UART1.begin(MIDI_CHANNEL_OMNI);
  MIDI_CM5_UART1.turnThruOff();
  //MIDI_CM5_UART1.setHandleSystemExclusive(HandleSysEx_MIDI_CM5_UART1);

  TinyUSB_MIDI.setStringDescriptor("Wave Terrain Synth");
  MIDI_USB_DEV.begin(MIDI_CHANNEL_OMNI);
  MIDI_USB_DEV.turnThruOff();
  //MIDI_USB_DEV.setHandleSystemExclusive(HandleSysEx_MIDI_USB_DEV);

  //while( !TinyUSBDevice.mounted() ) { delay(1); }  // wait until device mounted

  delay (500);
  //Serial.println(" PICO Serial-to-USB-Serial Starting... ");

  gpio_init(toggleSynthMode);
  gpio_set_dir(toggleSynthMode, GPIO_IN);
  gpio_pull_down(toggleSynthMode);

  gpio_init(toggleCtrlrMode);
  gpio_set_dir(toggleCtrlrMode, GPIO_IN);
  gpio_pull_down(toggleCtrlrMode);

  gpio_set_irq_enabled_with_callback(toggleSynthMode, GPIO_IRQ_EDGE_RISE, true, &GPIO_interrupt_handler);
  gpio_set_irq_enabled_with_callback(toggleCtrlrMode, GPIO_IRQ_EDGE_RISE, true, &GPIO_interrupt_handler);

  // Start repeating alarm
  add_alarm_in_us(interval_ms * 1000, blinkCode_callback, NULL, true);

}  // end setup1

//-----------------------------------------------

void GPIO_interrupt_handler(uint gpio, uint32_t events) {
  Serial.println(String("In GPIO_interrupt_handler... gpio=") + gpio + " events=" + events);

  if (events & GPIO_IRQ_EDGE_RISE) {
    Serial.println(String("    In GPIO_IRQ_EDGE_FALL..."));
    switch(gpio) {
      case toggleSynthMode:
          gPrevSend_CM5_MIDI = gSend_CM5_MIDI;
          gSend_CM5_MIDI = !gSend_CM5_MIDI;
          EEPromUpdate(Addr_Send_CM5_MIDI, gSend_CM5_MIDI);
          DrawMenu();
          Serial.println("  ...GPIO toggled Send CM5 MIDI");
      break;

      case toggleCtrlrMode:
          gPrevSend_PC_MIDI = gSend_PC_MIDI;
          gSend_PC_MIDI = !gSend_PC_MIDI;
          EEPromUpdate(Addr_Send_PC_MIDI, gSend_PC_MIDI);
          DrawMenu();
          Serial.println("  ...GPIO toggled Send PC MIDI");
      break;
    }
  }
}

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

  // Blink LED
  // if ( (now - prevTime2) > 2000) {
  //   prevTime2 = millis();
  //   BlinkLED(1);
  // }

  static uint8_t tenTimes = 0;
  // Read serial inbound data for menu
  if ( (now - prevTime3) > 500) {
    prevTime3 = millis();

    String serialData = Serial.readString();
    if (serialData == "m" || serialData == "M") {
      DrawMenu();
    }
    else if (serialData == "1") {
      gPrevSend_PC_MIDI = gSend_PC_MIDI;
      gSend_PC_MIDI = !gSend_PC_MIDI;
      EEPromUpdate(Addr_Send_PC_MIDI, gSend_PC_MIDI);
      DrawMenu();
      Serial.println("  ...toggled Send PC MIDI");
    }
    else if (serialData == "2") {
      gPrevSend_CM5_MIDI = gSend_CM5_MIDI;
      gSend_CM5_MIDI = !gSend_CM5_MIDI;
      EEPromUpdate(Addr_Send_CM5_MIDI, gSend_CM5_MIDI);
      DrawMenu();
      Serial.println("  ...toggled Send CM5 MIDI");
    }
    else if (serialData == "3") {
      gPrevSend_A_Note = gSend_A_Note;
      gSend_A_Note = !gSend_A_Note;
      if (gSend_A_Note == false) {
        MIDI_CM5_UART1.send(midi::NoteOff, 60, 64, 1);
        MIDI_USB_DEV.send(midi::NoteOff, 60, 64, 1);
      }
      EEPromUpdate(Addr_Send_A_Note, gSend_A_Note);
      DrawMenu();
      Serial.println("  ...toggled Send A Note");
    }
    else if (serialData == "f" || serialData == "F") {
      gPrevForm_Feed = gForm_Feed;
      gForm_Feed = !gForm_Feed;
      EEPromUpdate(Addr_Form_Feed, gForm_Feed);
      DrawMenu();
      Serial.println("  ...toggled Form Feed");
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
  static uint32_t prevTime2 = 0;

  // Read All MIDI ports
  if ( (micros() - prevTime1) > 800) { // 1000us 1ms
    prevTime1 = micros();
    MIDI_PICO_UART0_Get();     // read from WTS Controller (UART0/Serial1 port)
    delayMicroseconds(50);

    MIDI_CM5_UART1_Get();      // read from CM5 Compute Module (UART1/Serial2 port)
    delayMicroseconds(50);

    MIDI_USB_DEV_Get();        // read from PC/MAC MIDI (via USB)
    delayMicroseconds(50);
  }

  if ((millis() - prevTime2) > 1000) {
    prevTime2 = millis();
    if (gForm_Feed) DrawMenu();
    if (gSend_A_Note) {
      if (gA_Note_Is_Playing) {
        MIDI_CM5_UART1.send(midi::NoteOff, 60, 64, 1);
        MIDI_USB_DEV.send(midi::NoteOff, 60, 64, 1);
        Serial.println("Note OFFFFF");
      }
      else {
        if(gSend_CM5_MIDI) { MIDI_CM5_UART1.send(midi::NoteOn, 60, 127, 1); }
        if(gSend_PC_MIDI) { MIDI_USB_DEV.send(midi::NoteOn, 60, 127, 1); }
        Serial.println("Note ON");
      }
      gA_Note_Is_Playing = !gA_Note_Is_Playing;
    }
  }

} // end loop1

// =======================================================

// void HandleSysEx_MIDI_CM5_UART1(byte *data, unsigned int length) {
// //void HandleSysEx(const byte *data, unsigned int length) {
//   //Serial.println(String("SysEx Message: length... ") + length +" data: " + data[0] + " " + data[1] + " " + data[2] + " "+ data[3] + " "+ data[4] + " "+ data[5] + " "+ data[6] + " "+ data[7] + " " + data[8] + " " + data[9]);
//   // Serial.print(String("Recieved SysEx Message from WTS Synth Engine...  len=") + length + " ");

//   //Serial.println(String("Fr PC/MAC (MIDI USB) to Ctrlr (UART0) - msg Fr BRIDGE ") + MIDI_CM5_UART1.getType() + " " + MIDI_CM5_UART1.getData1() + " " + MIDI_CM5_UART1.getData2() + " " + MIDI_CM5_UART1.getChannel());
//   Serial.print(String("In SysEx handler from MIDI_CM5_UART1... msg ="));
//   for (auto j=0; j<length; ++j) {
//     Serial.print(String(" ") + data[j]);
//   }
//   Serial.println();
//   MIDI_PICO_UART0.sendSysEx(length, data, true);        // outbound to PICO MIDI port

// } // end fcn HandleSysEx

// // =======================================================

// void HandleSysEx_MIDI_USB_DEV(byte *data, unsigned int length) {
//   //void HandleSysEx(const byte *data, unsigned int length) {
//     //Serial.println(String("SysEx Message: length... ") + length +" data: " + data[0] + " " + data[1] + " " + data[2] + " "+ data[3] + " "+ data[4] + " "+ data[5] + " "+ data[6] + " "+ data[7] + " " + data[8] + " " + data[9]);
//     // Serial.print(String("Recieved SysEx Message from WTS Synth Engine...  len=") + length + " ");

//     //Serial.println(String("Fr PC/MAC (MIDI USB) to Ctrlr (UART0) - msg Fr BRIDGE ") + MIDI_CM5_UART1.getType() + " " + MIDI_CM5_UART1.getData1() + " " + MIDI_CM5_UART1.getData2() + " " + MIDI_CM5_UART1.getChannel());
//     Serial.print(String("In SysEx from handler MIDI_USB_DEV... msg ="));
//     for (auto j=0; j<length; ++j) {
//       Serial.print(String(" ") + data[j]);
//     }
//     Serial.println();
//     MIDI_PICO_UART0.sendSysEx(length, data, true);        // outbound to PICO MIDI port

//   } // end fcn HandleSysEx

// =======================================================

void MIDI_PICO_UART0_Get() { // inbound from PICO Controller (via UART0)
    //if (MIDI_PICO_UART0.read()) {
    while (MIDI_PICO_UART0.read()) {
      if (gSend_CM5_MIDI)  {
        MIDI_CM5_UART1.send(MIDI_PICO_UART0.getType(), MIDI_PICO_UART0.getData1(), MIDI_PICO_UART0.getData2(), MIDI_PICO_UART0.getChannel());      // outbound to PC/MAC Serial port
        Serial.println(String("Fr Ctrlr to CM5 - msg Fr BRIDGE ") + MIDI_PICO_UART0.getType() + " " + MIDI_PICO_UART0.getData1() + " " + MIDI_PICO_UART0.getData2() + " " + MIDI_PICO_UART0.getChannel());
        }

      if (gSend_PC_MIDI) {
        MIDI_USB_DEV.send(MIDI_PICO_UART0.getType(), MIDI_PICO_UART0.getData1(), MIDI_PICO_UART0.getData2(), MIDI_PICO_UART0.getChannel());        // outbound to PC/MAC MIDI port
        Serial.println(String("Fr Ctrlr to PC/MAC - msg Fr BRIDGE ") + MIDI_PICO_UART0.getType() + " " + MIDI_PICO_UART0.getData1() + " " + MIDI_PICO_UART0.getData2() + " " + MIDI_PICO_UART0.getChannel());
      }
    }
}

// =======================================================

void MIDI_CM5_UART1_Get() { // inbound from CM5 (via UART1)
    //if (MIDI_CM5_UART1.read()) {
    while (MIDI_CM5_UART1.read()) {
      Serial.println(String("Fr CM5 to Ctrlr - msg Fr BRIDGE ") + MIDI_CM5_UART1.getType() + " " + MIDI_CM5_UART1.getData1() + " " + MIDI_CM5_UART1.getData2() + " " + MIDI_CM5_UART1.getChannel());

      if ( MIDI_CM5_UART1.getType() == midi::SystemExclusive ) {
        Serial.print("  ...inbound SYSEX data from MIDI_CM5_UART1  msg:");
          for (auto j=0; j<MIDI_CM5_UART1.getSysExArrayLength(); ++j) {
            Serial.print(String(" ") + MIDI_CM5_UART1.getSysExArray()[j]);
          }
          Serial.println();
        MIDI_PICO_UART0.sendSysEx (MIDI_CM5_UART1.getSysExArrayLength(), MIDI_CM5_UART1.getSysExArray(), true);
        delayMicroseconds(50);
      }
      else
      {
        MIDI_PICO_UART0.send(MIDI_CM5_UART1.getType(), MIDI_CM5_UART1.getData1(), MIDI_CM5_UART1.getData2(), MIDI_CM5_UART1.getChannel()); // outbound to PICO Controller
      }

    }
}

// =======================================================

void MIDI_USB_DEV_Get() { // inbound MIDI from PC/MAC (via USB)
    //if (MIDI_USB_DEV.read()) {
    while (MIDI_USB_DEV.read()) {
      Serial.println(String("Fr PC/MAC to Ctrlr - msg Fr BRIDGE ") + MIDI_CM5_UART1.getType() + " " + MIDI_CM5_UART1.getData1() + " " + MIDI_CM5_UART1.getData2() + " " + MIDI_CM5_UART1.getChannel());

      if ( MIDI_USB_DEV.getType() == midi::SystemExclusive ) {
        Serial.print("  ...inbound SYSEX data from MIDI_USB_DEV  msg:");
          for (auto j=0; j<MIDI_USB_DEV.getSysExArrayLength(); ++j) {
            Serial.print(String(" ") + MIDI_USB_DEV.getSysExArray()[j]);
          }
          Serial.println();
        MIDI_PICO_UART0.sendSysEx (MIDI_USB_DEV.getSysExArrayLength(), MIDI_USB_DEV.getSysExArray(), true);
        delayMicroseconds(50);
      }
      else
      {
        MIDI_PICO_UART0.send(MIDI_USB_DEV.getType(), MIDI_USB_DEV.getData1(), MIDI_USB_DEV.getData2(), MIDI_USB_DEV.getChannel()); // outbound to Controller
      }

    }
}

//====================================================================

void BlinkLED(uint8_t times) {

    if (gSend_PC_MIDI && gSend_CM5_MIDI) {
      for (auto k=0; k<times*2; ++k) {
        digitalWrite(PICO_DEFAULT_LED_GPIO, HIGH);
        delay(125);
        digitalWrite(PICO_DEFAULT_LED_GPIO, LOW);
        delay(30);
        digitalWrite(PICO_DEFAULT_LED_GPIO, HIGH);
        delay(125);
        digitalWrite(PICO_DEFAULT_LED_GPIO, LOW);
        delay(30);
        digitalWrite(PICO_DEFAULT_LED_GPIO, HIGH);
        delay(125);
        digitalWrite(PICO_DEFAULT_LED_GPIO, LOW);
        delay(30);
        digitalWrite(PICO_DEFAULT_LED_GPIO, HIGH);
        delay(125);
        digitalWrite(PICO_DEFAULT_LED_GPIO, LOW);
        delay(30);
      }
    }

    else if (gSend_PC_MIDI && !gSend_CM5_MIDI) {
      for (auto k=0; k<times; ++k) {
        digitalWrite(PICO_DEFAULT_LED_GPIO, HIGH);
        delay(125);
        digitalWrite(PICO_DEFAULT_LED_GPIO, LOW);
        delay(30);
        digitalWrite(PICO_DEFAULT_LED_GPIO, HIGH);
        delay(125);
        digitalWrite(PICO_DEFAULT_LED_GPIO, LOW);
        delay(30);
      }
    }

    else if (!gSend_PC_MIDI && gSend_CM5_MIDI) {
      for (auto k=0; k<times; ++k) {
        digitalWrite(PICO_DEFAULT_LED_GPIO, HIGH);
        delay(550);
        digitalWrite(PICO_DEFAULT_LED_GPIO, LOW);
        delay(50);
      }
    }

    else {
      for (auto k=0; k<times; ++k) {
        digitalWrite(PICO_DEFAULT_LED_GPIO, HIGH);
        delay(70);
        digitalWrite(PICO_DEFAULT_LED_GPIO, LOW);
        delay(50);
      }
    }

} // end Blink LED

//====================================================================

int64_t blinkCode_callback(alarm_id_t id, void *user_data) {

    if (gSend_PC_MIDI && gSend_CM5_MIDI) {
      gpio_put(PICO_DEFAULT_LED_GPIO, blink_pattern_SendBoth[pattern_index]);
    }

    else if (gSend_PC_MIDI && !gSend_CM5_MIDI) {
      gpio_put(PICO_DEFAULT_LED_GPIO, blink_pattern_SendPC[pattern_index]);
    }

    else if (!gSend_PC_MIDI && gSend_CM5_MIDI) {
      gpio_put(PICO_DEFAULT_LED_GPIO, blink_pattern_SendCM5[pattern_index]);
    }

    else {
      gpio_put(PICO_DEFAULT_LED_GPIO, blink_pattern_None[pattern_index]);
    }

    pattern_index = (pattern_index + 1) % blink_count;
    return interval_ms * 1000; // reschedule in microseconds
}

//====================================================================

void DrawMenu() {

  Serial.print('\f'); // Clears screen
  Serial.print("  WTS DEBUGGER (PICO) ");
  Serial.println(formattedVer);
  Serial.println();
    Serial.println("  o--------------------------------------------------o");
    Serial.println("  |                                                  |");

  if (gSend_PC_MIDI) {
    Serial.println("  |  1 = Controller Mode MIDI (to PC/MAC)   ( on  )  |");
  }
  else{
    Serial.println("  |  1 = Controller Mode MIDI (to PC/MAC)   ( off )  |");
  }
    Serial.println("  |                                                  |");

  if (gSend_CM5_MIDI) {
    Serial.println("  |  2 = Synth Mode MIDI      (to CM5)      ( on  )  |");
  }
  else{
    Serial.println("  |  2 = Synth Mode MIDI      (to CM5)      ( off )  |");
  }

    Serial.println("  |                                                  |");

  if (gSend_A_Note) {
    Serial.println("  |  3 = Send MIDI Notes      (PC/MAC/CM5)  ( on  )  |");
  }
  else{
    Serial.println("  |  3 = Send MIDI Notes      (PC/MAC/CM5)  ( off )  |");
  }

    Serial.println("  |                                                  |");
  if (gForm_Feed) {
    Serial.println("  |  f = form feed                          ( on  )  |");
  }
  else{
    Serial.println("  |  f = form feed                          ( off )  |");
  }
    Serial.println("  |  m = this menu                                   |");
    Serial.println("  o--------------------------------------------------o");

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
    Serial.println("    PICO: GND   - GND        (Pin6) to WTS IO Board JP30 Pin11");
}

//====================================================================