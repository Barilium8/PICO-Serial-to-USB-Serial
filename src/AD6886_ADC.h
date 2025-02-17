// file: AD6886_ADC.h
// brief: configure and recieve data from the TI AD6886 Aanlog to Digital Converter
// authors: Steve Barile & Grayson Salaski
// date: Oct 31 2024

#pragma once

#include <Arduino.h>
#include <stdio.h>
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "Globals.h"

// SPI on PICO video:  https://www.youtube.com/watch?v=s7Lud1Gqrqw
// the source from the video sample: https://learnembeddedsystems.co.uk/bmp280-and-pi-pico-over-spi

// range = vref (typically 4.096v) x ZERO_nnn or BI_nnn
// const values found in TI AD8668 ADC Tech Doc - page 26
// https://www.ti.com/lit/ds/symlink/ads8668.pdf?ts=1729897803551&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FADS8668%253Futm_source%253Dgoogle%2526utm_medium%253Dcpc%2526utm_campaign%253Dasc-null-null-GPN_EN-cpc-pf-google-wwe%2526utm_content%253DADS8668%2526ds_k%253DADS8668%2526DCM%253Dyes%2526gad_source%253D1%2526gclid%253DCjwKCAjwg-24BhB_EiwA1ZOx8g4HdZvZKLqXv_SLApFszk2GzOx393IIx59OmWJ6AB8OxUZ-_mU9FhoCWcAQAvD_BwE%2526gclsrc%253Daw.ds


 // TI AD8668 ADC SPI info
 // https://www.ti.com/lit/ds/symlink/ads8668.pdf?ts=1729897803551&ref_url=https%253A%252F%252Fwww.ti.com%252Fproduct%252FADS8668%253Futm_source%253Dgoogle%2526utm_medium%253Dcpc%2526utm_campaign%253Dasc-null-null-GPN_EN-cpc-pf-google-wwe%2526utm_content%253DADS8668%2526ds_k%253DADS8668%2526DCM%253Dyes%2526gad_source%253D1%2526gclid%253DCjwKCAjwg-24BhB_EiwA1ZOx8g4HdZvZKLqXv_SLApFszk2GzOx393IIx59OmWJ6AB8OxUZ-_mU9FhoCWcAQAvD_BwE%2526gclsrc%253Daw.ds
 // page 26

/*** ADC address ****/
#define AD6886_NO_OP     0x0000  // Continue operation in previous mode
#define AD6886_STDBY     0x8200  // Device is placed into standby mode
#define AD6886_PWR_DN    0x8300  // Device is powered down
#define AD6886_RST       0x8500  // Program register is reset to default
#define AD6886_AUTO_RST  0xA000  // Auto mode enabled following a reset
#define AD6886_MAN_Ch_0  0xC000  // Channel 0 input is selected
#define AD6886_MAN_Ch_1  0xC400  // Channel 1 input is selected
#define AD6886_MAN_Ch_2  0xC800  // Channel 2 input is selected
#define AD6886_MAN_Ch_3  0xCC00  // Channel 3 input is selected
#define AD6886_MAN_Ch_4  0xD000  // Channel 4 input is selected
#define AD6886_MAN_Ch_5  0xD400  // Channel 5 input is selected
#define AD6886_MAN_Ch_6  0xD800  // Channel 6 input is selected
#define AD6886_MAN_Ch_7  0xDC00  // Channel 7 input is selected
#define AD6886_MAN_AUX   0xE000  // AUX channel input is selected

// RANGE SELECT REGISTERS
#define AD6886_RG_Ch_0       0x05   // Channel 0 Input Range: default 0x00 - bit 3-0 to select range
#define AD6886_RG_Ch_1       0x06   // Channel 1 Input Range: default 0x00 - bit 3-0 to select range
#define AD6886_RG_Ch_2       0x07   // Channel 2 Input Range: default 0x00 - bit 3-0 to select range
#define AD6886_RG_Ch_3       0x08   // Channel 3 Input Range: default 0x00 - bit 3-0 to select range
#define AD6886_RG_Ch_4       0x09   // Channel 4 Input Range: default 0x00 - bit 3-0 to select range
#define AD6886_RG_Ch_5       0x0A   // Channel 5 Input Range: default 0x00 - bit 3-0 to select range
#define AD6886_RG_Ch_6       0x0B   // Channel 6 Input Range: default 0x00 - bit 3-0 to select range
#define AD6886_RG_Ch_7       0x0C   // Channel 7 Input Range: default 0x00 - bit 3-0 to select range


const uint8_t UNI_1v28_RANGE = 0b1111;  // gain of 0.3125
const uint8_t UNI_2v56_RANGE = 0b0111;  // gain of 0.625
const uint8_t UNI_5v12_RANGE = 0b0110;  // gain of 1.25
const uint8_t UNI_10v24_RANGE = 0b0101; // gain of 2.5
const uint8_t BI_0v64_RANGE = 0b1011;   // gain of 0.15625
const uint8_t BI_1v28_RANGE = 0b0011;   // gain of 0.3125
const uint8_t BI_2v56_RANGE = 0b0010;   // gain of 0.625
const uint8_t BI_5v12_RANGE = 0b0001;   // gain of 1.25
const uint8_t BI_10v24_RANGE = 0b0000;  // gain of 2v5

const uint8_t AUTO_SEQ_EN_REG_ADDRESS = 0x01;
const uint8_t AD6886_ALL_INPUTS = 8; // 0-7 = inputs 1-8, 8 = All inputs


void PING_ANALYZER_PIN();
bool AD8668_SetProgReg16 (uint8_t address, uint8_t data);
void AD8668_SetCommandReg16(uint16_t command);
void AD8668_SetVoltageInputRange(uint8_t inputNumber, uint8_t inputVoltageMultiplier);
void AD8668_SetAutoScanEnable();
void AD8668_GetAllVoltages(uint16_t* voltageData);

//=======================================================================================

void PING_ANALYZER_PIN() {
    gpio_put(LOGIC_ANALYZER_GPIO, 1); // Set High
    delayMicroseconds(20);
    gpio_put(LOGIC_ANALYZER_GPIO, 0); // Set Low
    delayMicroseconds(20);
}

bool AD8668_SetProgReg16 (uint8_t address, uint8_t data) {
  bool rtn = false;
  uint16_t txData, rxDataArray[2]; // Array to store data to be sent
  txData = ((((uint16_t)address << 1) | 1) << 8) | (uint16_t)data;
  gpio_put(CS_CV, SPI_BEGIN);
  spi_read16_blocking(SPI_PORT, txData, rxDataArray, 2);
  gpio_put(CS_CV, SPI_END);
  delayMicroseconds(20);
  rxDataArray[1] = rxDataArray[1] >> 8;
  //Serial0_USB.println(String("Shifted Return Data for address (") + address + " = " + rxDataArray[1] + " =? " + data);
   if (rxDataArray[1] == data) { rtn = true; }
   return rtn;
}

void AD8668_SetCommandReg16(uint16_t command) {
  bool rtn = false;
  uint16_t txData[2];
  txData[0] = command;
  txData[1] = 0;
  gpio_put(CS_CV, SPI_BEGIN);
  spi_write16_blocking(SPI_PORT, txData, 2);
  gpio_put(CS_CV, SPI_END);
  delayMicroseconds(5);
}

uint16_t AD8668_ReadNextSampleData16 () {
  uint16_t rxData[2], rtn;
  gpio_put(CS_CV, SPI_BEGIN);
  spi_read16_blocking(SPI_PORT, AD6886_NO_OP, rxData, 2);
  gpio_put(CS_CV, SPI_END);
  delayMicroseconds(5);
  rtn = rxData[1] >> 4;
  return rtn;
}

void AD8668_SetVoltageInputRange(uint8_t inputNumber, uint8_t inputVoltageMultiplier) {
    bool rtn;
    //pingPin7();
    //pingPin7();
    if (inputNumber == AD6886_ALL_INPUTS) { //  inputNumber = [0,7], 8 = AD6886_ALL_INPUTS
      for (auto i=0; i<AD6886_ALL_INPUTS; ++i) {
        rtn = AD8668_SetProgReg16 (AD6886_RG_Ch_0 + i, inputVoltageMultiplier);
        if (!rtn) {Serial0_USB.println("AD8668_SetVoltageInputRange failed"); }
      }
    }
    else {
      rtn = AD8668_SetProgReg16 (AD6886_RG_Ch_0 + inputNumber, inputVoltageMultiplier);
      if (!rtn) {Serial0_USB.println("AD8668_SetVoltageInputRange failed"); }
    }
}

void AD8668_SetAutoScanEnable() {
    AD8668_SetProgReg16 (AUTO_SEQ_EN_REG_ADDRESS, 0b11111111);
}

void AD8668_GetAllVoltages(uint16_t* voltageData) {
    AD8668_SetCommandReg16(AD6886_AUTO_RST);  // set pointer to 1st enable ch - starts A to D
    for (auto i=0; i<8; ++i) {
      voltageData[i] = AD8668_ReadNextSampleData16(); // >> 2 // trim 2 bits of resolution
      //Serial0_USB.println(String("vData(") + i + ") = " + voltageData[i]);
    }
}
