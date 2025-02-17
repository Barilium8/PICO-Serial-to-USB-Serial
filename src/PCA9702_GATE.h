// file: PCA9702_GATE.h
// brief: recieve data from the NXP PCA9702 SPI shift register
// authors: Steve Barile
// date: Nov 1 2024

#pragma once

#include <Arduino.h>
#include <stdio.h>
#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "Globals.h"

// SPI on PICO video:  https://www.youtube.com/watch?v=s7Lud1Gqrqw
// the source from the video sample: https://learnembeddedsystems.co.uk/bmp280-and-pi-pico-over-spi

 // TI AD8668 ADC SPI info
 // https://www.mouser.com/datasheet/2/302/PCA9701_PCA9702-3139411.pdf
 // page 7

uint8_t PCA9702_GatAllGates();

//=======================================================================================

uint8_t PCA9702_GatAllGates() {
  uint8_t txData = 0, rxData = 0;
  gpio_put(CS_CG, SPI_BEGIN);
    delayMicroseconds(5);
    uint8_t read = spi_read_blocking(SPI_PORT, txData, &rxData, 1);
    delayMicroseconds(5);
  gpio_put(CS_CG, SPI_END);
  //Serial.println(String("GATE ") +  rxData + "  read " + read);
  return rxData;
}



