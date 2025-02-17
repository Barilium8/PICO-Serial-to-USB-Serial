#pragma once

#include <Arduino.h>
#include <stdio.h>

// the logic analyzer uses this pin to trigger for debugging
#define LOGIC_ANALYZER_GPIO 7  // Pin10

// SPI GPIO Defs
#define CV_RST 22
#define SCLK 14
#define MOSI 15
#define MISO 12
#define BAD_MISO 16
#define CS_CV 17
#define CS_CG 13
#define SPI_PORT spi1

const uint8_t SPI_BEGIN = 0x00;
const uint8_t SPI_END = 0x01;
const uint8_t SPI_READ = 0x80; // '| SPI_READ' sets MSB = 1
const uint8_t SPI_WRITE = 0x7F; // '& SPI_WRITE' sets MSB = 0

//const uint32_t SPI_CLK_RATE = 4'500'000; // Gate Max 5MHz, CV Max 17MHz
const uint32_t SPI_CLK_RATE = 100'000; // Gate Max 5MHz, CV Max 17MHz

#define Serial0_USB Serial