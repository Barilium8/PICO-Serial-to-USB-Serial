#ifndef __DMA_WS_H__
#define __DMA_WS_H__

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __TEST_CANTRELL__
#include <stdio.h>
#include "pico/stdlib.h"
#else
#include <inttypes.h>
//#warning "i dont know if you have to include something here or if platformio will just work"
#endif

// #define WS2812_NUM_OF 47
#define WS2812_PICO_PIN 11

/// @brief initialize driver for ws2812 and dma.
/// @param rgb_led_buffer pass the address to an LED buffer. the buffer array type must be 4 bytes wide.
void dma_ws2812_init(void * rgb_led_buffer, int rgb_led_buffer_size);

/// @brief call this function to trigger the dma to start running. this will update all the leds.
/// @return true if the dma was started. false if the dma was already running OR if the last successful call was < 20 ms prior
uint32_t dma_ws2812_leds_update();

#ifdef __cplusplus
}
#endif


#endif