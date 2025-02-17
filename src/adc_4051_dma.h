#ifndef __ADC_4051_DMA_H__
#define __ADC_4051_DMA_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include "pico/stdlib.h"

#define CD_4051_PIN_0 3//2
#define CD_4051_PIN_1 4//3
#define CD_4051_PIN_2 5//4
#define ADC_INPUT_PIN 26
#define ADC_NUM_INPUTS 8
#define ADC_NUM_SAMPLES 256ul

#if !defined(CD_4051_PIN_0) || !defined(CD_4051_PIN_1) || !defined(CD_4051_PIN_2)
#error "Define MUX Pins!! CD_4051_PIN_[0 ... 2]"
#endif

#ifndef ADC_INPUT_PIN
#define ADC_INPUT_PIN 26
#endif

#ifndef ADC_INPUT_CHANNEL
#define ADC_INPUT_CHANNEL (ADC_INPUT_PIN - 26)
#endif

#if ADC_INPUT_PIN - 26 != ADC_INPUT_CHANNEL
#error "ADC Input pin doesn't match selected ADC channel"
#endif

#ifndef ADC_NUM_INPUTS
#define ADC_NUM_INPUTS 8
#endif

#ifndef ADC_NUM_SAMPLES
#define ADC_NUM_SAMPLES 256ul
#endif

#if ADC_NUM_SAMPLES < 16
#error "Try 16 ADC samples as a minimum"
#endif

/// @brief yea
void adc_4051_dma_init();

/// @brief get all adc values, can be called any time
/// @param adc_buffer pointer to array to fill with adc values
void adc_4051_dma_get_all(uint16_t * adc_buffer);

/// @brief get one specific adc value, can be called any time
/// @param chan which adc input you want to read
/// @return yea
uint16_t adc_4051_dma_get(uint8_t chan);

#ifdef __cplusplus
}
#endif

#endif