#include "adc_4051_dma.h"
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "hardware/adc.h"
#include <string.h>

#define DMA_BUFFER_SIZE ADC_NUM_SAMPLES

int adc_dma_channel = 1;

uint16_t adc_values[ADC_NUM_INPUTS];

const uint32_t MUX_MASK = ((1ul << CD_4051_PIN_0) | (1ul << CD_4051_PIN_1) | (1ul << CD_4051_PIN_2));

uint16_t dma_buffer_dummy;

void adc_4051_dma_get_all(uint16_t * adc_buffer)
{
    memcpy(adc_buffer, adc_values, sizeof(adc_values));
}

uint16_t adc_4051_dma_get(uint8_t chan)
{
    return adc_values[chan];
}

void adcs_setup_mux(uint8_t channel)
{
  uint32_t mask;

  mask  = (uint32_t)((channel >> 0) & 1u) << CD_4051_PIN_0;
  mask |= (uint32_t)((channel >> 1) & 1u) << CD_4051_PIN_1;
  mask |= (uint32_t)((channel >> 2) & 1u) << CD_4051_PIN_2;

  gpio_put_masked(MUX_MASK, mask);
}

void dma_handler()
{
    static uint8_t cnt = 0;
    dma_hw->ints0 = 1 << adc_dma_channel;
    adc_run(false);
    adc_fifo_drain();

    //read the accumulator
    int32_t samp = dma_hw->sniff_data;
    //16 bits
    #if ADC_NUM_SAMPLES > 16
    samp /= (ADC_NUM_SAMPLES >> 4);
    #endif

    adc_values[cnt] = samp;
    dma_hw->sniff_data = 0;

    //increment to next channel
    cnt++;
    cnt &= ADC_NUM_INPUTS - 1;
    adcs_setup_mux(cnt);

    dma_channel_start(adc_dma_channel);
    adc_run(true);
}

void adc_4051_dma_init()
{
    gpio_init_mask(MUX_MASK);
    gpio_set_dir_out_masked(MUX_MASK);
    gpio_put_masked(MUX_MASK, false);

    adc_init();
    adc_gpio_init(ADC_INPUT_PIN);
    adc_select_input(ADC_INPUT_CHANNEL);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,
        false
    );
    adc_set_clkdiv(0);
    sleep_ms(1000);

    adc_dma_channel = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(adc_dma_channel);

    // Reading from constant address, writing to constant
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, false);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&cfg, DREQ_ADC);

    dma_channel_configure(adc_dma_channel, &cfg,
        &dma_buffer_dummy,   // dst
        &adc_hw->fifo,          // src
        DMA_BUFFER_SIZE,        // transfer count
        false                    // start immediately
    );

    dma_sniffer_enable(adc_dma_channel, DMA_SNIFF_CTRL_CALC_VALUE_SUM, 1);

    dma_channel_set_irq0_enabled(adc_dma_channel, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_handler();
}

