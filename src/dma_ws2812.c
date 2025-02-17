
#include "dma_ws2812.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include <Arduino.h>

/// @brief this will be changed if the DMA channel is already in use
int dma_channel = 0;

void * led_buf = NULL;

uint32_t dma_ws2812_leds_update()
{
    static uint32_t prev_time_us = 0;

    if (dma_channel_is_busy(dma_channel))
    return 0;

    uint32_t time_now = micros();
    //uint32_t time_now = time_us_32();

    if (time_now - prev_time_us < 20000)
    return 0;

    prev_time_us = time_now;

    dma_channel_set_read_addr(dma_channel, led_buf, false);
    dma_channel_start(dma_channel);

    return 1;
}

static const uint16_t ws2812_program_instructions[] =
{
            //     .wrap_target
    0x6221, //  0: out    x, 1            side 0 [2]
    0x1123, //  1: jmp    !x, 3           side 1 [1]
    0x1400, //  2: jmp    0               side 1 [4]
    0xa442, //  3: nop                    side 0 [4]
            //     .wrap
};

static const struct pio_program ws2812_program =
{
    .instructions = ws2812_program_instructions,
    .length = 4,
    .origin = -1,
};

static inline pio_sm_config ws2812_program_get_default_config(uint offset)
{
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + 0, offset + 3);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}

static inline void ws2812_program_init(PIO pio, uint sm, uint offset, uint pin, float freq)
{
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    pio_sm_config c = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false, true, 8);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    int cycles_per_bit = 10;
    float div = clock_get_hz(clk_sys) / (freq * cycles_per_bit);
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

void dma_ws2812_init(void * rgb_led_buffer, int rgb_led_buffer_size)
{
    //const uint8_t transfer_count = WS2812_NUM_OF * 3;
    const uint8_t transfer_count =  rgb_led_buffer_size * 3;
    const uint8_t led_pin = WS2812_PICO_PIN;
    const pio_program_t* prog = &ws2812_program;
    led_buf = rgb_led_buffer;

    uint offset = pio_add_program(pio0, prog);

    uint state_machine = 0;

    ws2812_program_init(pio0, state_machine, offset, led_pin, 800000);

    //compilation will fail here if all dma channels are in use.
    dma_channel = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_channel);
    channel_config_set_dreq(&cfg, pio_get_dreq(pio0, state_machine, true));
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);


    dma_channel_configure(dma_channel, &cfg,
        &pio0->txf[state_machine],      // dst
        led_buf,                 // src
        transfer_count,                     // transfer count
        true                           // start immediately
    );


}


#ifdef __TEST_CANTRELL__


int main()
{
    uint8_t test[WS2812_NUM_OF * 3] = {0}; // passing in size rgb_led_buffer_size

    dma_ws2812_init(test);

    //pio_sm_put_blocking(pio0, 0, test[0]);

    while(1)
    {
        test[2] += dma_ws2812_leds_update();
        test[0] = ~test[2];
    }

    return 0;
}

#endif