/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "soc/uart_struct.h"

#define UART_SBUS UART_NUM_2
#define SBUS_TXD 17
#define SBUS_RXD 16
#define BUF_SIZE 256

static void uart_init()
{
    uart_config_t uart_config = {
       .baud_rate = 100000,
       .data_bits = UART_DATA_8_BITS,
       .parity = UART_PARITY_EVEN,
       .stop_bits = UART_STOP_BITS_1,
       .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
       .rx_flow_ctrl_thresh = 122,
    };

    uart_param_config(UART_SBUS, &uart_config);
    uart_set_pin(UART_SBUS, SBUS_TXD, SBUS_RXD,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_line_inverse(UART_SBUS, UART_INVERSE_RXD);
    uart_driver_install(UART_SBUS, BUF_SIZE, BUF_SIZE, 0, NULL, 0);

    printf("uart2 initialized for SBUS input\n");
}

#define SBUS_NUM_CHANNELS 16
#define SBUS_FRAME_SIZE	25
#define SBUS_FLAGS_BYTE	23
#define SBUS_FAILSAFE_BIT 3
#define SBUS_FRAMELOST_BIT 2

struct {
  uint16_t bytes[SBUS_FRAME_SIZE]; // including start bit, parity and stop bits
  uint16_t bit_ofs;
} sbus_state;

static uint16_t pwm_values[SBUS_NUM_CHANNELS];

/* S.BUS decoder matrix based on src/modules/px4iofirmware/sbus.c from
   PX4Firmware.

   Each channel value can come from up to 3 input bytes. Each row in the
   matrix describes up to three bytes, and each entry gives:

   - byte offset in the data portion of the frame
   - right shift applied to the data byte
   - mask for the data byte
   - left shift applied to the result into the channel value
 */
struct sbus_bit_pick {
	uint8_t byte;
	uint8_t rshift;
	uint8_t mask;
	uint8_t lshift;
};

static const struct sbus_bit_pick sbus_decoder[SBUS_NUM_CHANNELS][3] = {
  /*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
  /*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
  /*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
  /*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
  /*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
  /*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
  /*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
  /*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
  /*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
  /*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
  /* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
  /* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
  /* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
  /* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
  /* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
  /* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};

static bool
sbus_decode (const uint8_t frame[SBUS_FRAME_SIZE])
{
    /* check frame boundary markers to avoid out-of-sync cases */
    if (frame[0] != 0x0f)
        return false;

    /* use the decoder matrix to extract channel data */
    for (int ch = 0; ch < SBUS_NUM_CHANNELS; ch++) {
        unsigned value = 0;

        for (int pick = 0; pick < 3; pick++) {
            const struct sbus_bit_pick *decode = &sbus_decoder[ch][pick];

            if (decode->mask != 0) {
                unsigned piece = frame[1 + decode->byte];
                piece >>= decode->rshift;
                piece &= decode->mask;
                piece <<= decode->lshift;
                value |= piece;
            }
        }

        /* Map SBUS range.
            200 -> 1000micro
           1800 -> 2000micro
        */
        pwm_values[ch] = (uint16_t)((value-200)/1.6 +.5f) + 1000;
        //printf("%d: %d\n", ch, pwm_values[ch]);
    }

  return true;
}

extern xQueueHandle sbus_queue;

// If no input for 5000us, skip to next mark.
#define FRAME_SPACE (5 / portTICK_PERIOD_MS)

void sbus_read(void)
{
    uint8_t c;
    uint8_t buf[25];
    int n, count = 0;
    bool start;

    start = false;
    while (count < 25) {
        n = uart_read_bytes(UART_SBUS, &c, 1, portMAX_DELAY);
        if (n <= 0)
            break;
        //printf("0x%02x ", c);
        if (!start)
            {
                if (c != 0x0f)
                    continue;
                start = true;
            }
        buf[count++] = c;
        if (count < 25)
            continue;
        if (buf[24] == 0x0 || (buf[24] & 0xc4) == 0x4) {
            //dump_frame (buf);
            if (sbus_decode(buf)) {
                if (xQueueSend(sbus_queue, pwm_values, 8*sizeof(uint16_t))
                    != pdTRUE) {
                    //printf("fail to queue pwm_values\n");
                }
            }
            break;
        }
        // Skip to end byte
        while (uart_read_bytes(UART_SBUS, &c, 1, portMAX_DELAY) == 1
               && !(c == 0x0 || (c & 0xc4) == 0x4))
            ;
        break;
    }
}

void sbus_task(void *arg)
{
    uart_init();

    while(1) {
        sbus_read();
    }
}

