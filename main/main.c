#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define MSP_UART_NUM    UART_NUM_2
#define MSP_TX_PIN      GPIO_NUM_18
#define MSP_RX_PIN      GPIO_NUM_19
#define MSP_BAUDRATE    230400
#define RX_BUF_SIZE     256
#define MAX_CHANNELS    16
#define MSP_CMD_RC      105

#define SERVO_A_GPIO    GPIO_NUM_12
#define SERVO_B_GPIO    GPIO_NUM_13
#define SERVO_C_GPIO    GPIO_NUM_14
#define SERVO_D_GPIO    GPIO_NUM_25
#define SERVO_E_GPIO    GPIO_NUM_26
#define SERVO_F_GPIO    GPIO_NUM_27

#define SERVO_FREQ      50
#define SERVO_MIN_US    1000
#define SERVO_MAX_US    2000

#define MIN_CHANNEL     1000
#define MAX_CHANNEL     2000

#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_RES        LEDC_TIMER_16_BIT

#define SERVO_A_CH  LEDC_CHANNEL_0
#define SERVO_B_CH  LEDC_CHANNEL_1
#define SERVO_C_CH  LEDC_CHANNEL_2
#define SERVO_D_CH  LEDC_CHANNEL_3
#define SERVO_E_CH  LEDC_CHANNEL_4
#define SERVO_F_CH  LEDC_CHANNEL_5

static const char *TAG = "MSP_SERVO";

uint16_t rc_channels[MAX_CHANNELS];
uint16_t ServoA = 1500;
uint16_t ServoB = 1500;
uint16_t ServoC = 1500;
uint16_t ServoD = 1500;
uint16_t ServoE = 1500;
uint16_t ServoF = 1500;
SemaphoreHandle_t rc_mutex;

int32_t map_constrain(int32_t x,
                      int32_t in_min,
                      int32_t in_max,
                      int32_t out_min,
                      int32_t out_max)
{
    if (x < in_min) x = in_min;
    if (x > in_max) x = in_max;

    return (x - in_min) * (out_max - out_min) /
           (in_max - in_min) + out_min;
}

/* ================= SERVO ================= */
static uint32_t servo_us_to_duty(uint32_t us)
{
    uint32_t duty_max = (1 << LEDC_RES) - 1;
    return (us * duty_max) / 20000;
}

void servo_set_us(ledc_channel_t ch, uint32_t us)
{
    uint32_t duty = servo_us_to_duty(us);
    ledc_set_duty(LEDC_MODE, ch, duty);
    ledc_update_duty(LEDC_MODE, ch);
}

void servo_set_from_rc(ledc_channel_t ch, uint16_t rc)
{
    uint32_t us = map_constrain(rc, MIN_CHANNEL, MAX_CHANNEL,
                                SERVO_MIN_US, SERVO_MAX_US);
    servo_set_us(ch, us);
}

void servo_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_RES,
        .freq_hz          = SERVO_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch_a = {
        .gpio_num   = SERVO_A_GPIO,
        .speed_mode = LEDC_MODE,
        .channel    = SERVO_A_CH,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };

    ledc_channel_config_t ch_b = ch_a;
    ch_b.channel  = SERVO_B_CH;
    ch_b.gpio_num = SERVO_B_GPIO;

    ledc_channel_config_t ch_c = ch_a;
    ch_c.channel  = SERVO_C_CH;
    ch_c.gpio_num = SERVO_C_GPIO;

    ledc_channel_config_t ch_d = ch_a;
    ch_d.channel  = SERVO_D_CH;
    ch_d.gpio_num = SERVO_D_GPIO;

    ledc_channel_config_t ch_e = ch_a;
    ch_e.channel  = SERVO_E_CH;
    ch_e.gpio_num = SERVO_E_GPIO;

    ledc_channel_config_t ch_f = ch_a;
    ch_f.channel  = SERVO_F_CH;
    ch_f.gpio_num = SERVO_F_GPIO;


    ledc_channel_config(&ch_a);
    ledc_channel_config(&ch_b);
    ledc_channel_config(&ch_c);
    ledc_channel_config(&ch_d);
    ledc_channel_config(&ch_e);
    ledc_channel_config(&ch_f);
}

void msp_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = MSP_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    uart_driver_install(MSP_UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(MSP_UART_NUM, &cfg);
    uart_set_pin(MSP_UART_NUM, MSP_TX_PIN, MSP_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void msp_send(uint8_t cmd)
{
    uint8_t packet[6];
    uint8_t checksum = 0;

    packet[0] = '$';
    packet[1] = 'M';
    packet[2] = '<';
    packet[3] = 0;
    packet[4] = cmd;
    checksum  = packet[3] ^ packet[4];
    packet[5] = checksum;

    uart_write_bytes(MSP_UART_NUM, (const char *)packet, 6);
}

void msp_parse_byte(uint8_t b)
{
    static uint8_t state = 0, len = 0, cmd = 0, idx = 0, crc = 0;
    static uint8_t payload[64];

    switch (state) {
        case 0: if (b == '$') state = 1; break;
        case 1: if (b == 'M') state = 2; else state = 0; break;
        case 2: if (b == '>') state = 3; else state = 0; break;
        case 3: len = b; crc = b; state = 4; break;
        case 4: cmd = b; crc ^= b; idx = 0;
                state = (len > 0) ? 5 : 6; break;
        case 5:
            payload[idx++] = b;
            crc ^= b;
            if (idx >= len) state = 6;
            break;
        case 6:
            if (crc == b && cmd == MSP_CMD_RC) {
                xSemaphoreTake(rc_mutex, portMAX_DELAY);
                for (int i = 0; i < MAX_CHANNELS; i++) {
                    rc_channels[i] =
                        payload[i * 2] | (payload[i * 2 + 1] << 8);
                }
                xSemaphoreGive(rc_mutex);
            }
            state = 0;
            break;
    }
}

void msp_rx_task(void *arg)
{
    uint8_t buf[RX_BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(
            MSP_UART_NUM, buf, RX_BUF_SIZE, 10 / portTICK_PERIOD_MS);
        for (int i = 0; i < len; i++) {
            msp_parse_byte(buf[i]);
        }
    }
}

void main_task(void *arg)
{
    uint16_t rc[MAX_CHANNELS];

    while (1) {
        xSemaphoreTake(rc_mutex, portMAX_DELAY);
        memcpy(rc, rc_channels, sizeof(rc));
        xSemaphoreGive(rc_mutex);

        /* ===== MULTIPLEXER SERVO ===== */
        if (rc[14] <= 1500) {              
            if (rc[13] < 1250)       ServoA = rc[12];
            else if (rc[13] < 1750)  ServoB = rc[12];
            else                     ServoC = rc[12];
        } else {                              
            if (rc[13] < 1250)       ServoD = rc[12];
            else if (rc[13] < 1750)  ServoE = rc[12];
            else                     ServoF = rc[12];
        }

        /* ===== APPLY SERVO ===== */
        servo_set_from_rc(SERVO_A_CH, ServoA);
        servo_set_from_rc(SERVO_B_CH, ServoB);
        servo_set_from_rc(SERVO_C_CH, ServoC);
        servo_set_from_rc(SERVO_D_CH, ServoD);
        servo_set_from_rc(SERVO_E_CH, ServoE);
        servo_set_from_rc(SERVO_F_CH, ServoF);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


void app_main(void)
{
    rc_mutex = xSemaphoreCreateMutex();
    msp_uart_init();
    servo_init();

    xTaskCreate(msp_rx_task, "msp_rx", 4096, NULL, 10, NULL);
    xTaskCreate(main_task, "main", 4096, NULL, 5, NULL);

    while (1) {
        msp_send(MSP_CMD_RC);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
