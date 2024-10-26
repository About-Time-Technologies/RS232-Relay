/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include <inttypes.h>




static const char *TAG = "RS232-Relay";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO GPIO_NUM_8

#define GPIO_TX GPIO_NUM_21
#define GPIO_RX GPIO_NUM_22
#define UART_NUM UART_NUM_1

static const uint8_t relay_gpio_len = 12;
gpio_num_t relay_gpio[12] = {
    GPIO_NUM_5,
    GPIO_NUM_4,
    GPIO_NUM_3,
    GPIO_NUM_2,
    GPIO_NUM_1,
    GPIO_NUM_0,
    GPIO_NUM_15,
    GPIO_NUM_18,
    GPIO_NUM_19,
    GPIO_NUM_20,
    GPIO_NUM_21,
    GPIO_NUM_22,
};

static uint8_t s_led_state = 0;

static led_strip_handle_t led_strip;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void configure_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // Disable hardware flow control
        //.flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
        .rx_flow_ctrl_thresh = 122,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, GPIO_TX, GPIO_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Setup UART buffered IO with event queue
    const int uart_buffer_size = (1024 * 2);
    QueueHandle_t uart_queue;
    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, uart_buffer_size, \
                                            uart_buffer_size, 10, &uart_queue, 0));
}

static void configure_relay_gpio(void) 
{
    uint64_t pin_bit_mask = 0;
    
    for (int i = 0; i < relay_gpio_len; i++) {
        pin_bit_mask |= 1ULL << relay_gpio[i];
    }

    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = pin_bit_mask;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

static void configure_led(void)
{
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO, // The GPIO that connected to the LED strip's data line
        .max_leds = 1, // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812, // LED strip model
        .flags.invert_out = false, // whether to invert the output signal (useful when your hardware has a level inverter)
    };

    led_strip_rmt_config_t rmt_config = {
    #if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .rmt_channel = 0,
    #else
        .clk_src = RMT_CLK_SRC_DEFAULT, // different clock source can lead to different power consumption
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false, // whether to enable the DMA feature
    #endif
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
}

void app_main(void)
{
    configure_uart();

    /* Configure the peripheral according to the LED type */
    configure_led();

    configure_relay_gpio();

    int relay_count = 0;


    while (1) {
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led();
        /* Toggle the LED state */
        s_led_state = !s_led_state;

        for (int i = 0; i < relay_gpio_len; i++) {
            int state = 0;
            if (i == relay_count) state = 1;
            gpio_set_level(relay_gpio[i], state);
        }

        ESP_LOGI(TAG, "Turning on relay: %i", relay_count);

        // Write data to UART.
        char* test_str = "Hello!.\n";
        uart_write_bytes(UART_NUM, (const char*)test_str, strlen(test_str));

        relay_count++;
        if (relay_count >= relay_gpio_len) relay_count = 0;

        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
