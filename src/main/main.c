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
#include <string.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"




static const char *TAG = "RS232-Relay";
static const char *UART_TAG = "UART Data";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO GPIO_NUM_8

#define GPIO_TX GPIO_NUM_16
#define GPIO_RX GPIO_NUM_17
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

uint16_t relay_state = 0;

nvs_handle_t storage_handle;
bool nvs_initialised = false;

static void nvs_read_state() {
    // Opening "storage" nvs handle
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &storage_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s opening NVS handle!", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "NVS storage handle opened");

        // Read state from NVS
        ESP_LOGI(TAG, "Reading state from NVS...");
        err = nvs_get_u16(storage_handle, "relay_state", &relay_state);
        switch (err) {
            case ESP_OK:
                ESP_LOGI(TAG, "Current state: %i", relay_state);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGI(TAG, "State not yet initialised in NVS");
                break;
            default :
                ESP_LOGE(TAG, "NVS Error (%s) reading!", esp_err_to_name(err));
        }

        // Close
        nvs_close(storage_handle);
    }
}

static void nvs_write_state(void) {
    // Opening "storage" nvs handle
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &storage_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s opening NVS handle!", esp_err_to_name(err));
    } else {
        ESP_LOGD(TAG, "NVS storage handle opened");

        // Write
        err = nvs_set_u16(storage_handle, "relay_state", relay_state);
        ESP_LOGI(TAG,"Updating state (%i) in NVS: %s", relay_state, ((err != ESP_OK) ? "Failed!" : "Done"));

        // Commit written value.
        // After setting any values, nvs_commit() must be called to ensure changes are written
        // to flash storage. Implementations may write to storage at other times,
        // but this is not guaranteed.
        err = nvs_commit(storage_handle);
        ESP_LOGD(TAG, "Committing to NVS: %s", ((err != ESP_OK) ? "Failed!" : "Done"));

        // Close
        nvs_close(storage_handle);
    }    
}

static bool update_relay_gpio() {
    uint8_t binaryString[relay_gpio_len + 1];

    for (uint16_t i = 0; i < relay_gpio_len; i++) {
        uint16_t state = ((1 << i) & relay_state) >> i;
        bool enabled = state == 1;

        binaryString[i] = '0' + state;

        //ESP_LOGD(UART_TAG, "Relay %i's state is %s", i+1, enabled ? "ON" : "OFF");
        gpio_set_level(relay_gpio[i], enabled);
    }

    binaryString[relay_gpio_len] = '\0';

    ESP_LOGI(UART_TAG, "State: %s", (char*) binaryString);

    return true;
}

static bool set_relay(uint8_t id, bool state) {
    // Validate relayID for our system
    if (id > relay_gpio_len) {
        ESP_LOGW(UART_TAG, "Max relay ID is %i, relay ID provided was %i", relay_gpio_len, id);
        return false;
    }

    if (id == 0) {
        for (uint8_t i = 1; i < relay_gpio_len + 1; i++) {
            set_relay(i, state);
        }
        return true;
    }
    
    ESP_LOGI(UART_TAG, "Setting relay %i to %s", id, state == 1 ? "ON" : "OFF");
    
    uint16_t old_relay_state = relay_state;
    if (state) {
        relay_state = relay_state | (1 << (id - 1));
    } else {
        relay_state = relay_state & ~(1 << (id - 1));
    }

    ESP_LOGD(UART_TAG, "Current relay state: %i", relay_state);

    if (relay_state == old_relay_state) return true;
    // We only need to save the new state if the state has actually changed!

    // Save relay state to NVS
    nvs_write_state();

    // Update physical relay state
    update_relay_gpio();

    return true;
}

static bool query_relay(uint8_t id) {
    // Validate relayID for our system
    if (id > relay_gpio_len) {
        ESP_LOGW(UART_TAG, "Max relay ID is %i, relay ID provided was %i", relay_gpio_len, id);
        return false;
    }

    if (id == 0) {
        for (int i = 1; i < relay_gpio_len; i++) {
            query_relay(i);
        }
        return true;
    }

    uint16_t state = ((1 << (id - 1)) & relay_state) >> (id - 1);;
    ESP_LOGI(UART_TAG, "Relay query: relay %i's state is %s", id, state == 1? "ON" : "OFF");
    
    uint8_t data[6];

    data[0] = (id / 10) + '0';
    data[1] = (id % 10) + '0';
    data[2] = '=';

    if (state == 1) {
        data[3] = '+';
    } else {
        data[3] = '-';
    }

    data[4] = '\r';

    data[5] = '\0';

    ESP_LOGD(UART_TAG, "Outgoing message: %s", (char *) data);
    
    uart_write_bytes(UART_NUM, (const char *) data, 5); // We don't want to write the null termination byte to UART
    
    return true;
}

static bool validate_char_digit(uint8_t char_digit) {
    if (char_digit < '0' || char_digit > '9') {
        ESP_LOGW(UART_TAG, "Character %c is not a digit", char_digit);
        return false;
    }
    return true;
}

static bool process_data(uint8_t data[], uint8_t length) {
    // Valid data format:
    // 05:+\r
    // Valid response format:
    // 05=+\r

    // Data will be at max 128 characters long, so validate that length is less
    if (length > 128) 
    {
        ESP_LOGW(UART_TAG, "Length greater than 128");
        return false;
    }

    // All messages are 5 characters long
    if (length != 5) {
        ESP_LOGW(UART_TAG, "Length of command bytes should be 5 not %i", length);
        return false;
    }

    // 3rd character is the command (:) or response (=)
    // Here we expect command only
    if (data[2] != ':') {
        ESP_LOGW(UART_TAG, "Character \':\' expected in 3rd byte not %c", data[2]);
        return false;
    }

    // Confirm the message ends in \r
    if (data[4] != '\r') {
        ESP_LOGW(UART_TAG, "Carriage return character \'\\r\' expected in 5th byte not %c", data[4]);
        return false;
    }

    // Begin with 2 digits indicating the relay ID (starting at 1, 0 indicates all)
    
    // Figure out the relay ID
    uint8_t relayID = 0;

    // Validate that both slots are valid digits
    if (!validate_char_digit(data[0]) || !validate_char_digit(data[1])) return false;

    uint8_t first_digit = data[0] - '0';
    uint8_t second_digit = data[1] - '0';

    relayID = first_digit * 10 + second_digit;

    // Commands are as follows:
    // + -> down (relays ON)
    // - -> up (relays OFF)
    // ? -> query state - we should respond
    // Query state for ID 0 returns a series of responses for all relay states
    
    switch (data[3]) {
        case '+':
            return set_relay(relayID, true);
            break;

        case '-':
            return set_relay(relayID, false);
            break;

        case '?':
            return query_relay(relayID);
            break;

        default:
            ESP_LOGW(UART_TAG, "Invalid command character in 4th byte, expected +, -, or ?, got %c", data[3]);
            return false; // None of the valid commands were found
    }

    // We shouldn't get here - the return path should be made in the switch above.
    // If we get here something has gone wrong!
    return false;
}

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

    ESP_LOGI(TAG, "UART Initialised");
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
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_LOGI(TAG, "GPIO Initialised");
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

    ESP_LOGI(TAG, "LED Strip Initialised");
}



static void configure_nvs(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    nvs_initialised = true;

    ESP_LOGI(TAG, "NVS Flash Initialised");
}

void app_main(void)
{
    configure_uart();

    configure_led();

    configure_relay_gpio();

    configure_nvs();

    // Read state from NVS
    nvs_read_state();
    nvs_write_state();

    // Set initial physical output state
    update_relay_gpio();

    uint8_t data_buffer[128];

    while(1) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM, data_buffer, (128 - 1), 20 / portTICK_PERIOD_MS);

        if (len) {
            process_data(data_buffer, len);
        }
    }

    // Test code

    while (1) {
        ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led();
        /* Toggle the LED state */
        s_led_state = !s_led_state;

        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
    }
}
