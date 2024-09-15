/* UART Events Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "ScreenCommand.hpp"

static const char *TAG = "RS232-Relay";

static const uint8_t RX_BUF_SIZE = 128;

#define UART_NUM (UART_NUM_1)
#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)
#define LOG_LEVEL (ESP_LOG_INFO)


#define LED_PIN (GPIO_NUM_13)

TaskHandle_t txTaskHandle;
uint8_t rxData[128];

void screenDown(const uint8_t screenIndex) {

}

void screenUp(const uint8_t screenIndex) {

}

void screenQuery(const uint8_t screenIndex) {

}

void processDataBuffer(const uint8_t rxBytes) {
    // Validate command
    ScreenCommand screenCommand;
    if (screenCommand.parseCommand(rxData, rxBytes)) {
        // We have a valid command stored in screenCommand

        // Action based on that command
        switch (screenCommand.command)
        {
        case ScreenCommand::Command::DOWN:
            screenDown(screenCommand.id);
            break;

        case ScreenCommand::Command::UP:
            screenUp(screenCommand.id);
            break;

        case ScreenCommand::Command::QUERY:
            screenQuery(screenCommand.id);
            break;
        
        default:
            break;
        }
    }

    // Incoming command was of an invalid format
    ESP_LOGW(TAG, "Invalid command format recieved");
}

void rxTask(void *arg) {
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, LOG_LEVEL);

    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM, rxData, RX_BUF_SIZE * 2, 200 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            ESP_LOGD(RX_TASK_TAG, "Bytes available to read");
            rxData[rxBytes] = 0; // Null terminated string

            processDataBuffer(rxBytes);

            //vTaskResume(txTaskHandle);
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, rxData);
        }
    }

    // Free up any allocated memory
}

void txTask(void *arg) {
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, LOG_LEVEL);

	uint8_t *data = (uint8_t*) malloc(150);
    while (1) {
		int len = sprintf((char *)data, "Data Received is '%s' \n", rxData);
		uart_write_bytes(UART_NUM, data, len);
		vTaskSuspend(NULL);
    }
    free(data);

    // Free up any allocated memory
}

void initLog(void) {
    esp_log_level_set(TAG, LOG_LEVEL);
}

void initUART(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // TODO - esp okay checks on each of these commands

    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

void initI2C(void) {

}

void initRelay(void) {
    // Initialise the GPIO relay controller over I2C
}

void initLED(void) {
    // Initialise the onboard WS2812B LED
}

void initTimeout(void) {
    // Reboot the device automagically every X seconds
}

void init(void) {
    initLog();
    initUART();
    initI2C();
    initRelay();
    initLED();
    initTimeout();
}

void loadState(void) {

}

void saveState(void) {

}

void app_main(void)
{
    init();
    loadState();

    xTaskCreate(rxTask, "UART_RX_TASK", 1024 * 2, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(txTask, "UART_TX_TASK", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
}
