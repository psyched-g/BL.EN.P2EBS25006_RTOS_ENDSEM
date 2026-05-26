#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"

#include "esp_app_trace.h"
#include "esp_trace.h"
#include "esp_log.h"

#include "soc/uart_pins.h"

#define SEGGER_SYSVIEW_GET_CORE_ID() xPortGetCoreID()
#define SEGGER_SYSVIEW_CORE_COUNT    2

// =====================================================
// GPIO DEFINITIONS
// =====================================================
#define LED_PIN        GPIO_NUM_4
#define BUTTON_PIN     GPIO_NUM_25

// =====================================================
// GLOBALS
// =====================================================
SemaphoreHandle_t buttonSemaphore;

// =====================================================
// TRACE CONFIG
// =====================================================
esp_trace_open_params_t esp_trace_get_user_params(void)
{
    static esp_apptrace_config_t app_trace_config =
        APPTRACE_CONFIG_DEFAULT();

#if CONFIG_APPTRACE_DEST_UART
    app_trace_config.dest_cfg.uart.tx_pin_num = U0TXD_GPIO_NUM;
    app_trace_config.dest_cfg.uart.rx_pin_num = U0RXD_GPIO_NUM;
#endif

    esp_trace_open_params_t trace_params = {
        .core_cfg = NULL,
        .encoder_name = "sysview",
        .encoder_cfg = NULL,
        .transport_name = "apptrace",
        .transport_cfg = &app_trace_config,
    };

    return trace_params;
}

// =====================================================
// TASK A : LED BLINK TASK
// LOW PRIORITY
// =====================================================
void task_led(void *params)
{
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while (1)
    {
        gpio_set_level(LED_PIN, 1);
        ESP_LOGI("TASK_A", "LED ON");

        vTaskDelay(pdMS_TO_TICKS(500));

        gpio_set_level(LED_PIN, 0);
        ESP_LOGI("TASK_A", "LED OFF");

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// =====================================================
// TASK B : UART PRINT TASK
// MEDIUM PRIORITY
// =====================================================
void task_uart(void *params)
{
    while (1)
    {
        ESP_LOGI("TASK_B", "Periodic UART Message");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// =====================================================
// TASK C : BUTTON HANDLING TASK
// HIGH PRIORITY
// =====================================================


// =====================================================
// DEBUG BUTTON TASK
// Used to verify GPIO input state
// =====================================================
void task_debug_button(void *params)
{
    while (1)
    {
        if (xSemaphoreTake(buttonSemaphore,
                           portMAX_DELAY) == pdTRUE)
        {
            ESP_LOGI("TASK_C",
                     "Button Press Detected!");

            // Debounce delay
            vTaskDelay(pdMS_TO_TICKS(200));

            // Re-enable interrupt
            gpio_intr_enable(BUTTON_PIN);
        }
    }
}
// =====================================================
// BUTTON ISR
// =====================================================
// =====================================================
// CUSTOM BUTTON ISR
// =====================================================
static void custom_button_isr(void *arg)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    // Disable interrupt temporarily
    gpio_intr_disable(BUTTON_PIN);

    // Give semaphore to button task
    xSemaphoreGiveFromISR(buttonSemaphore,
                          &higher_priority_task_woken);

    // Request context switch
    if (higher_priority_task_woken == pdTRUE)
    {
        portYIELD_FROM_ISR();
    }
}

// =====================================================
// MAIN
// =====================================================
void app_main(void)
{
    static esp_apptrace_config_t app_trace_config =
        APPTRACE_CONFIG_DEFAULT();

#if CONFIG_APPTRACE_DEST_UART
    app_trace_config.dest_cfg.uart.tx_pin_num =
        U0TXD_GPIO_NUM;

    app_trace_config.dest_cfg.uart.rx_pin_num =
        U0RXD_GPIO_NUM;
#endif

    // =================================================
    // INIT TRACE
    // =================================================
    esp_apptrace_init(&app_trace_config);

    // =================================================
    // BUTTON GPIO CONFIG
    // =================================================
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };

    gpio_config(&io_conf);

    // =================================================
    // CREATE BINARY SEMAPHORE
    // =================================================
    buttonSemaphore = xSemaphoreCreateBinary();

    if (buttonSemaphore == NULL)
    {
        ESP_LOGE("MAIN",
                 "Semaphore creation failed!");

        return;
    }

    // =================================================
    // INSTALL ISR SERVICE
    // =================================================
    gpio_install_isr_service(0);

    gpio_isr_handler_add(BUTTON_PIN,
                     custom_button_isr,
                     NULL);

    gpio_intr_enable(BUTTON_PIN);

    // =================================================
    // CREATE TASKS
    // =================================================

    // TASK A -> LOW PRIORITY
    xTaskCreatePinnedToCore(
        task_led,
        "TASK_A_LED",
        4096,
        NULL,
        1,
        NULL,
        0);

    // TASK B -> MEDIUM PRIORITY
    xTaskCreatePinnedToCore(
        task_uart,
        "TASK_B_UART",
        4096,
        NULL,
        2,
        NULL,
        0);

  
    // DEBUG TASK
    xTaskCreatePinnedToCore(
        task_debug_button,
        "DEBUG_BUTTON",
        2048,
        NULL,
        3,
        NULL,
        0);

    ESP_LOGI("MAIN", "System Started");
}