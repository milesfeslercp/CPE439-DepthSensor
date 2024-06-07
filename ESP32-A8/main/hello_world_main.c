#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_pm.h"
#include "portmacro.h"

#define BUTTON_GPIO GPIO_NUM_0
// Initialize GPIO
#define LED_GPIO GPIO_NUM_32

// Semaphore handle
static SemaphoreHandle_t xSemaphore = NULL;

// Task function that blinks an LED
void task_blink_led(void *params) {
    while (1) {
        // Wait for semaphore to be given
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            for (int i = 0; i < 5; i++) {
                gpio_set_level(LED_GPIO, 1);
                printf("LED on\n");
                vTaskDelay(200 / portTICK_PERIOD_MS); // LED on for 200ms
                gpio_set_level(LED_GPIO, 0);
                printf("LED off\n");
                vTaskDelay(200 / portTICK_PERIOD_MS); // LED off for 200ms

            }
        }
    }
}

// Task function that gives semaphore
void task_give_semaphore(void *params) {
    while (1) {
        xSemaphoreGive(xSemaphore);
        vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait for 5 seconds
    }
}

// GPIO interrupt handler
static void IRAM_ATTR gpio_isr_handler(void *arg) {
    // Give the semaphore from ISR
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}



// Initialize GPIO
void init_gpio(void) {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_NEGEDGE; // Interrupt on negative edge
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << BUTTON_GPIO);
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    // Set up LED GPIO
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, gpio_isr_handler, NULL);
}


void app_main(void) {
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 240,
        .min_freq_mhz = 10,
        .light_sleep_enable = true // enables tickless idle mode
    };

    esp_err_t pmconfigreturn = esp_pm_configure(&pm_config);
    if (pmconfigreturn != ESP_OK) {
        printf("esp_pm_configure failed\n");
        // print pmconfigreturn
        printf("pmconfigreturn: %d\n", pmconfigreturn);
    }

    // Enable tickless idle mode
    esp_sleep_enable_timer_wakeup(50 * 1000); // 50 ms idle before sleep
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, 1);

    xSemaphore = xSemaphoreCreateBinary();
    init_gpio();
    
    // Create tasks
    xTaskCreate(task_blink_led, "TaskBlinkLED", 2048, NULL, 5, NULL);
    xTaskCreate(task_give_semaphore, "TaskGiveSemaphore", 2048, NULL, 5, NULL);
}