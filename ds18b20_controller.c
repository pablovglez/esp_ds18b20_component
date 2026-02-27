#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "freertos/queue.h"
#include "ds18b20_controller.h"

// GPIO pin for DS18B20 data line
// Check available pins in gpio_num.h
int ONE_WIRE_PIN = 4; // Default value, will be overridden by config
int POLL_INTERVAL_MS = 60000; // Default polling interval (1 minute)

static const char *TAG = "DS18B20_CONTROLLER";

// DS18B20 commands
#define DS18B20_CMD_CONVERTTEMP     0x44
#define DS18B20_CMD_RSCRATCHPAD      0xBE
#define DS18B20_CMD_WSCRATCHPAD      0x4E
#define DS18B20_CMD_CPYSCRATCHPAD    0x48
#define DS18B20_CMD_RECEEPROM        0xB8
#define DS18B20_CMD_RPWRSUPPLY       0xB4

// 1-Wire timing constants (in microseconds)
#define ONE_WIRE_RESET_PULSE      600 //480
#define ONE_WIRE_RESET_WAIT        80 //60
#define ONE_WIRE_PRESENCE_WAIT     320 //240
#define ONE_WIRE_READ_SLOT         70
#define ONE_WIRE_WRITE_SLOT        70
#define ONE_WIRE_RECOVERY           5
#define MAX_RETRIES 3

// Floating filter
#define FILTER_SIZE 5

// Global definitions
QueueHandle_t ds18b20_message_queue = NULL;
uint32_t temp_timestamp = 0;
static float temp_buffer[FILTER_SIZE];
static uint8_t buffer_index = 0;
static bool buffer_full = false;

/**
 * @brief Set GPIO direction and level
 */
static void one_wire_set_output(bool level)
{
    gpio_set_direction(ONE_WIRE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(ONE_WIRE_PIN, level);
}

/**
 * @brief Set GPIO as input
 */
static void one_wire_set_input(void)
{
    gpio_set_direction(ONE_WIRE_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ONE_WIRE_PIN, GPIO_PULLUP_ONLY);
}

/**
 * @brief Read GPIO level
 */
static int one_wire_read(void)
{
    return gpio_get_level(ONE_WIRE_PIN);
}

/**
 * @brief Perform 1-Wire reset and check for presence pulse
 */
static bool one_wire_reset(void)
{
    bool presence = false;
    // Disable interrupts for timing-critical section
    portDISABLE_INTERRUPTS();
    
    // Pull line low for reset pulse
    one_wire_set_output(0);
    ets_delay_us(ONE_WIRE_RESET_PULSE);
    
    // Release line and wait for devices to respond
    one_wire_set_input();
    ets_delay_us(ONE_WIRE_RESET_WAIT);
    
    // Check for presence pulse (line pulled low by device)
    if (!one_wire_read()) {
        presence = true;
    }
    
    // Wait for remainder of presence pulse window
    ets_delay_us(ONE_WIRE_PRESENCE_WAIT - ONE_WIRE_RESET_WAIT);
    
    // Re-enable interrupts after timing-critical section
    portENABLE_INTERRUPTS();
    
    return presence;
}

/**
 * @brief Write a single bit to 1-Wire bus
 */
static void one_wire_write_bit(int bit)
{
    if (bit) {
        // Write 1 bit
        one_wire_set_output(0);
        ets_delay_us(6);
        one_wire_set_input();
        ets_delay_us(64);
    } else {
        // Write 0 bit
        one_wire_set_output(0);
        ets_delay_us(60);
        one_wire_set_input();
        ets_delay_us(10);
    }
}

/**
 * @brief Read a single bit from 1-Wire bus
 */
static int one_wire_read_bit(void)
{
    int bit;
    
    one_wire_set_output(0);
    ets_delay_us(6);
    one_wire_set_input();
    ets_delay_us(9);
    
    bit = one_wire_read();
    ets_delay_us(55);
    
    return bit;
}

/**
 * @brief Write a byte to 1-Wire bus
 */
static void one_wire_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        one_wire_write_bit(byte & 0x01);
        byte >>= 1;
    }
}

/**
 * @brief Read a byte from 1-Wire bus
 */
static uint8_t one_wire_read_byte(void)
{
    uint8_t byte = 0;
    
    for (int i = 0; i < 8; i++) {
        byte >>= 1;
        if (one_wire_read_bit()) {
            byte |= 0x80;
        }
    }
    
    return byte;
}

/**
 * @brief Read temperature from DS18B20
 */
static float ds18b20_read_temperature(void)
{
    uint8_t scratchpad[9];
    int16_t raw_temp;
    float temp_c;
    
    for (int i = 0; i < MAX_RETRIES; i++) {
        if (one_wire_reset()) {
            break;
        } else {
            ESP_LOGD(TAG, "DS18B20 not detected, retrying... (%d/%d)", i + 1, MAX_RETRIES);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (i == MAX_RETRIES - 1) {
            ESP_LOGE(TAG, "Failed to detect DS18B20 after %d attempts", MAX_RETRIES);
            return -999.0f;
        }
    }
    
    
    // Skip ROM (assuming single device)
    one_wire_write_byte(0xCC);
    
    // Start temperature conversion
    one_wire_write_byte(DS18B20_CMD_CONVERTTEMP);
    
    // Wait for conversion (750ms max for 12-bit resolution)
    vTaskDelay(pdMS_TO_TICKS(750));
    
    // Reset again
    one_wire_reset();
    
    // Skip ROM
    one_wire_write_byte(0xCC);
    
    // Read scratchpad
    one_wire_write_byte(DS18B20_CMD_RSCRATCHPAD);
    
    // Read 9 bytes of scratchpad
    for (int i = 0; i < 9; i++) {
        scratchpad[i] = one_wire_read_byte();
    }
    
    // Calculate temperature
    raw_temp = (scratchpad[1] << 8) | scratchpad[0];
    
    // Convert to Celsius (12-bit resolution = 0.0625°C per bit)
    temp_c = raw_temp * 0.0625f;
    
    // Log scratchpad for debugging
    ESP_LOGD(TAG, "Scratchpad: %02X %02X %02X %02X %02X %02X %02X %02X %02X",
             scratchpad[0], scratchpad[1], scratchpad[2], scratchpad[3],
             scratchpad[4], scratchpad[5], scratchpad[6], scratchpad[7], scratchpad[8]);
    
    return temp_c;
}

void ds18b20_init(int gpio_num, int poll_interval_ms)
{   
    // Update global variables with config values if valid
    if (gpio_num >= 0 && gpio_num < GPIO_NUM_MAX) {
        ONE_WIRE_PIN = gpio_num;
    } else {
        ESP_LOGW(TAG, "Invalid GPIO number %d, using default GPIO%d", gpio_num, ONE_WIRE_PIN);
    }
    if (poll_interval_ms > 0) {
        POLL_INTERVAL_MS = poll_interval_ms;
    } else {
        ESP_LOGW(TAG, "Invalid polling interval %d ms, using default %d ms", poll_interval_ms, POLL_INTERVAL_MS);
    }
    // Configure GPIO for 1-Wire
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ONE_WIRE_PIN),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,  // Open-drain mode
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Create a queue for processing received sentences
    ds18b20_message_queue = xQueueCreate(10, sizeof(temp_message_t));
    if (ds18b20_message_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create message queue");
    } else {
        ESP_LOGI(TAG, "Message queue created successfully");
    }
    
    ESP_LOGI(TAG, "DS18B20 initialized on GPIO %d with polling interval %d ms", ONE_WIRE_PIN, POLL_INTERVAL_MS);
}

static float apply_moving_average(float new_temp)
{
    temp_buffer[buffer_index] = new_temp;
    buffer_index = (buffer_index + 1) % FILTER_SIZE;
    
    if (!buffer_full && buffer_index == 0) {
        buffer_full = true;
    }
    
    float sum = 0;
    uint8_t count = buffer_full ? FILTER_SIZE : buffer_index;
    
    for (uint8_t i = 0; i < count; i++) {
        sum += temp_buffer[i];
    }
    
    return sum / count;
}

/**
 * @brief Main task for reading DS18B20
 */
void ds18b20_task(void *pvParameters)
{
    // Check for device presence
    vTaskDelay(pdMS_TO_TICKS(100));
    
    for (int i = 0; i < MAX_RETRIES; i++) {
        if (one_wire_reset()) {
            ESP_LOGI(TAG, "DS18B20 detected successfully!");
            break;
        } else {
            ESP_LOGD(TAG, "DS18B20 not detected, retrying... (%d/%d)", i + 1, MAX_RETRIES);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (i == MAX_RETRIES - 1) {
            ESP_LOGE(TAG, "No DS18B20 found on GPIO %d!", ONE_WIRE_PIN);
            ESP_LOGE(TAG, "Check wiring: VCC->3.3V, GND->GND, DATA->GPIO%d with 4.7k pull-up", 
                    ONE_WIRE_PIN);
        }
    }
    
    while (1) {
        float raw_temperature = ds18b20_read_temperature();

        if (raw_temperature > -100) { // Valid reading
            // Apply moving average filter
            float filtered_temperature = apply_moving_average(raw_temperature);
            ESP_LOGI(TAG, "Raw temperature: %.2f C", raw_temperature);
            ESP_LOGI(TAG, "Filtered temperature: %.2f C", filtered_temperature);
            if (fabs(filtered_temperature - raw_temperature) <= 5.0f){
                // Log the received sentence
                int temperature = (int)(raw_temperature * 100);
                uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
                temp_message_t msg;
                msg.temperature = temperature;
                msg.delta_t = current_time - temp_timestamp;
                temp_timestamp = current_time;
                
                if (ds18b20_message_queue != NULL) {
                    if (xQueueSend(ds18b20_message_queue, &msg, 0) != pdTRUE) {
                        ESP_LOGW(TAG, "Queue full, message dropped");
                    }
                }
            }
        } else {
            ESP_LOGE(TAG, "Failed to read temperature (error code: %.0f)", raw_temperature);
        }
        
        
        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
}

esp_err_t get_ds18b20_queue_message(temp_message_t *msg) {
    if (xQueueReceive(ds18b20_message_queue, msg, portMAX_DELAY) == pdTRUE) {
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}