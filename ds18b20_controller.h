#ifndef __DS18B20_CONTROLLER_H__
#define __DS18B20_CONTROLLER_H__

#include "freertos/queue.h"

// Message structure for queue
typedef struct {
    int temperature;
    uint32_t delta_t;
} temp_message_t;

void ds18b20_init(int gpio_num, int poll_interval_ms);
void ds18b20_task(void *pvParameters);
esp_err_t get_ds18b20_queue_message(temp_message_t *msg);

#endif // __DS18B20_CONTROLLER_H__