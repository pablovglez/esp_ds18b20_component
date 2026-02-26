#ifndef __DS18B20_CONTROLLER_H__
#define __DS18B20_CONTROLLER_H__

#include "freertos/queue.h"

void ds18b20_init(int gpio_num, int poll_interval_ms);
void ds18b20_task(void *pvParameters);
void temperature_processing_task(void *pvParameters);
QueueHandle_t get_ds18b20_message_queue(void);

#endif // __DS18B20_CONTROLLER_H__