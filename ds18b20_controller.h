#ifndef __DS18B20_CONTROLLER_H__
#define __DS18B20_CONTROLLER_H__

void ds18b20_init(int gpio_num, int poll_interval_ms);
void ds18b20_task(void *pvParameters);

#endif // __DS18B20_CONTROLLER_H__