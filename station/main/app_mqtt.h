#ifndef __APP_MQTT_H
#define __APP_MQTT_H
#include <stdint.h>
typedef void(*mqtt_data_pt_t)(uint8_t *data,uint16_t length);
void mqtt_app_start(void);
void mqtt_data_pt_set_callback(void *cb);
void publish_state_to_mqtt(int onoff_state, uint32_t io_num);
#endif