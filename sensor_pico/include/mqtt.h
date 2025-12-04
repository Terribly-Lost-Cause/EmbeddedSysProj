#ifndef MQTT_H
#define MQTT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// call this once AFTER Wi-Fi is up (or you can let it redo Wi-Fi if you want)
bool mqtt_init(void);

// send {"x":..,"y":..} to the PC endpoint over TCP
void publish_data(float x, float y);

#ifdef __cplusplus
}
#endif

#endif // MQTT_H
