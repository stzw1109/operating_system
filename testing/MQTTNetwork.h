#ifndef INC_MQTTNETWORK_H_
#define INC_MQTTNETWORK_H_

#include "MQTTClient.h"

int mqttnetwork_connect(Network* n, char* addr, int port);
int mqttnetwork_read(Network* n, unsigned char* buffer, int len, int timeout_ms);
int mqttnetwork_write(Network* n, unsigned char* buffer, int len, int timeout_ms);
void mqttnetwork_disconnect(Network* n);

#endif /* INC_MQTTNETWORK_H_ */
