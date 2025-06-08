#ifndef INC_MQTTNETWORK_H_
#define INC_MQTTNETWORK_H_

#include "MQTTClient.h" // Assumes this header defines the Network struct
#include <stdint.h>     // For uint8_t

// The 'addr' parameter should now be a pointer to a uint8_t array representing the IP address.
// This design choice simplifies the network layer by offloading DNS resolution
// to the user or by requiring a pre-resolved IP.
int mqttnetwork_connect(Network* n, uint8_t* addr_ip, int port);
int mqttnetwork_read(Network* n, unsigned char* buffer, int len, int timeout_ms);
int mqttnetwork_write(Network* n, unsigned char* buffer, int len, int timeout_ms);
void mqttnetwork_disconnect(Network* n);

#endif /* INC_MQTTNETWORK_H_ */
