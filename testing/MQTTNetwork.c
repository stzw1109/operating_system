#include "mqttnetwork.h"
#include "socket.h"
#include "wizchip_conf.h"

#define MQTT_SOCKET SOCK_TCPS
#define MQTT_LOCAL_PORT 5000

int mqtt_sock = -1;

int mqttnetwork_connect(Network* n, char* addr, int port) {
    uint8_t destip[4] = {192, 168, 137, 157}; // Replace with your broker IP
    mqtt_sock = socket(MQTT_SOCKET, Sn_MR_TCP, MQTT_LOCAL_PORT, 0);
    if (mqtt_sock < 0) return -1;

    if (connect(MQTT_SOCKET, destip, port) != SOCK_OK)
        return -1;

    n->my_socket = MQTT_SOCKET;
    n->mqttread = mqttnetwork_read;
    n->mqttwrite = mqttnetwork_write;
    n->disconnect = mqttnetwork_disconnect;

    return 0;
}

int mqttnetwork_read(Network* n, unsigned char* buffer, int len, int timeout_ms) {
    (void)timeout_ms;
    return recv(n->my_socket, buffer, len);
}

int mqttnetwork_write(Network* n, unsigned char* buffer, int len, int timeout_ms) {
    (void)timeout_ms;
    return send(n->my_socket, buffer, len);
}

void mqttnetwork_disconnect(Network* n) {
    disconnect(n->my_socket);
    close(n->my_socket);
}
