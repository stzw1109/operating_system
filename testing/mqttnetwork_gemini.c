#include "mqttnetwork.h"
#include "socket.h"       // W5500 socket functions
#include "wizchip_conf.h" // W5500 configuration
#include <string.h>       // For memcpy

// Define the socket number to use for MQTT client
#define MQTT_SOCKET_NUM SOCK_TCPS // Using socket 0 (SOCK_TCPS is typically 0)
#define MQTT_LOCAL_PORT 5000      // Local port for the MQTT client

int mqtt_sock_fd = -1; // Global file descriptor for the MQTT socket

/**
 * @brief Establishes a TCP connection to the MQTT broker.
 * @param n: Pointer to the Network structure.
 * @param addr_ip: Pointer to a uint8_t array containing the broker's IP address.
 * @param port: The port number of the MQTT broker (e.g., 1883 for non-TLS).
 * @retval 0 on success, -1 on failure.
 */
int mqttnetwork_connect(Network* n, uint8_t* addr_ip, int port) {
    // If a socket is already open, close it before opening a new one
    if (mqtt_sock_fd != -1) {
        close(mqtt_sock_fd);
        mqtt_sock_fd = -1;
    }

    // Open a TCP socket
    mqtt_sock_fd = socket(MQTT_SOCKET_NUM, Sn_MR_TCP, MQTT_LOCAL_PORT, 0);
    if (mqtt_sock_fd < 0) {
        // Handle socket creation error (e.g., no available sockets)
        return -1;
    }

    // Attempt to connect to the broker
    if (connect(MQTT_SOCKET_NUM, addr_ip, (uint16_t)port) != SOCK_OK) {
        close(mqtt_sock_fd); // Close socket on connection failure
        mqtt_sock_fd = -1;
        return -1;
    }

    // Set up the Network structure with function pointers
    n->my_socket = MQTT_SOCKET_NUM; // Store the socket descriptor
    n->mqttread = mqttnetwork_read;
    n->mqttwrite = mqttnetwork_write;
    n->disconnect = mqttnetwork_disconnect;

    return 0; // Success
}

/**
 * @brief Reads data from the TCP socket.
 * @param n: Pointer to the Network structure.
 * @param buffer: Buffer to store received data.
 * @param len: Maximum number of bytes to read.
 * @param timeout_ms: Timeout in milliseconds (not used by W5500 recv, which is blocking).
 * @retval Number of bytes read, or -1 on error.
 */
int mqttnetwork_read(Network* n, unsigned char* buffer, int len, int timeout_ms) {
    // W5500's recv is typically blocking until data is available or an error occurs.
    // The timeout_ms parameter is not directly used by the W5500's recv function.
    // If non-blocking read with timeout is needed, it would require a timer or osDelay
    // loop with check for Sn_SR_RECV (received data) socket status.
    (void)timeout_ms; // Suppress unused parameter warning

    return recv(n->my_socket, buffer, (uint16_t)len);
}

/**
 * @brief Writes data to the TCP socket.
 * @param n: Pointer to the Network structure.
 * @param buffer: Buffer containing data to send.
 * @param len: Number of bytes to send.
 * @param timeout_ms: Timeout in milliseconds (not used by W5500 send, which is blocking).
 * @retval Number of bytes written, or -1 on error.
 */
int mqttnetwork_write(Network* n, unsigned char* buffer, int len, int timeout_ms) {
    // W5500's send is typically blocking until data is sent or an error occurs.
    (void)timeout_ms; // Suppress unused parameter warning

    return send(n->my_socket, buffer, (uint16_t)len);
}

/**
 * @brief Disconnects and closes the TCP socket.
 * @param n: Pointer to the Network structure.
 * @retval None
 */
void mqttnetwork_disconnect(Network* n) {
    disconnect(n->my_socket); // Disconnect TCP connection
    close(n->my_socket);      // Close the socket
    mqtt_sock_fd = -1;         // Reset global socket descriptor
}
