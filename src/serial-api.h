/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef SERIAL_API_H
#define SERIAL_API_H

#include <stdint.h>

/* The only purpose of this for now is printing full messages for debug */
struct serial_api_port_state_s {
    uint8_t rx_buf[280];
    uint16_t rx_len;
    uint16_t rx_full_msg_len;
    uint8_t last_version;
    uint16_t rx_error_cnt;

    void (*cmd_rx_cb)(uint8_t cmd, const uint8_t *payload, uint8_t payload_len);
};

void serial_api_reset(struct serial_api_port_state_s *port);
void serial_api_rx_byte(struct serial_api_port_state_s *port, uint8_t byte);

#endif /* SERIAL_API_H */
