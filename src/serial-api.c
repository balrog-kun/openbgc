/* vim: set ts=4 sw=4 sts=4 et : */
#include <Arduino.h> /* For micros(), delayMicroseconds() */
#include <stdio.h>

#include "main.h"
#include "util.h"

#include "serial-api.h"

#define V1_START '>'
#define V2_START '$'

void serial_api_reset(struct serial_api_port_state_s *port) {
    port->rx_len = 0;
}

void handle_cmd(struct serial_api_port_state_s *port, uint8_t cmd,
        const uint8_t *payload, uint8_t payload_len) {
    if (port->cmd_rx_cb)
        port->cmd_rx_cb(cmd, payload, payload_len);
}

static uint16_t sbgc_crc16(uint8_t *data, uint16_t len) {
    uint16_t crc = 0;
    int i;

    while (len--) {
        /* Do the whole calc with bits reversed, 8005 -> a001 */
        crc ^= *data++;

        for (i = 0; i < 8; i++)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xa001 : 0);
    }

    /* Reverse them back except for the byteswap */
    crc = (crc & 0x0f0f) << 4 | (crc & 0xf0f0) >> 4;
    crc = (crc & 0x3333) << 2 | (crc & 0xcccc) >> 2;
    crc = (crc & 0x5555) << 1 | (crc & 0xaaaa) >> 1;
    return crc;
}

void serial_api_rx_byte(struct serial_api_port_state_s *port, uint8_t byte) {
    if (port->rx_len == 0 && byte != V1_START && byte != V2_START)
        return;

    port->rx_buf[port->rx_len++] = byte;

    if (port->rx_len < 4)
        return;

    if (port->rx_len == 4) {
        if ((uint8_t) (port->rx_buf[1] + port->rx_buf[2]) != port->rx_buf[3]) {
            port->rx_error_cnt++;
            while (--port->rx_len) {
                memmove(port->rx_buf, port->rx_buf + 1, port->rx_len);
                if (port->rx_buf[0] == V1_START || port->rx_buf[0] == V2_START)
                    return;
            }
            return;
        }

        port->rx_full_msg_len = 4 + port->rx_buf[2] + (port->rx_buf[0] == V2_START ? 2 : 1);
        return;
    }

    if (port->rx_len < port->rx_full_msg_len)
        return;

    if (port->rx_buf[0] == V1_START) {
        uint8_t sum, pos;

        for (pos = 0; pos < port->rx_buf[2]; pos)
            sum += port->rx_buf[4 + pos++];

        if (port->rx_buf[4 + pos] != sum) {
            port->rx_error_cnt++;
            serial_api_reset(port);
            return;
        }
    } else {
        const uint8_t *crc = port->rx_buf + (4 + port->rx_buf[2]);

        if (sbgc_crc16(port->rx_buf + 1, 3 + port->rx_buf[2]) != (uint16_t) crc[0] << 8 | crc[1]) {
            port->rx_error_cnt++;
            serial_api_reset(port);
            return;
        }
    }

    port->last_version =  port->rx_buf[0];
    handle_cmd(port, port->rx_buf[1], port->rx_buf + 4, port->rx_buf[2]);
    serial_api_reset(port);
}

void serial_api_tx_cmd(struct serial_api_port_state_s *port, uint8_t cmd,
        const uint8_t *payload, uint8_t payload_len) {
    uint8_t buf[payload_len + 6];

    if (!port->bytes_tx_cb)
        return;

    buf[0] = port->last_version ?: V2_START;
    buf[1] = cmd;
    buf[2] = payload_len;
    buf[3] = buf[1] + buf[2];
    memcpy(buf + 4, payload, payload_len);

    if (buf[0] == V1_START) {
        uint8_t sum, pos;

        for (pos = 0; pos < buf[2];)
            sum += buf[4 + pos++];

        buf[4 + pos++] = sum;
        port->bytes_tx_cb(buf, 4 + pos);
    } else {
        uint16_t crcval = sbgc_crc16(buf + 1, 3 + buf[2]);

        buf[4 + buf[2]] = crcval >> 8;
        buf[5 + buf[2]] = crcval;
        port->bytes_tx_cb(buf, 6 + buf[2]);
    }
}
