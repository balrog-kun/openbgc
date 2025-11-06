/* vim: set ts=4 sw=4 sts=4 et : */
#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include "util.h"
extern "C" {
#include "main.h"
}
#include "storage.h"

/* Support either the on-chip embedded flash or the external Microchip 24FC256
 * I2C EEPROM for storage.
 *
 * The internal flash is rated for 10k to 100k erase cycles.  The 24FC256 EEPROM
 * is rated for 1M write cycles.
 */
static obgc_i2c *i2c_bus;
static uint8_t i2c_addr;
static uint16_t i2c_eeprom_size;

struct storage_image_s {
    struct storage_header_s {
        uint16_t config_version;
        uint16_t config_len;
        uint16_t config_crc;
    } header;

    struct obgc_storage_config_s config;
};

CHECK_SIZE(storage_image_s, FLASH_PAGE_SIZE);

#define RAM_IMAGE_SIZE ((sizeof(struct storage_image_s) + 7) & ~7)

union {
    uint8_t bytes[RAM_IMAGE_SIZE];
    struct storage_image_s structure;
} ram_image;

__asm__(".globl config\n"
        ".set config, ram_image + 8\n");

static uint16_t crc16_ccitt(const struct obgc_storage_config_s *config_data) {
    const uint8_t *data = (const uint8_t *) config_data;
    uint16_t len = sizeof(struct obgc_storage_config_s);
    uint16_t crc = -1;
    int i;

    while (len--) {
        crc ^= (uint16_t) (*(uint8_t *) data++) << 8;

        for (i = 0; i < 8; i++)
            crc = (crc << 1) ^ ((crc >> 15) ? 0x1021 : 0);
    }

    return crc;
}

static uint16_t i2c_eeprom_image_addr(void) {
    /* Start somewhere towards the end of the chip but at a constant address.
     * So not RAM_IMAGE_SIZE from the end because that may vary with config
     * versions and the header would move making backwards compatibility more
     * difficult.
     */
    return (i2c_eeprom_size - FLASH_PAGE_SIZE) & ~63;
}

static int i2c_eeprom_read(int (*cb)(const struct storage_image_s *)) {
    union {
        uint8_t bytes[RAM_IMAGE_SIZE];
        struct storage_image_s structure;
    } tmp_image;
    uint8_t *pos = &tmp_image.bytes[0];
    uint16_t left = RAM_IMAGE_SIZE;
    uint16_t from_addr = i2c_eeprom_image_addr();

    while (left) {
        uint8_t chunk_size = min(min(left, (uint16_t) (64 - (from_addr & 63))), (uint16_t) I2C_BUFFER_LENGTH);
        int read_size = i2c_bus->requestFrom(i2c_addr, chunk_size, from_addr, 2, true);

        if (read_size != chunk_size) {
            char msg[128];
            sprintf(msg, "Short or failed read after %i bytes at 0x%04x",
                    RAM_IMAGE_SIZE - left + read_size, from_addr + read_size);
            error_print(msg);
            return -1;
        }

        left -= read_size;
        from_addr += read_size;
        while (read_size--)
            *pos++ = i2c_bus->read();
    }

    return cb(&tmp_image.structure);
}

#define EEPROM_PROGRAM_TIMEOUT_MS 500
static int i2c_eeprom_write(void) {
    uint8_t *pos = &ram_image.bytes[0];
    uint16_t left = RAM_IMAGE_SIZE;
    uint16_t to_addr = i2c_eeprom_image_addr();

    while (left) {
        unsigned long start_ms;
        bool ack = false;
        uint8_t chunk_size = min(left, (uint16_t) (64 - (to_addr & 63)));

        i2c_bus->beginTransmission(i2c_addr);
        if (i2c_bus->write((uint8_t) (to_addr >> 8)) != 1 ||
                i2c_bus->write((uint8_t) to_addr) != 1) {
            char msg[128];
            sprintf(msg, "address write() failed after %i bytes at 0x%04x",
                    RAM_IMAGE_SIZE - left, to_addr);
            error_print(msg);
        }

        left -= chunk_size;
        to_addr += chunk_size;
        while (chunk_size--)
            if (i2c_bus->write(*pos++) != 1) {
                char msg[128];
                i2c_bus->endTransmission();
                sprintf(msg, "write() failed after %i bytes at 0x%04x",
                        RAM_IMAGE_SIZE - left - chunk_size, to_addr - chunk_size);
                error_print(msg);
                return -1;
            }

        if (i2c_bus->endTransmission() != 0) {
            error_print("endTransmission() failed");
            return -1;
        }

        /* Start polling for end of programming */
        start_ms = millis();
        while (!ack && (unsigned long) (millis() - start_ms) < EEPROM_PROGRAM_TIMEOUT_MS) {
            i2c_bus->beginTransmission(i2c_addr);
            ack = i2c_bus->write(0);
            i2c_bus->endTransmission();
        }

        if (!ack) {
            char msg[128];
            sprintf(msg, "Programming ACK timeout after %i bytes at 0x%04x",
                    RAM_IMAGE_SIZE - left, to_addr);
            error_print(msg);
            return -1;
        }
    }

    return 0;
}

static const struct storage_image_s *storage_flash_addr(void) {
    /* Use the last page of on-chip flash for storage, banks not supported */
    uint16_t page_num = *(uint16_t *) FLASH_SIZE_DATA_REGISTER - 1;

    return (struct storage_image_s *) (unsigned long) (FLASH_BASE | (page_num * FLASH_PAGE_SIZE));
}

static int flash_read(int (*cb)(const struct storage_image_s *)) {
    const struct storage_image_s *flash_image = storage_flash_addr();

    return cb(flash_image);
}

static int flash_write(void) {
    const struct storage_image_s *flash_image = storage_flash_addr();
    uint32_t addr = (unsigned long) flash_image;

    uint32_t erase_ret = -1;
    FLASH_EraseInitTypeDef erase_init_info = {
        .TypeErase   = FLASH_TYPEERASE_PAGES,
        .PageAddress = addr,
        .NbPages     = 1,
    };
    int pos;

    if (!memcmp(&ram_image, flash_image, sizeof(struct storage_image_s)))
        return 0;

    if (HAL_FLASH_Unlock() != HAL_OK) {
        error_print("Flash unlock failed");
        return -1;
    }

    if (HAL_FLASHEx_Erase(&erase_init_info, &erase_ret) != HAL_OK) {
        char msg[128];
        sprintf(msg, "Flash page erase failed: %lu", erase_ret);
        error_print(msg);
        return -1;
    }

    for (pos = 0; pos < RAM_IMAGE_SIZE; pos += sizeof(uint64_t)) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + pos,
                    *(uint64_t *) &ram_image.bytes[pos]) != HAL_OK) {
            char msg[128];
            sprintf(msg, "Program DoubleWord failed: %lu", HAL_FLASH_GetError());
            error_print(msg);
            return -1;
        }
    }

    return 0;
}

static void update_header(void) {
    ram_image.structure.header.config_version = STORAGE_CONFIG_VERSION;
    ram_image.structure.header.config_len = sizeof(struct obgc_storage_config_s);
    ram_image.structure.header.config_crc = crc16_ccitt(&config);
}

static int storage_image_validate_read_cb(const struct storage_image_s *image) {
    /* TODO: might want to check this last (but sanity check length first) to detect
     * older configs with correct CRC to tell the caller to: try a conversion, or at
     * least refuse to overwrite the old config until explicitly acked by user.
     */
    if (image->header.config_version != STORAGE_CONFIG_VERSION) {
        error_print("Saved config version mismatch");
        return -1;
    }

    if (image->header.config_len != sizeof(struct obgc_storage_config_s)) {
        error_print("Saved config length mismatch");
        return -1;
    }

    if (image->header.config_crc != crc16_ccitt(&image->config)) {
        error_print("Saved config CRC mismatch");
        return -1;
    }

    memcpy(&ram_image, image, sizeof(struct storage_image_s));
    return 0;
}

static int storage_image_dump_cb(const struct storage_image_s *image) {
    uint16_t len = min((uint16_t) sizeof(struct storage_image_s), (uint16_t) 0xffff/*image->header.config_len*/);
    const uint8_t *pos = (const uint8_t *) image;

    while (len) {
        int num = min((uint16_t) 16, len);
        int i, j;
        char line[128];
        uint8_t line_len = 0;

        for (i = 0; i < num; i++)
            line_len += sprintf(line + line_len, "%s %02x", i == 8 ? " " : "", pos[i]);
        j = (num - i) * 3 + (i <= 8 ? 1 : 0) + 2;
        memset(line + line_len, 0, j);
        line_len += j;
        for (i = 0; i < num; i++)
            line[line_len++] = pos[i] >= 32 ? pos[i] : '.';
        line[line_len] = '\0';
        error_print(line);
        len -= num;
        pos += num;
    }

    return 0;
}

int storage_read(void) {
    if (i2c_bus)
        return i2c_eeprom_read(storage_image_validate_read_cb);
    else
        return flash_read(storage_image_validate_read_cb);
}

void storage_dump(void) {
    if (i2c_bus)
        i2c_eeprom_read(storage_image_dump_cb);
    else
        flash_read(storage_image_dump_cb);
}

int storage_write(void) {
    update_header();

    if (i2c_bus)
        return i2c_eeprom_write();
    else
        return flash_write();
}

void storage_init_internal_flash(void) {
    i2c_bus = NULL;
}

void storage_init_i2c_eeprom(uint8_t addr, obgc_i2c *i2c, uint16_t size) {
    i2c_bus = i2c;
    i2c_addr = addr;
    i2c_eeprom_size = size;
}
