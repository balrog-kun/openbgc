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

static void update_header(void) {
    ram_image.structure.header.config_version = STORAGE_CONFIG_VERSION;
    ram_image.structure.header.config_len = sizeof(struct obgc_storage_config_s);
    ram_image.structure.header.config_crc = crc16_ccitt(&config);
}

static const struct storage_image_s *storage_flash_addr(void) {
    /* Use the last page of on-chip flash for storage, banks not supported */
    /* TODO: eventually switch to using the external 32kB flash chip */
    uint16_t page_num = *(uint16_t *) FLASH_SIZE_DATA_REGISTER - 1;

    return (struct storage_image_s *) (unsigned long) (FLASH_BASE | (page_num * FLASH_PAGE_SIZE));
}

int storage_read(void) {
    const struct storage_image_s *flash_image = storage_flash_addr();

    /* TODO: might want to check this last (but sanity check length first) to detect
     * older configs with correct CRC to tell the caller to: try a conversion, or at
     * least refuse to overwrite the old config until explicitly acked by user.
     */
    if (flash_image->header.config_version != STORAGE_CONFIG_VERSION) {
        error_print("Flash config version mismatch");
        return -1;
    }

    if (flash_image->header.config_len != sizeof(struct obgc_storage_config_s)) {
        error_print("Flash config length mismatch");
        return -1;
    }

    if (flash_image->header.config_crc != crc16_ccitt(&flash_image->config)) {
        error_print("Flash config CRC mismatch");
        return -1;
    }

    memcpy(&ram_image, flash_image, sizeof(struct storage_image_s));
    return 0;
}

int storage_write(void) {
    const struct storage_image_s *flash_image = storage_flash_addr();
    uint32_t addr = (unsigned long) flash_image;

    uint32_t erase_ret = -1;
    FLASH_EraseInitTypeDef erase_init_info = {
        .TypeErase   = FLASH_TYPEERASE_PAGES,
        .PageAddress = addr,
        .NbPages     = 1,
    };
    int pos;

    update_header();

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

void storage_init_internal_flash(void) {
    i2c_bus = NULL;
}

void storage_init_i2c_eeprom(uint8_t addr, obgc_i2c *i2c, uint32_t size) {
    i2c_bus = i2c;
    i2c_addr = addr;
}
