/*
 * This file is part of AtomGL.
 *
 * Copyright 2024 Davide Bettio <davide@uninstall.it>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <driver/gpio.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/task.h>

#include <context.h>
#include <interop.h>

#include <i2c_driver.h>

#include "display_common.h"

#define TAG "SSD1306"

#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define PAGE_HEIGHT 8
#define PAGES_NUM 8
#define CHAR_WIDTH 8

#define I2C_ADDRESS 0x3C

#define CTRL_BYTE_CMD_SINGLE 0x80
#define CTRL_BYTE_CMD_STREAM 0x00
#define CTRL_BYTE_DATA_STREAM 0x40

#define CMD_DISPLAY_INVERTED 0xA7
#define CMD_DISPLAY_ON 0xAF
#define CMD_SET_SEGMENT_REMAP 0xA1
#define CMD_SET_COM_SCAN_MODE 0xC8
#define CMD_SET_CHARGE_PUMP 0x8D

#define CMD_DISPLAY_OFF                     0xAE
#define CMD_SET_DISPLAY_CLOCK_DIV_OSC_FREQ  0xD5
#define CMD_SET_MULTIPLEX_RATIO             0xA8
#define CMD_SET_DISPLAY_OFFSET              0xD3
#define CMD_SET_DISPLAY_START_LINE          0x40
#define CMD_SET_MEMORY_ADDR_MODE            0x20
#define CMD_SET_COLUMN_ADDR_LOWER           0x00
#define CMD_SET_COLUMN_ADDR_HIGHER          0x10
#define CMD_SET_COM_PINS_HW_CONFIG          0xDA
#define CMD_SET_CONTRAST_CONTROL            0x81
#define CMD_SET_PRECHARGE_PERIOD            0xD9
#define CMD_SET_VCOMH_DESELECT_LEVEL        0xDB
#define CMD_ENTIRE_DISPLAY_RAM_CONTINUE     0xA4
#define CMD_NORMAL_DISPLAY                  0xA6
#define CMD_INTERNAL_IREF_SELECT            0xAD
#define CMD_SET_PAGE_ADDR                   0xB0

#define CMD_SET_SEGMENT_REMAP_FLIPPED 0xA1
#define CMD_SET_COM_SCAN_MODE_FLIPPED 0xC8

#define CMD_SET_COLUMN_ADDRESS_RANGE        0x21

// TODO: let's change name, since also non SPI display are supported now
struct SPI
{
    term i2c_host;
    bool is_sh1106;
    bool is_ssd1315;
    Context *ctx;
};

static void do_update(Context *ctx, term display_list);

#include "font.c"
#include "display_items.h"
#include "draw_common.h"
#include "monochrome.h"
#include "message_helpers.h"

static void do_update(Context *ctx, term display_list)
{
    int proper;
    int len = term_list_length(display_list, &proper);

    BaseDisplayItem *items = malloc(sizeof(BaseDisplayItem) * len);

    term t = display_list;
    for (int i = 0; i < len; i++) {
        init_item(&items[i], term_get_list_head(t), ctx);
        t = term_get_list_tail(t);
    }

    int screen_width = DISPLAY_WIDTH;
    int screen_height = DISPLAY_HEIGHT;
    struct SPI *spi = ctx->platform_data;

    int memsize = (DISPLAY_WIDTH * (PAGE_HEIGHT + 1)) / sizeof(uint8_t);
    uint8_t *buf = malloc(memsize);
    memset(buf, 0, memsize);

    i2c_port_t i2c_num;
    if (i2c_driver_acquire(spi->i2c_host, &i2c_num, ctx->global) != I2CAcquireOk) {
        fprintf(stderr, "Invalid I2C peripheral\n");
        return;
    }

    for (int ypos = 0; ypos < screen_height; ypos++) {
        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = draw_x(buf, xpos, ypos, items, len);
            xpos += drawn_pixels;
        }

        uint8_t *out_buf = buf + (DISPLAY_WIDTH / 8);
        for (int i = 0; i < DISPLAY_WIDTH; i++) {
            out_buf[i] |= ((buf[i / 8] >> (i % 8)) & 1) << (ypos % 8);
        }

        if ((ypos % PAGE_HEIGHT) == (PAGE_HEIGHT - 1)) {
            i2c_cmd_handle_t cmd;
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

            i2c_master_write_byte(cmd, CTRL_BYTE_CMD_SINGLE, true);
            i2c_master_write_byte(cmd, CMD_SET_PAGE_ADDR | (ypos / PAGE_HEIGHT), true);
            
            // --- X-OFFSET FIX START ---
            if (spi->is_sh1106) {
                // For SH1106, 128-pixel displays often start at internal GDDRAM column 2.
                // So, to have your buffer's column 0 show at the physical screen's column 0,
                // you would need to set the internal column address to 2.
                // If your SH1106 works as is with 0x00, 0x10, then it's either not a common SH1106
                // or your draw_x function somehow compensates. We'll leave it as is if working.
                i2c_master_write_byte(cmd, CTRL_BYTE_CMD_SINGLE, true);
                i2c_master_write_byte(cmd, 0x00, true); // Lower nibble of Column Address 0
                i2c_master_write_byte(cmd, CTRL_BYTE_CMD_SINGLE, true);
                i2c_master_write_byte(cmd, 0x10, true); // Higher nibble of Column Address 0
            }
            if (spi->is_ssd1315) {
                // With CMD_SET_COLUMN_ADDRESS_RANGE (0x21) in init,
                // we just need to reset the pointer to 0.
                i2c_master_write_byte(cmd, CTRL_BYTE_CMD_SINGLE, true);
                i2c_master_write_byte(cmd, CMD_SET_COLUMN_ADDR_LOWER, true); // Lower nibble of Column Address 0
                i2c_master_write_byte(cmd, CTRL_BYTE_CMD_SINGLE, true);
                i2c_master_write_byte(cmd, CMD_SET_COLUMN_ADDR_HIGHER, true); // Higher nibble of Column Address 0
            }
            // --- X-OFFSET FIX END ---
            
            i2c_master_write_byte(cmd, CTRL_BYTE_DATA_STREAM, true);

            if (spi->is_sh1106) {
                // add 2 empty pages on sh1106 since it can have up to 132 pixels
                // and 128 pixel screen starts at (2, 0)
                i2c_master_write_byte(cmd, 0, true);
                i2c_master_write_byte(cmd, 0, true);
            }

            for (uint8_t j = 0; j < DISPLAY_WIDTH; j++) {
                i2c_master_write_byte(cmd, out_buf[j], true);
            }

            // no need to send the last 2 page, the position will be set on next line again
            // if (spi->is_sh1106) {
            //    i2c_master_write_byte(cmd, 0, true);
            //    i2c_master_write_byte(cmd, 0, true);
            // }

            i2c_master_stop(cmd);
            esp_err_t res_i2c = i2c_master_cmd_begin(i2c_num, cmd, 10 / portTICK_PERIOD_MS);
            if (res_i2c != ESP_OK) {
                 ESP_LOGE(TAG, "I2C write failed for display update: 0x%.2X", res_i2c);
            }
            i2c_cmd_link_delete(cmd);

            memset(buf, 0, memsize);
        }
    }

    i2c_driver_release(spi->i2c_host, ctx->global);

    free(buf);
    destroy_items(items, len);
}

static void display_init(Context *ctx, term opts)
{
    GlobalContext *glb = ctx->global;

    term i2c_host
        = interop_kv_get_value_default(opts, ATOM_STR("\x8", "i2c_host"), term_invalid_term(), glb);
    if (i2c_host == term_invalid_term()) {
        ESP_LOGE(TAG, "Missing i2c_host config option.");
        return;
    }

    bool invert = interop_kv_get_value(opts, ATOM_STR("\x6", "invert"), glb) == TRUE_ATOM;

    display_messages_queue = xQueueCreate(32, sizeof(Message *));

    struct SPI *spi = malloc(sizeof(struct SPI));
    ctx->platform_data = spi;

    spi->ctx = ctx;

    term compat_value_term = interop_kv_get_value_default(opts, ATOM_STR("\xA", "compatible"), term_nil(), ctx->global);
    int str_ok;
    char *compat_string = interop_term_to_string(compat_value_term, &str_ok);
    if (str_ok && compat_string) {
        spi->is_sh1106 = !strcmp(compat_string, "sino-wealth,sh1106");
        spi->is_ssd1315 = !strcmp(compat_string, "solomon-systech,ssd1315");
        free(compat_string);
    } else {
        return;
    }

    int reset_gpio;
    if (!display_common_gpio_from_opts(opts, ATOM_STR("\x5", "reset"), &reset_gpio, glb)) {
        ESP_LOGI(TAG, "Reset GPIO not configured.");
    } else {
        gpio_set_direction(reset_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(reset_gpio, 0);
        vTaskDelay(50 / portTICK_PERIOD_MS);
        gpio_set_level(reset_gpio, 1);
    }

    i2c_port_t i2c_num;
    if (i2c_driver_acquire(i2c_host, &i2c_num, glb) != I2CAcquireOk) {
        fprintf(stderr, "Invalid I2C peripheral\n");
        return;
    }
    spi->i2c_host = i2c_host;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, CTRL_BYTE_CMD_STREAM, true);

    if (spi->is_ssd1315) {
        // === SSD1315 SPECIFIC INIT (Matching u8x8 driver that works) ===

        // 1. Display OFF (Sleep Mode)
        i2c_master_write_byte(cmd, CMD_DISPLAY_OFF, true); // 0xAE

        // 2. Set Display Clock Divide Ratio / Oscillator Frequency
        i2c_master_write_byte(cmd, CMD_SET_DISPLAY_CLOCK_DIV_OSC_FREQ, true); // 0xD5
        i2c_master_write_byte(cmd, 0x90, true); // Matching u8x8 value (changed from 0x80)

        // 3. Set Multiplex Ratio
        i2c_master_write_byte(cmd, CMD_SET_MULTIPLEX_RATIO, true); // 0xA8
        i2c_master_write_byte(cmd, 0x3F, true); // 1/64 duty cycle (for 128x64 display)

        // 4. Set Display Offset
        i2c_master_write_byte(cmd, CMD_SET_DISPLAY_OFFSET, true); // 0xD3
        i2c_master_write_byte(cmd, 0x00, true); // No vertical offset

        // 5. Set Display Start Line
        i2c_master_write_byte(cmd, CMD_SET_DISPLAY_START_LINE, true); // 0x40 (Start line 0)

        // 6. Set Memory Addressing Mode
        i2c_master_write_byte(cmd, CMD_SET_MEMORY_ADDR_MODE, true); // 0x20
        i2c_master_write_byte(cmd, 0x02, true); // Page Addressing Mode

        // --- NEW: Set Column Address Range (0 to 127) once at init ---
        i2c_master_write_byte(cmd, CTRL_BYTE_CMD_SINGLE, true); // Send as single command
        i2c_master_write_byte(cmd, CMD_SET_COLUMN_ADDRESS_RANGE, true); // 0x21
        i2c_master_write_byte(cmd, 0x00, true); // Start column 0
        i2c_master_write_byte(cmd, 0x7F, true); // End column 127 (DISPLAY_WIDTH - 1)

        // 7. Set Segment Remap (Matching u8x8 value)
        i2c_master_write_byte(cmd, CMD_SET_SEGMENT_REMAP_FLIPPED, true); // 0xA1

        // 8. Set COM Output Scan Direction (Matching u8x8 value)
        i2c_master_write_byte(cmd, CMD_SET_COM_SCAN_MODE_FLIPPED, true); // 0xC8

        // 9. Set COM Pins Hardware Configuration
        i2c_master_write_byte(cmd, CMD_SET_COM_PINS_HW_CONFIG, true); // 0xDA
        i2c_master_write_byte(cmd, 0x12, true); // Alternative COM pin config, disable Left/Right remap

        // 10. Set Contrast Control
        i2c_master_write_byte(cmd, CMD_SET_CONTRAST_CONTROL, true); // 0x81
        i2c_master_write_byte(cmd, 0x7F, true); // Mid-range contrast

        // 11. Set Pre-charge Period (Matching u8x8 value)
        i2c_master_write_byte(cmd, CMD_SET_PRECHARGE_PERIOD, true); // 0xD9
        i2c_master_write_byte(cmd, 0x22, true); // Matching u8x8 value (changed from 0xF1)

        // 12. Set VCOMH Deselect Level (Matching u8x8 value)
        i2c_master_write_byte(cmd, CMD_SET_VCOMH_DESELECT_LEVEL, true); // 0xDB
        i2c_master_write_byte(cmd, 0x30, true); // Matching u8x8 value (changed from 0x20)

        // 13. Internal IREF Selection (SSD1315 SPECIFIC!)
        i2c_master_write_byte(cmd, CMD_INTERNAL_IREF_SELECT, true); // 0xAD
        i2c_master_write_byte(cmd, 0x10, true); // Enable internal IREF, 19uA setting

        // 14. Set Charge Pump
        i2c_master_write_byte(cmd, CMD_SET_CHARGE_PUMP, true); // 0x8D
        i2c_master_write_byte(cmd, 0x14, true); // Enable charge pump (7.5V setting)

        // 15. Entire Display ON (Resume to GDDRAM content)
        i2c_master_write_byte(cmd, CMD_ENTIRE_DISPLAY_RAM_CONTINUE, true); // 0xA4

        // 16. Normal / Inverse Display
        if (invert) {
            i2c_master_write_byte(cmd, CMD_DISPLAY_INVERTED, true); // 0xA7
        } else {
            i2c_master_write_byte(cmd, CMD_NORMAL_DISPLAY, true); // 0xA6 (Explicitly normal)
        }

        // 17. Display ON
        i2c_master_write_byte(cmd, CMD_DISPLAY_ON, true); // 0xAF

    } else {
        // === ORIGINAL SSD1306 / SH1106 INIT ===
        // Note: CTRL_BYTE_CMD_STREAM is already sent above.
        i2c_master_write_byte(cmd, CMD_SET_CHARGE_PUMP, true);
        i2c_master_write_byte(cmd, 0x14, true);
        i2c_master_write_byte(cmd, CMD_SET_SEGMENT_REMAP_FLIPPED, true); // Assume 0xA1 for SSD1306
        i2c_master_write_byte(cmd, CMD_SET_COM_SCAN_MODE_FLIPPED, true); // Assume 0xC8 for SSD1306
        if (invert) {
            i2c_master_write_byte(cmd, CMD_DISPLAY_INVERTED, true);
        }
        i2c_master_write_byte(cmd, CMD_DISPLAY_ON, true);
    }

    i2c_master_stop(cmd);


    esp_err_t res = i2c_master_cmd_begin(i2c_num, cmd, 50 / portTICK_PERIOD_MS);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "ssd1306 OLED configuration failed. error: 0x%.2X", res);
    } else {
        xTaskCreate(process_messages, "display", 10000, spi, 1, NULL);
    }

    i2c_cmd_link_delete(cmd);
    i2c_driver_release(i2c_host, glb);
}

Context *ssd1306_display_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);
    ctx->native_handler = display_driver_consume_mailbox;
    display_init(ctx, opts);

    return ctx;
}
