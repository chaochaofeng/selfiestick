#include <stdlib.h>
#include <sys/cdefs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "lcd_panel.st7735s";

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    uint8_t fb_bits_per_pixel;
    uint8_t rotation;
	uint8_t colmod_cal;
	int x_gap, y_gap;
} st7735s_panel_t;

static esp_err_t panel_st7735s_del(esp_lcd_panel_t *panel)
{
    st7735s_panel_t *st7735s = __containerof(panel, st7735s_panel_t, base);

    if (st7735s->reset_gpio_num >= 0) {
        gpio_reset_pin(st7735s->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del st7735s panel @%p", st7735s);
    free(st7735s);
    return ESP_OK;

}
static esp_err_t panel_st7735s_reset(esp_lcd_panel_t *panel)
{
    st7735s_panel_t *st7735s = __containerof(panel, st7735s_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7735s->io;

    // perform hardware reset
    if (st7735s->reset_gpio_num >= 0) {
        gpio_set_level(st7735s->reset_gpio_num, st7735s->reset_level);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(st7735s->reset_gpio_num, !st7735s->reset_level);
        vTaskDelay(pdMS_TO_TICKS(100));
    } else { // perform software reset
        esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5m before sending new command
    }

    return ESP_OK;
}

static esp_err_t panel_st7735s_init(esp_lcd_panel_t *panel)
{
    st7735s_panel_t *st7735s = __containerof(panel, st7735s_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7735s->io;

    // LCD goes into sleep mode and display will be turned off after power on reset, exit sleep mode first
    esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));
    esp_lcd_panel_io_tx_param(io, 0xb1, (uint8_t[]) {
        0x05, 0x3c, 0x3c,
    }, 3);
    esp_lcd_panel_io_tx_param(io, 0xb2, (uint8_t[]) {
        0x05, 0x3c, 0x3c,
    }, 3);

    esp_lcd_panel_io_tx_param(io, 0xb3, (uint8_t[]) {
        0x05, 0x3c, 0x3c,0x05, 0x3c, 0x3c,
    }, 6);

    esp_lcd_panel_io_tx_param(io, 0xb4, (uint8_t[]) {
        0x03,
    }, 1);

    esp_lcd_panel_io_tx_param(io, 0xc0, (uint8_t[]) {
        0xab, 0x0b, 0x04,
    }, 3);

	esp_lcd_panel_io_tx_param(io, 0xc1, (uint8_t[]) {
		0xc5,
	}, 1);
	esp_lcd_panel_io_tx_param(io, 0xc2, (uint8_t[]) {
		0x0d, 0x00,
	}, 2);
	esp_lcd_panel_io_tx_param(io, 0xc3, (uint8_t[]) {
		0x8d, 0x6a,
	}, 2);
	esp_lcd_panel_io_tx_param(io, 0xc4, (uint8_t[]) {
		0x8d, 0xee,
	}, 2);
	esp_lcd_panel_io_tx_param(io, 0xc5, (uint8_t[]) {
		0x0f,
	}, 1);
	esp_lcd_panel_io_tx_param(io, 0xe0, (uint8_t[]) {
 		0x07,0x0E,0x08,0x07,
		0x10,0x07,0x02,0x07,
		0x09,0x0F,0x25,0x36,
		0x00,0x08,0x04,0x10,
	}, 16);
	esp_lcd_panel_io_tx_param(io, 0xe1, (uint8_t[]) {
		0x0A,0x0D,0x08,0x07,
		0x0F,0x07,0x02,0x07,
		0x09,0x0F,0x25,0x35,
		0x00,0x09,0x04,0x10,
	}, 16);
	esp_lcd_panel_io_tx_param(io, 0xfc, (uint8_t[]) {
		0x80,
	}, 1);
	esp_lcd_panel_io_tx_param(io, 0x3a, (uint8_t[]) {
		0x05,
	}, 1);
	esp_lcd_panel_io_tx_param(io, 0x36, (uint8_t[]) {
		0xA8,
	}, 1);

    return ESP_OK;
}

static esp_err_t panel_st7735s_draw_bitmap(esp_lcd_panel_t *panel,
	int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    st7735s_panel_t *st7735s = __containerof(panel, st7735s_panel_t, base);
    esp_lcd_panel_io_handle_t io = st7735s->io;

    esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, (uint8_t[]) {
        (x_start + st7735s->x_gap) >> 8,
		 x_start + st7735s->x_gap,
		(x_end   + st7735s->x_gap - 1) >> 8,
         x_end   + st7735s->x_gap - 1,
    }, 4);
    esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, (uint8_t[]) {
		(y_start + st7735s->y_gap) >> 8,
         y_start + st7735s->y_gap,
        (y_end   + st7735s->y_gap - 1) >> 8,
         y_end   + st7735s->y_gap - 1,
    }, 4);

	// transfer frame buffer
	size_t len = (x_end - x_start) * (y_end - y_start) * st7735s->fb_bits_per_pixel / 8;
	esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len);
	
	return ESP_OK;
}
static esp_err_t panel_st7735s_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
	st7735s_panel_t *st7735s = __containerof(panel, st7735s_panel_t, base);
	esp_lcd_panel_io_handle_t io = st7735s->io;

	int command = 0;
	if (invert_color_data) {
		command = LCD_CMD_INVON;
	} else {
		command = LCD_CMD_INVOFF;
	}
	esp_lcd_panel_io_tx_param(io, command, NULL, 0);
	return ESP_OK;
}
static esp_err_t panel_st7735s_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
	return ESP_OK;
}
static esp_err_t panel_st7735s_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
	return ESP_OK;
}
static esp_err_t panel_st7735s_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
	st7735s_panel_t *st7735s = __containerof(panel, st7735s_panel_t, base);

	st7735s->x_gap = x_gap;
	st7735s->y_gap = y_gap;
	return ESP_OK;
}
static esp_err_t panel_st7735s_disp_off(esp_lcd_panel_t *panel, bool off)
{
	st7735s_panel_t *st7735s = __containerof(panel, st7735s_panel_t, base);
	esp_lcd_panel_io_handle_t io = st7735s->io;

    int command = 0;
    if (off) {
        command = LCD_CMD_DISPOFF;
    } else {
        command = LCD_CMD_DISPON;
    }
    esp_lcd_panel_io_tx_param(io, command, NULL, 0);

	return ESP_OK;
}

esp_err_t esp_lcd_new_panel_st7735s(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    st7735s_panel_t *st7735s = NULL;
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    st7735s = calloc(1, sizeof(st7735s_panel_t));
    ESP_GOTO_ON_FALSE(st7735s, ESP_ERR_NO_MEM, err, TAG, "no mem for st7735s panel");

    if (panel_dev_config->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    uint8_t fb_bits_per_pixel = 0;
    switch (panel_dev_config->bits_per_pixel) {
    case 16: // RGB565
        //st7735s->colmod_cal = 0x55;
        fb_bits_per_pixel = 16;
        break;
    case 18: // RGB666
        //st7735s->colmod_cal = 0x66;
        // each color component (R/G/B) should occupy the 6 high bits of a byte, which means 3 full bytes are required for a pixel
        fb_bits_per_pixel = 24;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    st7735s->io = io;
    st7735s->fb_bits_per_pixel = fb_bits_per_pixel;
    st7735s->reset_gpio_num = panel_dev_config->reset_gpio_num;
    st7735s->reset_level = panel_dev_config->flags.reset_active_high;
    st7735s->base.del = panel_st7735s_del;
    st7735s->base.reset = panel_st7735s_reset;
    st7735s->base.init = panel_st7735s_init;
    st7735s->base.draw_bitmap = panel_st7735s_draw_bitmap;
    st7735s->base.invert_color = panel_st7735s_invert_color;
    st7735s->base.set_gap = panel_st7735s_set_gap;
    st7735s->base.mirror = panel_st7735s_mirror;
    st7735s->base.swap_xy = panel_st7735s_swap_xy;
    st7735s->base.disp_off = panel_st7735s_disp_off;
    *ret_panel = &(st7735s->base);
    ESP_LOGD(TAG, "new st7735s panel @%p", st7735s);

    return ESP_OK;

err:
    if (st7735s) {
        if (panel_dev_config->reset_gpio_num >= 0) {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(st7735s);
    }
    return ret;
}


