/*
 * SPDX-FileCopyrightText: 2021-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"

#include "key_input_lvgl.h"

static const char *TAG = "lvgl_spi";

// Using SPI2 in the example
#define LCD_HOST  SPI2_HOST

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)

#define LCD_SCLK 6
#define LCD_MOSI 7
#define LCD_CS   10
#define LCD_DC   8
#define LCD_RST  9

// The pixel number in horizontal and vertical
#define LCD_H_RES              160
#define LCD_V_RES              80

// Bit number used to represent command and parameter

#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2

esp_lcd_panel_io_handle_t io_handle = NULL;

static bool lcd_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    struct _lv_disp_drv_t *disp_drv = (struct _lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_drv);
    return false;
}

static void lcd_lvgl_flush_cb(struct _lv_disp_drv_t * disp_drv,
	const lv_area_t *area, lv_color_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle =
		(esp_lcd_panel_handle_t)disp_drv->user_data;

	int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;

    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, (void *)px_map);
}

void lcd_lvgl_update_cb(struct _lv_disp_drv_t * disp_drv)
{
	esp_lcd_panel_handle_t panel_handle =
		(esp_lcd_panel_handle_t)disp_drv->user_data;

	switch (disp_drv->rotated) {
		case LV_DISP_ROT_90:
			esp_lcd_panel_io_tx_param(io_handle, 0x36, (uint8_t[]) {
				0x08,
			}, 1);

			esp_lcd_panel_set_gap(panel_handle, 26, 1);
			break;
		case LV_DISP_ROT_180:
			esp_lcd_panel_io_tx_param(io_handle, 0x36, (uint8_t[]) {
				0x78,
			}, 1);

			esp_lcd_panel_set_gap(panel_handle, 1, 26);
			break;
		case LV_DISP_ROT_270:
			esp_lcd_panel_io_tx_param(io_handle, 0x36, (uint8_t[]) {
				0xC8,
			}, 1);

			esp_lcd_panel_set_gap(panel_handle, 26, 1);
			break;
		case LV_DISP_ROT_NONE:
			esp_lcd_panel_io_tx_param(io_handle, 0x36, (uint8_t[]) {
				0xA8,
			}, 1);

			esp_lcd_panel_set_gap(panel_handle, 1, 26);
		default:
			break;
	}
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = 50;

    while (1) {
        task_delay_ms = lv_timer_handler();

        vTaskDelay(task_delay_ms / portTICK_PERIOD_MS);
    }
}

void lvgl_spi_lcd_init(void)
{
	static lv_disp_drv_t disp_drv;

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_SCLK,
        .mosi_io_num = LCD_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };

    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_DC,
        .cs_gpio_num = LCD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = lcd_notify_lvgl_flush_ready,
        .user_ctx = (void *)&disp_drv,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RST,
        .color_space = ESP_LCD_COLOR_SPACE_RGB,
        .bits_per_pixel = 16,
    };
	esp_lcd_new_panel_st7735s(io_handle, &panel_config, &panel_handle);

    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
	esp_lcd_panel_invert_color(panel_handle, true);
	esp_lcd_panel_disp_off(panel_handle, false);
	esp_lcd_panel_set_gap(panel_handle, 1, 26);

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(LCD_H_RES * 40 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(LCD_H_RES * 40 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // initialize LVGL draw buffers

    ESP_LOGI(TAG, "Register display driver to LVGL");

    static lv_disp_draw_buf_t draw_buf_dsc_1;
   lv_disp_draw_buf_init(&draw_buf_dsc_1, buf1, buf2, LCD_H_RES * 20);

	lv_disp_drv_init(&disp_drv);

    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;

	disp_drv.flush_cb = lcd_lvgl_flush_cb;
	disp_drv.drv_update_cb = lcd_lvgl_update_cb;
	disp_drv.draw_buf = &draw_buf_dsc_1;
	disp_drv.user_data = panel_handle;

	lv_disp_drv_register(&disp_drv);

	lv_port_indev_init();

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);
}
