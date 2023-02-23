/**
 * @file st7789v.c
 *
 * Mostly taken from lbthomsen/esp-idf-littlevgl github.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "esp_log.h"

#include "st7789v.h"

#include "disp_spi.h"
#include "driver/gpio.h"

/*********************
 *      DEFINES
 *********************/
#define TAG "st7789v"
/**********************
 *      TYPEDEFS
 **********************/

/*The LCD needs a bunch of command/argument values to be initialized. They are stored in this struct. */
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void st7789v_set_orientation(uint8_t orientation);

static void st7789v_send_color(void *data, size_t length);

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void st7789v_init(void)
{
    lcd_init_cmd_t st7789v_init_cmds[] = {
        /* Added cmds for 'v' version of st7789.
           Set according to constructor documentation */
        {ST7789V_SLPOUT, {0}, 0x80},

        {ST7789V_PORCTRL, {0x0c,0x0c,0x00,0x33,0x33}, 5},
        {ST7789V_GCTRL, {0x35}, 1},
        {ST7789V_VCOMS, {0x1A}, 1},
        {ST7789V_LCMCTRL, {0x2C}, 1},
        {ST7789V_VDVVRHEN, {0x01}, 1},
        {ST7789V_VRHS, {0x0B}, 1},
        {ST7789V_VDVSET, {0x20}, 1},
        {ST7789V_FRCTR2, {0x0F}, 1},
        {ST7789V_REGSEL2, {0x0F}, 1},
        {ST7789V_REGSEL1, {0x08}, 1},
        {ST7789V_WRCACE, {0x90}, 1},
        {ST7789V_PWCTRL1, {0xA7, 0xA1}, 2},
        {ST7789V_PVGAMCTRL, {0xd0,0x00,0x02,0x07,0x0b,0x1a,0x31,0x54,0x40,0x29,0x12,0x12,0x12,0x17}, 14},
        {ST7789V_NVGAMCTRL, {0xd0,0x00,0x02,0x07,0x05,0x25,0x2d,0x44,0x45,0x1c,0x18,0x16,0x1c,0x1d}, 14},

#if ST7789V_INVERT_COLORS == 1
 		{ST7789V_INVOFF, {0}, 0}, // set non-inverted mode
#else
		{ST7789V_INVON, {0}, 0}, // set inverted mode
#endif

        {ST7789V_MADCTL, {0x00}, 1}, // Try to set to 0x28 if your display is flippeds flipped 
        {ST7789V_COLMOD, {0x05}, 1},
        {ST7789V_DISPON, {0}, 0x00},
        {0, {0}, 0xff},
    };

    //Initialize non-SPI GPIOs
    gpio_pad_select_gpio(ST7789V_DC);
    gpio_set_direction(ST7789V_DC, GPIO_MODE_OUTPUT);
    
#if !defined(ST7789V_SOFT_RST)
    gpio_pad_select_gpio(ST7789V_RST);
    gpio_set_direction(ST7789V_RST, GPIO_MODE_OUTPUT);
#endif

    //Reset the display
#if !defined(ST7789V_SOFT_RST)
    gpio_set_level(ST7789V_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(ST7789V_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);
#else
    st7789v_send_cmd(ST7789V_SWRESET);
#endif

    ESP_LOGI("st7789v", "ST7789V start initialization.");

    //Send all the commands
    uint16_t cmd = 0;
    while (st7789v_init_cmds[cmd].databytes!=0xff) {
        st7789v_send_cmd(st7789v_init_cmds[cmd].cmd);
        st7789v_send_data(st7789v_init_cmds[cmd].data, st7789v_init_cmds[cmd].databytes&0x1F);
        if (st7789v_init_cmds[cmd].databytes & 0x80) {
                vTaskDelay(ST7789V_TIME_TO_WAIT_AFTER_LONG_OPERATIONS / portTICK_RATE_MS);
        }
        cmd++;
    }

    st7789v_set_orientation(CONFIG_LV_DISPLAY_ORIENTATION);
}

/* The ST7789V display controller can drive up to 320*240 displays, when using a 240*240 or 240*135
 * displays there's a gap of 80px or 40/52/53px respectively. 52px or 53x offset depends on display orientation.
 * We need to edit the coordinates to take into account those gaps, this is not necessary in all orientations. */
void st7789v_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
    uint8_t data[4] = {0};

    uint16_t offsetx1 = area->x1;
    uint16_t offsetx2 = area->x2;
    uint16_t offsety1 = area->y1;
    uint16_t offsety2 = area->y2;

#if (CONFIG_LV_TFT_DISPLAY_OFFSETS)
    offsetx1 += CONFIG_LV_TFT_DISPLAY_X_OFFSET;
    offsetx2 += CONFIG_LV_TFT_DISPLAY_X_OFFSET;
    offsety1 += CONFIG_LV_TFT_DISPLAY_Y_OFFSET;
    offsety2 += CONFIG_LV_TFT_DISPLAY_Y_OFFSET;

#elif (LV_HOR_RES_MAX == 240) && (LV_VER_RES_MAX == 240)
    #if (CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT)
        offsetx1 += 80;
        offsetx2 += 80;
    #elif (CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE_INVERTED)
        offsety1 += 80;
        offsety2 += 80;
    #endif
#elif (LV_HOR_RES_MAX == 240) && (LV_VER_RES_MAX == 135)
    #if (CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT) || \
        (CONFIG_LV_DISPLAY_ORIENTATION_PORTRAIT_INVERTED)
        offsetx1 += 40;
        offsetx2 += 40;
        offsety1 += 53;
        offsety2 += 53;
    #endif
#elif (LV_HOR_RES_MAX == 135) && (LV_VER_RES_MAX == 240)
    #if (CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE) || \
        (CONFIG_LV_DISPLAY_ORIENTATION_LANDSCAPE_INVERTED)
        offsetx1 += 52;
        offsetx2 += 52;
        offsety1 += 40;
        offsety2 += 40;
    #endif
#endif

    /*Column addresses*/
    st7789v_send_cmd(ST7789V_CASET);
    data[0] = (offsetx1 >> 8) & 0xFF;
    data[1] = offsetx1 & 0xFF;
    data[2] = (offsetx2 >> 8) & 0xFF;
    data[3] = offsetx2 & 0xFF;
    st7789v_send_data(data, 4);

    /*Page addresses*/
    st7789v_send_cmd(ST7789V_RASET);
    data[0] = (offsety1 >> 8) & 0xFF;
    data[1] = offsety1 & 0xFF;
    data[2] = (offsety2 >> 8) & 0xFF;
    data[3] = offsety2 & 0xFF;
    st7789v_send_data(data, 4);

    /*Memory write*/
    st7789v_send_cmd(ST7789V_RAMWR);

    size_t size = (size_t)lv_area_get_width(area) * (size_t)lv_area_get_height(area);

    st7789v_send_color((void*)color_map, size * 2);

}

/**********************
 *   STATIC FUNCTIONS
 **********************/
void st7789v_send_cmd(uint8_t cmd)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ST7789V_DC, 0);
    disp_spi_send_data(&cmd, 1);
}

void st7789v_send_data(void * data, uint16_t length)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ST7789V_DC, 1);
    disp_spi_send_data(data, length);
}

static void st7789v_send_color(void * data, size_t length)
{
    disp_wait_for_pending_transactions();
    gpio_set_level(ST7789V_DC, 1);
    disp_spi_send_colors(data, length);
}

static void st7789v_set_orientation(uint8_t orientation)
{
    // ESP_ASSERT(orientation < 4);

    const char *orientation_str[] = {
        "PORTRAIT", "PORTRAIT_INVERTED", "LANDSCAPE", "LANDSCAPE_INVERTED"
    };

    ESP_LOGI(TAG, "Display orientation: %s", orientation_str[orientation]);

    uint8_t data[] =
    {
#if CONFIG_LV_PREDEFINED_DISPLAY_TTGO
	0x60, 0xA0, 0x00, 0xC0
#else
	0xC0, 0x00, 0x60, 0xA0
#endif
    };

    ESP_LOGI(TAG, "0x36 command value: 0x%02X", data[orientation]);

    st7789v_send_cmd(ST7789V_MADCTL);
    st7789v_send_data((void *) &data[orientation], 1);
}
