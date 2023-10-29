/**
 * @file lvgl_helpers.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "sdkconfig.h"
#include "lvgl_helpers.h"
#include "esp_log.h"

#include "lvgl_tft/disp_spi.h"
#include "lvgl_touch/tp_spi.h"

#include "lvgl_spi_conf.h"

#include "lvgl_i2c/i2c_manager.h"

#include "esp_lcd_backlight.h"

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

/*********************
 *      DEFINES
 *********************/

 #define TAG "lvgl_helpers"

/**********************
 *      TYPEDEFS
 **********************/

struct _lvgl_driver_user_data {
    disp_backlight_h *bckl_handle;
};
typedef struct _lvgl_driver_user_data driver_data_int;

/**********************
 *  STATIC PROTOTYPES
 **********************/
/* Initialize display backligt */
static disp_backlight_h *lvgl_driver_backlight_init(void);


/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/* Interface and driver initialization */
void lvgl_driver_init(lv_disp_drv_t *drv)
{
    driver_data_int *user_data = calloc(1, sizeof(driver_data_int));
    if (user_data == NULL){
        ESP_LOGW(TAG, "Not enough memory");
        return;
    }
    drv->user_data = user_data;

#if defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT || defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT_INVERTED
    drv->rotated = 1;
#endif

    /* When using a monochrome display we need to register the callbacks:
     * - rounder_cb
     * - set_px_cb */
#ifdef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    drv->rounder_cb = disp_driver_rounder;
    drv->set_px_cb = disp_driver_set_px;
#endif

    drv->hor_res = CONFIG_LV_HOR_RES_MAX;
    drv->ver_res = CONFIG_LV_VER_RES_MAX;

    /* Since LVGL v8 CONFIG_CONFIG_LV_HOR_RES_MAX and CONFIG_LV_VER_RES_MAX are not defined, so
     * print it only if they are defined. */
#if (LVGL_VERSION_MAJOR < 8)
    ESP_LOGI(TAG, "Display hor size: %d, ver size: %d", CONFIG_CONFIG_LV_HOR_RES_MAX, CONFIG_LV_VER_RES_MAX);
#endif

#if defined (CONFIG_LV_TFT_DISPLAY_CONTROLLER_FT81X)
    ESP_LOGI(TAG, "Initializing SPI master for FT81X");

    lvgl_spi_driver_init(TFT_SPI_HOST,
        DISP_SPI_MISO, DISP_SPI_MOSI, DISP_SPI_CLK,
        SPI_BUS_MAX_TRANSFER_SZ, 1,
        DISP_SPI_IO2, DISP_SPI_IO3);

    disp_spi_add_device(TFT_SPI_HOST);
    disp_driver_init();
    user_data->bckl_handle = lvgl_driver_backlight_init();

#if defined (CONFIG_LV_TOUCH_CONTROLLER_FT81X)
    touch_driver_init();
#endif

    return;
#endif

#if defined (SHARED_SPI_BUS)
    ESP_LOGI(TAG, "Initializing shared SPI master");

    lvgl_spi_driver_init(TFT_SPI_HOST,
        TP_SPI_MISO, DISP_SPI_MOSI, DISP_SPI_CLK,
        SPI_BUS_MAX_TRANSFER_SZ, 1,
        -1, -1);

    disp_spi_add_device(TFT_SPI_HOST);
    tp_spi_add_device(TOUCH_SPI_HOST);

    disp_driver_init();
    user_data->bckl_handle = lvgl_driver_backlight_init();
    touch_driver_init();

    return;
#endif

/* Display controller initialization */
#if defined CONFIG_LV_TFT_DISPLAY_PROTOCOL_SPI
    ESP_LOGI(TAG, "Initializing SPI master for display");

    lvgl_spi_driver_init(TFT_SPI_HOST,
        DISP_SPI_MISO, DISP_SPI_MOSI, DISP_SPI_CLK,
        SPI_BUS_MAX_TRANSFER_SZ, 1,
        DISP_SPI_IO2, DISP_SPI_IO3);

    disp_spi_add_device(TFT_SPI_HOST);

    disp_driver_init();
    user_data->bckl_handle = lvgl_driver_backlight_init();
#elif defined (CONFIG_LV_I2C_DISPLAY)
    disp_driver_init();
    user_data->bckl_handle = lvgl_driver_backlight_init();
#else
#error "No protocol defined for display controller"
#endif

/* Touch controller initialization */
#if CONFIG_LV_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
    #if defined (CONFIG_LV_TOUCH_DRIVER_PROTOCOL_SPI)
        ESP_LOGI(TAG, "Initializing SPI master for touch");

        lvgl_spi_driver_init(TOUCH_SPI_HOST,
            TP_SPI_MISO, TP_SPI_MOSI, TP_SPI_CLK,
            0 /* Defaults to 4094 */, 2,
            -1, -1);

        tp_spi_add_device(TOUCH_SPI_HOST);

        touch_driver_init();
    #elif defined (CONFIG_LV_I2C_TOUCH)
        touch_driver_init();
    #elif defined (CONFIG_LV_TOUCH_DRIVER_ADC)
        touch_driver_init();
    #elif defined (CONFIG_LV_TOUCH_DRIVER_DISPLAY)
        touch_driver_init();
    #else
    #error "No protocol defined for touch controller"
    #endif
#else
#endif
}

disp_backlight_h *lvgl_driver_backlight_init(void)
{
    // We still use menuconfig for these settings
    // It will be set up during runtime in the future
#if (defined(CONFIG_LV_DISP_BACKLIGHT_SWITCH) || defined(CONFIG_LV_DISP_BACKLIGHT_PWM))
    const disp_backlight_config_t bckl_config = {
        .gpio_num = CONFIG_LV_DISP_PIN_BCKL,
#if defined CONFIG_LV_DISP_BACKLIGHT_PWM
        .pwm_control = true,
#else
        .pwm_control = false,
#endif
#if defined CONFIG_LV_BACKLIGHT_ACTIVE_LVL
        .output_invert = false, // Backlight on high
#else
        .output_invert = true, // Backlight on low
#endif
        .timer_idx = 0,
        .channel_idx = 0 // @todo this prevents us from having two PWM controlled displays
    };
    disp_backlight_h bckl_handle = disp_backlight_new(&bckl_config);
    disp_backlight_set(bckl_handle, 100);
    return bckl_handle;
#else
    return NULL;
#endif
}

void lvgl_driver_backlight_set(lv_disp_drv_t *drv, int brightness_percent) {
    disp_backlight_set(((driver_data_int *)drv->user_data)->bckl_handle, brightness_percent);
}


/* Initialize spi bus master
 *
 * NOTE: dma_chan type and value changed to int instead of spi_dma_chan_t
 * for backwards compatibility with ESP-IDF versions prior v4.3.
 *
 * We could use the ESP_IDF_VERSION_VAL macro available in the "esp_idf_version.h"
 * header available since ESP-IDF v4.
 */
bool lvgl_spi_driver_init(int host,
    int miso_pin, int mosi_pin, int sclk_pin,
    int max_transfer_sz,
    int dma_channel,
    int quadwp_pin, int quadhd_pin)
{
    assert((0 <= host) && (SPI_HOST_MAX > host));
    const char *spi_names[] = {
        "SPI1_HOST", "SPI2_HOST", "SPI3_HOST"
    };

    ESP_LOGI(TAG, "Configuring SPI host %s", spi_names[host]);
    ESP_LOGI(TAG, "MISO pin: %d, MOSI pin: %d, SCLK pin: %d, IO2/WP pin: %d, IO3/HD pin: %d",
        miso_pin, mosi_pin, sclk_pin, quadwp_pin, quadhd_pin);

    ESP_LOGI(TAG, "Max transfer size: %d (bytes)", max_transfer_sz);

    spi_bus_config_t buscfg = {
        .miso_io_num = miso_pin,
        .mosi_io_num = mosi_pin,
        .sclk_io_num = sclk_pin,
        .quadwp_io_num = quadwp_pin,
        .quadhd_io_num = quadhd_pin,
        .max_transfer_sz = max_transfer_sz
    };

    ESP_LOGI(TAG, "Initializing SPI bus...");
    #if defined (CONFIG_IDF_TARGET_ESP32C3)
    dma_channel = SPI_DMA_CH_AUTO;
    #endif
    esp_err_t ret = spi_bus_initialize(host, &buscfg, (spi_dma_chan_t)SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);

    return ESP_OK != ret;
}
