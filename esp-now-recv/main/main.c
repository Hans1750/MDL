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
#include "amoled_driver.h"
#include "touch_driver.h"
#include "i2c_driver.h"
#include "power_driver.h"
#include "demos/lv_demos.h"
#include "tft_driver.h"
#include "product_pins.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

static const char *TAG = "main";

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY 2

#define IMAGE_WIDTH 480
#define IMAGE_HEIGHT 480
#define SCREEN_RADIUS 240 // Nastavi radij kroga (prilagodi glede na zaslon)

static bool show_checkmark = false;
char text[128];

void init_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    ESP_ERROR_CHECK(esp_wifi_start());
}

void init_espnow(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_LOGI("ESP-NOW", "ESP-NOW inicializiran");
}

void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    char mac_str[18];
    const uint8_t *mac_addr = recv_info->src_addr;

    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5]);

    ESP_LOGI("ESP-NOW", "Sporočilo prejeto od %s: %.*s", mac_str, len, data);
    memcpy(text, data, len);
    text[len] = '\0';
}
void register_receive_cb()
{
    esp_now_register_recv_cb(on_data_recv);
}

void show_image_with_circle()
{ // 2️⃣ DODAJ ZELEN KROG NAD SLIKO
    lv_obj_t *circle = lv_obj_create(lv_scr_act());
    lv_obj_set_size(circle, 2 * SCREEN_RADIUS, 2 * SCREEN_RADIUS); // Krog pokriva sliko
    lv_obj_align(circle, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_radius(circle, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_color(circle, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_set_style_border_width(circle, 20, 0);
    lv_obj_set_style_bg_opa(circle, LV_OPA_TRANSP, 0); // Naredi notranjost prozorno

    // 3️⃣ ZELENI KROG MORA BITI NAD SLIKO
    lv_obj_set_parent(circle, lv_scr_act());
}

static void switch_display(lv_timer_t *timer)
{
    lv_obj_clean(lv_scr_act()); // Počisti zaslon

    if (!show_checkmark)
    {
        show_image_with_circle(); // Prikaži sliko z okvirjem
    }
    else
    {
        lv_obj_t *bg = lv_obj_create(lv_scr_act());
        lv_obj_set_size(bg, LV_HOR_RES, LV_VER_RES);
        lv_obj_align(bg, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_color(bg, lv_color_black(), 0);
        lv_obj_set_style_border_width(bg, 0, 0);

        // 2️⃣ Dodaj kljukico s simbolom LVGL
        lv_obj_t *checkmark = lv_label_create(lv_scr_act());
        lv_label_set_text(checkmark, LV_SYMBOL_OK); // Namesto "✔" uporabi LVGL simbol
        lv_obj_set_style_text_color(checkmark, lv_palette_main(LV_PALETTE_GREEN), 0);
        lv_obj_set_style_text_font(checkmark, &lv_font_montserrat_40, 0); // VEČJA KLJUKICA
        lv_obj_align(checkmark, LV_ALIGN_TOP_MID, 0, 40);                 // Poravnava kljukice

        // 3️⃣ Dodaj besedilo
        lv_obj_t *label = lv_label_create(lv_scr_act());
        lv_label_set_text(label, text);
        lv_obj_set_style_text_color(label, lv_color_white(), 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_40, 0); // VEČJE BESEDILO
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 50);                  // Besedilo v sredino zaslona
    }

    show_checkmark = !show_checkmark; // Obrni stanje
}

// Ustvari timer za preklop
void start_switching()
{
    lv_timer_create(switch_display, 5000, NULL);
}

static SemaphoreHandle_t lvgl_mux = NULL;

static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)

lv_disp_drv_t disp_drv; // contains callback functions

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
#if DISPLAY_FULLRESH
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    display_push_colors(area->x1, area->y1, w, h, (uint16_t *)color_map);
    lv_disp_flush_ready(drv);
#else
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    display_push_colors(offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, (uint16_t *)color_map);
#endif
}

#if BOARD_HAS_TOUCH
static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    int16_t touchpad_x[1] = {0};
    int16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    /* Get coordinates */
    touchpad_cnt = touch_get_data(touchpad_x, touchpad_y, 1);

    if (touchpad_cnt > 0)
    {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}
#endif

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

bool example_lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If timeout_ms is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void example_lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1)
    {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (example_lvgl_lock(-1))
        {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS)
        {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        }
        else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS)
        {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

void example_lvgl_demo_ui(lv_disp_t *disp);

void app_main(void)
{

    nvs_flash_init();
    init_wifi();
    init_espnow();
    register_receive_cb();

    ESP_LOGI(TAG, "------ Initialize I2C.");
    i2c_driver_init();

    ESP_LOGI(TAG, "------ Initialize PMU.");
    if (!power_driver_init())
    {
        ESP_LOGE(TAG, "ERROR :No find PMU ....");
    }

    ESP_LOGI(TAG, "------ Initialize TOUCH.");
    touch_init();

    ESP_LOGI(TAG, "------ Initialize DISPLAY.");
    display_init();

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
#if CONFIG_SPIRAM

    lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(DISPLAY_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf1);

    lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(DISPLAY_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf2);

    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, DISPLAY_BUFFER_SIZE);

#else
    lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(AMOLED_HEIGHT * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(AMOLED_HEIGHT * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, AMOLED_HEIGHT * 20);
#endif

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = AMOLED_HEIGHT;
    disp_drv.ver_res = AMOLED_WIDTH;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.full_refresh = DISPLAY_FULLRESH;
    lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "lvgl_tick",
        .skip_unhandled_events = false};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

#if BOARD_HAS_TOUCH
    ESP_LOGI(TAG, "Register touch driver to LVGL");
    static lv_indev_drv_t indev_drv; // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = example_lvgl_touch_cb;
    lv_indev_drv_register(&indev_drv);
#endif

    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);

    ESP_LOGI(TAG, "Display LVGL");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (example_lvgl_lock(-1))
    {
        show_image_with_circle();
        start_switching();
        // Release the mutex
        example_lvgl_unlock();
    }
    // tskIDLE_PRIORITY,
    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
}