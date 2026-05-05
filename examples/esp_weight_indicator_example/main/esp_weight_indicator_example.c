#include <stdio.h>
#include "esp_weight_indicator.h"
#include "hx711.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

// This example demonstrates how to use the weight indicator component with a weight sensor driver (HX711) and others components.
// All peripheral components (weight sensor, display, communication, etc.) that are connected to the 
// weight indicator component can be easily replaced by other components, as long as they implement the same interface 
// (callbacks) defined in the weight indicator component.

static const char *TAG = "main";

// display task and related definitions
/*==============================================================
                        BIBLIOTECAS
==============================================================*/
// Sistema
#include <unistd.h>
#include <sys/lock.h>
#include <sys/param.h>

// ESP-IDF
#include "esp_timer.h"
#include "esp_err.h"

// Driver LCD e I2C
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "driver/i2c_master.h"
#include "esp_lcd_panel_vendor.h"

// LVGL (interface gráfica)
#include "lvgl.h"


/*==============================================================
                        DEFINES - HARDWARE
==============================================================*/

#define I2C_BUS_PORT 0

#define EXAMPLE_LCD_PIXEL_CLOCK_HZ (400 * 1000)
#define EXAMPLE_PIN_NUM_SDA 19
#define EXAMPLE_PIN_NUM_SCL 18
#define EXAMPLE_PIN_NUM_RST -1
#define EXAMPLE_I2C_HW_ADDR 0x3C

/*==============================================================
                        DEFINES - DISPLAY
==============================================================*/

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES 128
#define EXAMPLE_LCD_V_RES 64
// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS 8
#define EXAMPLE_LCD_PARAM_BITS 8

/*==============================================================
                        DEFINES - LVGL
==============================================================*/

#define EXAMPLE_LVGL_TICK_PERIOD_MS 5
#define EXAMPLE_LVGL_TASK_STACK_SIZE (4 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY 2
#define EXAMPLE_LVGL_PALETTE_SIZE 8
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1000 / CONFIG_FREERTOS_HZ

/*==============================================================
                        VARIÁVEIS GLOBAIS
==============================================================*/

// To use LV_COLOR_FORMAT_I1, we need an extra buffer to hold the converted data
static uint8_t oled_buffer[EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES / 8];
// LVGL library is not thread-safe, this example will call LVGL APIs from different tasks, so use a mutex to protect it
static _lock_t lvgl_api_lock;

static const char *OLED_TAG = "OLED";

/*==============================================================
                        FUNÇÕES LVGL
==============================================================*/

void example_lvgl_demo_ui(lv_disp_t *disp)
{
    lv_obj_t *scr = lv_display_get_screen_active(disp);
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
    lv_label_set_text(label, "00000");
    /* Size of the screen (if you use rotation 90 or 270, please use lv_display_get_vertical_resolution) */
    lv_obj_set_width(label, lv_display_get_horizontal_resolution(disp));
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t io_panel, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp = (lv_display_t *)user_ctx;
    lv_display_flush_ready(disp);
    return false;
}

/*==============================================================
                CALLBACK DE RENDERIZAÇÃO (LVGL → OLED)
==============================================================*/

static void example_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_handle_t panel_handle = lv_display_get_user_data(disp);

    // This is necessary because LVGL reserves 2 x 4 bytes in the buffer, as these are assumed to be used as a palette. Skip the palette here
    // More information about the monochrome, please refer to https://docs.lvgl.io/9.2/porting/display.html#monochrome-displays
    px_map += EXAMPLE_LVGL_PALETTE_SIZE;

    uint16_t hor_res = lv_display_get_physical_horizontal_resolution(disp);
    int x1 = area->x1;
    int x2 = area->x2;
    int y1 = area->y1;
    int y2 = area->y2;

    for (int y = y1; y <= y2; y++)
    {
        for (int x = x1; x <= x2; x++)
        {
            /* The order of bits is MSB first
                        MSB           LSB
               bits      7 6 5 4 3 2 1 0
               pixels    0 1 2 3 4 5 6 7
                        Left         Right
            */
            bool chroma_color = (px_map[(hor_res >> 3) * y + (x >> 3)] & 1 << (7 - x % 8));

            /* Write to the buffer as required for the display.
             * It writes only 1-bit for monochrome displays mapped vertically.*/
            uint8_t *buf = oled_buffer + hor_res * (y >> 3) + (x);
            if (chroma_color)
            {
                (*buf) &= ~(1 << (y % 8));
            }
            else
            {
                (*buf) |= (1 << (y % 8));
            }
        }
    }
    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, x1, y1, x2 + 1, y2 + 1, oled_buffer);
}


/*==============================================================
                        TIMER LVGL
==============================================================*/

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

/*==============================================================
                        TASK LVGL
==============================================================*/

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(OLED_TAG, "Starting LVGL task");
    uint32_t time_till_next_ms = 0;
    while (1)
    {
        _lock_acquire(&lvgl_api_lock);
        time_till_next_ms = lv_timer_handler();
        _lock_release(&lvgl_api_lock);
        // in case of triggering a task watch dog time out
        time_till_next_ms = MAX(time_till_next_ms, EXAMPLE_LVGL_TASK_MIN_DELAY_MS);
        // in case of lvgl display not ready yet
        time_till_next_ms = MIN(time_till_next_ms, EXAMPLE_LVGL_TASK_MAX_DELAY_MS);
        usleep(1000 * time_till_next_ms);
    }
}

/*==============================================================
                        TASK OLED (MAIN)
==============================================================*/

static void oled_task(void *arg)
{

    QueueHandle_t queue = (QueueHandle_t)arg;
    ESP_LOGI(OLED_TAG, "Initialize I2C bus");
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .i2c_port = I2C_BUS_PORT,
        .sda_io_num = EXAMPLE_PIN_NUM_SDA,
        .scl_io_num = EXAMPLE_PIN_NUM_SCL,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus));

    ESP_LOGI(OLED_TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t io_config = {
        .dev_addr = EXAMPLE_I2C_HW_ADDR,
        .scl_speed_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .control_phase_bytes = 1,               // According to SSD1306 datasheet
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,   // According to SSD1306 datasheet
        .lcd_param_bits = EXAMPLE_LCD_CMD_BITS, // According to SSD1306 datasheet
        .dc_bit_offset = 6,                     // According to SSD1306 datasheet
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &io_handle));

    ESP_LOGI(OLED_TAG, "Install SSD1306 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .bits_per_pixel = 1,
        .reset_gpio_num = EXAMPLE_PIN_NUM_RST,
    };
    esp_lcd_panel_ssd1306_config_t ssd1306_config = {
        .height = EXAMPLE_LCD_V_RES,
    };
    panel_config.vendor_config = &ssd1306_config;
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(OLED_TAG, "Initialize LVGL");
    lv_init();
    // create a lvgl display
    lv_display_t *display = lv_display_create(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);
    // associate the i2c panel handle to the display
    lv_display_set_user_data(display, panel_handle);
    // create draw buffer
    void *buf = NULL;
    ESP_LOGI(OLED_TAG, "Allocate separate LVGL draw buffers");
    // LVGL reserves 2 x 4 bytes in the buffer, as these are assumed to be used as a palette.
    size_t draw_buffer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES / 8 + EXAMPLE_LVGL_PALETTE_SIZE;
    buf = heap_caps_calloc(1, draw_buffer_sz, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    assert(buf);

    // LVGL9 suooprt new monochromatic format.
    lv_display_set_color_format(display, LV_COLOR_FORMAT_I1);
    // initialize LVGL draw buffers
    lv_display_set_buffers(display, buf, NULL, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_FULL);
    // set the callback which can copy the rendered image to an area of the display
    lv_display_set_flush_cb(display, example_lvgl_flush_cb);

    ESP_LOGI(OLED_TAG, "Register io panel event callback for LVGL flush ready notification");
    const esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = example_notify_lvgl_flush_ready,
    };
    /* Register done callback */
    esp_lcd_panel_io_register_event_callbacks(io_handle, &cbs, display);

    ESP_LOGI(OLED_TAG, "Use esp_timer as LVGL tick timer");
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    ESP_LOGI(OLED_TAG, "Create LVGL task");
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);

    ESP_LOGI(OLED_TAG, "Display LVGL Scroll Text");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    
    lv_obj_t *scr = lv_display_get_screen_active(display);
    lv_obj_t *label_weight = lv_label_create(scr);
    lv_label_set_text(label_weight, "00000");
    lv_obj_set_style_text_font(label_weight, &lv_font_montserrat_24, 0);
    lv_obj_align(label_weight, LV_ALIGN_TOP_MID, 0, 0); //LV_ALIGN_CENTER

  
    lv_obj_t *label_S = lv_label_create(scr);
    lv_label_set_text(label_S, " ");
    lv_obj_set_style_text_font(label_S, &lv_font_montserrat_14, 0);
    lv_obj_align(label_S, LV_ALIGN_BOTTOM_RIGHT, 0, 0); //LV_ALIGN_CENTER

    esp_weight_event_t strain_disp;
    char strain_str[20];
    while (1)
    {
        xQueueReceive(queue, &strain_disp, portMAX_DELAY);
        _lock_acquire(&lvgl_api_lock);
        sprintf(strain_str, "%.3f kg", strain_disp.filtered);
        lv_label_set_text(label_weight, strain_str);
    
        _lock_release(&lvgl_api_lock);

        _lock_acquire(&lvgl_api_lock);
        
        if (strain_disp.stable) {
            lv_label_set_text(label_S, "S");
        } else {
            lv_label_set_text(label_S, " ");
        }
        _lock_release(&lvgl_api_lock);

    }
        
}

// call backs de conexão entre driver e middleware

esp_err_t update_weight(void *driver, int32_t *data_raw){

            hx711_t *dev = (hx711_t *)driver; // Cast do ponteiro genérico para o tipo específico do driver
           
            esp_err_t r = hx711_wait(dev, 500);
            if (r != ESP_OK)
            {
                ESP_LOGE(TAG, "Device not found: %d (%s)\n", r, esp_err_to_name(r));
                return r;
            }

            r = hx711_read_average(dev, 1, data_raw);
            if (r != ESP_OK)
            {
                ESP_LOGE(TAG, "Could not read data: %d (%s)\n", r, esp_err_to_name(r));
                return r;
            }
            
            r = ESP_OK;
            return r;
}
    
esp_err_t notify_weight_dispay(void *ctx , const esp_weight_event_t *e){
    
    QueueHandle_t queue = (QueueHandle_t)ctx;

    esp_weight_event_t event = {
        .weight = e->weight,
        .raw = e->raw,
        .stable = e->stable,
        .filtered = e->filtered
    };

    xQueueSend(queue, &event, portMAX_DELAY);


    
   
    return ESP_OK;
}

void app_main(void)
{
        
    QueueHandle_t queue_display = xQueueCreate(3, sizeof(esp_weight_event_t));

    // INIT DRIVER WEIGHT SENSOR
    hx711_t dev =
        {
            .dout = 17,
            .pd_sck = 16,
            .gain = HX711_GAIN_A_64};

    ESP_ERROR_CHECK(hx711_init(&dev));
    
    esp_weight_indicator_handle_t weight_indicator_handle;
    esp_weight_indicator_config_t weight_indicator_config = {
        .weight = 0,
        .driver = &dev,
        .filter_config = {
            .type = WEIGHT_FILTER_MOVING_AVG,
            .alpha = 0.1,
            .window_size = 5
        },
        .stability_config = {
            .threshold = 0.001
        }
    };

    // Initialize the weight indicator component
    esp_err_t ret = esp_weight_indicator_init(&weight_indicator_config, &weight_indicator_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize weight indicator: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Weight indicator initialized successfully");

    ret = esp_weight_indicator_update_cb_register(weight_indicator_handle, update_weight);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register read callback: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Read callback registered successfully");

    ret = esp_weight_indicator_notify_cb_register(weight_indicator_handle, notify_weight_dispay, queue_display);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register notify callback: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Notify callback registered successfully");

    
    xTaskCreate(oled_task, "oled_task", 4096, queue_display , 5, NULL);

    while (1) {
  
        ret = esp_weight_indicator_update(weight_indicator_handle);
        if (ret == ESP_OK) { 
            esp_weight_indicator_notify(weight_indicator_handle);
        } else {
            ESP_LOGE(TAG, "Failed to read weight: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(1000)); // Delay de 1 segundo
        }

       
       
        
    }
}
