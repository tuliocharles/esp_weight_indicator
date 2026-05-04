#include <stdio.h>
#include "esp_weight_indicator.h"
#include "esp_check.h"
#include "esp_nvs.h"

#define MAX_LISTENERS 5 // Maximum number of listeners that can be registered to receive weight updates
#define MAX_WINDOW 10   // Maximum window size for moving average filter, can be adjusted based on requirements
#define STAB_WINDOW 10

static const char *TAG = "weight";

typedef struct esp_weight_indicator_t esp_weight_indicator_t;

typedef struct
{
    float buf[STAB_WINDOW];
    int32_t index;
    int32_t count;

} stability_state_t;

typedef struct
{

    float ema_value; // current value of the EMA filter

    float window[MAX_WINDOW]; // buffer to hold the last N values for moving average
    int index;                // current index in the window buffer
    int count;                // number of values currently in the window (up to MAX_WINDOW)

} weight_filter_state_t;

typedef struct
{
    esp_weight_indicator_notify_cb_t cb;
    void *ctx;
} notify_entry_t;

typedef struct
{
    uint8_t version;

    // calibration parameters
    int32_t offset;
    float scale;

    // display parameters
    uint8_t decimals;

    // stability parameters
    float stable_threshold;  // variação máxima permitida
    uint16_t stable_time_ms; // tempo mínimo para considerar estável

    // filtering parameters
    weight_filter_config_t filter_config;

    // tare parameters
    float tare;
    bool tare_enabled;

} weight_persistent_t;

struct esp_weight_indicator_t
{
    notify_entry_t listeners[MAX_LISTENERS];
    size_t count;

    weight_persistent_t persistent; // persistent data, such as calibration parameters, display settings, stability settings, filter settings, tare settings, etc. This data will be stored in NVS and loaded on initialization.

    esp_weight_event_t last_event; // last weight event, containing the last raw and filtered weight values, stability status, etc.

    weight_filter_state_t filter_state; // state of the filter, such as the current EMA value, the buffer for moving average, etc.

    stability_state_t stability_state; // state for stability detection, such as the buffer for recent weight values, index, count, etc.

    void *driver;                                     // Pointer to the weight sensor driver, can be used to store a reference to the driver instance
    esp_weight_indicator_update_cb_t update_callback; // Callback function for updates
    // esp_weight_indicator_notify_cb_t notify_callback; // Callback function for writes

    // persistent storage fields
    esp_nvs_handle_t esp_nvs_handle;
};

static float moving_avg(esp_weight_indicator_handle_t handle)
{
    handle->filter_state.window[handle->filter_state.index] = handle->last_event.weight;
    handle->filter_state.index = (handle->filter_state.index + 1) % MAX_WINDOW;

    if (handle->filter_state.count < MAX_WINDOW)
        handle->filter_state.count++;

    float sum = 0;
    for (int i = 0; i < handle->filter_state.count; i++)
        sum += handle->filter_state.window[i];

    return sum / handle->filter_state.count;
}

static float ema(esp_weight_indicator_handle_t handle)
{

    if (handle->filter_state.count == 0)
    {
        handle->filter_state.ema_value = handle->last_event.weight;
        handle->filter_state.count = 1;
    }
    else
    {
        handle->filter_state.ema_value = handle->persistent.filter_config.alpha * handle->last_event.weight + (1 - handle->persistent.filter_config.alpha) * handle->filter_state.ema_value;
    }

    return handle->filter_state.ema_value;
}

static void stability_push(esp_weight_indicator_handle_t handle)
{
    handle->stability_state.buf[handle->stability_state.index] = handle->last_event.filtered;
    handle->stability_state.index = (handle->stability_state.index + 1) % STAB_WINDOW;

    if (handle->stability_state.count < STAB_WINDOW)
        handle->stability_state.count++;
}

static bool stability_check(esp_weight_indicator_handle_t handle)
{
    if (handle->stability_state.count < STAB_WINDOW)
        return false; // ainda enchendo

    float min = handle->stability_state.buf[0];
    float max = handle->stability_state.buf[0];

    for (int i = 1; i < handle->stability_state.count; i++)
    {
        if (handle->stability_state.buf[i] < min)
            min = handle->stability_state.buf[i];
        if (handle->stability_state.buf[i] > max)
            max = handle->stability_state.buf[i];
    }

    return (max - min) < handle->persistent.stable_threshold;
}

esp_err_t esp_weight_indicator_init(esp_weight_indicator_config_t *config, esp_weight_indicator_handle_t *handle)
{

    esp_err_t ret = ESP_OK;
    esp_weight_indicator_t *esp_weight_indicator = NULL;
    ESP_GOTO_ON_FALSE(config && handle, ESP_ERR_INVALID_ARG, err, TAG, "Invalid arguments");
    esp_weight_indicator = calloc(1, sizeof(esp_weight_indicator_t));
    ESP_GOTO_ON_FALSE(esp_weight_indicator, ESP_ERR_NO_MEM, err, TAG, "Failed to allocate memory for weight indicator");

    // esp_weight_indicator->weight = config->weight;
    esp_weight_indicator->driver = config->driver;

    // Initialize NVS for persistent storage
    esp_nvs_config_t nvs_config = {
        .name_space = "weight",
        .key = "persistent",
        .value_size = sizeof(weight_persistent_t), // enough to store the persistent data
    };

    ret = init_esp_nvs(&nvs_config, &esp_weight_indicator->esp_nvs_handle);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize NVS: %s", esp_err_to_name(ret));
        goto err;
    }

    // esp_nvs_change_key("weight", esp_weight_indicator->esp_nvs_handle);

    ret = esp_nvs_load_blob(esp_weight_indicator->esp_nvs_handle, &esp_weight_indicator->persistent, sizeof(weight_persistent_t));

    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "No existing calibration data found, using defaults");
        // Set default calibration values
        esp_weight_indicator->persistent.offset = -13300;
        esp_weight_indicator->persistent.scale = 4.9 / 1000000.0f;
        esp_weight_indicator->persistent.decimals = 2;
        esp_weight_indicator->persistent.stable_threshold = 0.5f; // example threshold
        esp_weight_indicator->persistent.stable_time_ms = 1000;   // example time
        esp_weight_indicator->persistent.filter_config.alpha = config->filter_config.alpha;
        esp_weight_indicator->persistent.filter_config.type = config->filter_config.type;
        esp_weight_indicator->persistent.filter_config.window_size = config->filter_config.window_size;
        esp_weight_indicator->persistent.stable_threshold = config->stability_config.threshold;
        esp_weight_indicator->persistent.tare = 0.0f;
        esp_weight_indicator->persistent.tare_enabled = false;
        ret = esp_nvs_save_blob(esp_weight_indicator->esp_nvs_handle, &esp_weight_indicator->persistent, sizeof(weight_persistent_t));
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to save default calibration data: %s", esp_err_to_name(ret));
            goto err;
        }
    }

    *handle = esp_weight_indicator;
    ESP_LOGI(TAG, "Weight indicator initialized successfully");
    ret = ESP_OK;
    return ret;

err:

    if (esp_weight_indicator)
    {
        free(esp_weight_indicator);
        esp_weight_indicator = NULL;
    }

    ESP_LOGE(TAG, "Failed to initialize weight indicator: %s", esp_err_to_name(ret));
    return ret;
}

esp_err_t esp_weight_indicator_update_cb_register(esp_weight_indicator_handle_t handle, esp_weight_indicator_update_cb_t cb)
{
    // testar se os handles são válidos
    ESP_RETURN_ON_FALSE(handle && cb, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    handle->update_callback = cb;
    ESP_LOGI(TAG, "Callback registered at %p", cb);
    return ESP_OK;
}

esp_err_t esp_weight_indicator_notify_cb_register(esp_weight_indicator_handle_t handle, esp_weight_indicator_notify_cb_t cb, void *ctx)
{
    // testar se os handles são válidos
    ESP_RETURN_ON_FALSE(handle && cb, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    handle->listeners[handle->count++] = (notify_entry_t){
        .cb = cb,
        .ctx = ctx};
    ESP_LOGI(TAG, "Callback registered at %p", cb);
    return ESP_OK;
}

esp_err_t esp_weight_indicator_update(esp_weight_indicator_handle_t handle)
{
    // testar se os handles são válidos
    esp_err_t ret = ESP_OK;
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    if (handle->update_callback)
    {
        ret = handle->update_callback(handle->driver, &handle->last_event.raw);

        handle->last_event.weight = (float)(handle->last_event.raw - handle->persistent.offset) * handle->persistent.scale;

        switch (handle->persistent.filter_config.type)
        {
        case WEIGHT_FILTER_MOVING_AVG:
            handle->last_event.filtered = moving_avg(handle);
            break;
        case WEIGHT_FILTER_EMA:
            handle->last_event.filtered = ema(handle);
            break;
        default:
            handle->last_event.filtered = handle->last_event.weight;
            break;
        }

        stability_push(handle);
        handle->last_event.stable = stability_check(handle);

        return ret;
    }

    ESP_LOGW(TAG, "Update callback not registered");
    return ESP_ERR_INVALID_STATE;
}

esp_err_t esp_weight_indicator_notify(esp_weight_indicator_handle_t handle)
{
    // testar se os handles são válidos
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    esp_weight_event_t event = {
        .weight = handle->last_event.weight,
        .raw = handle->last_event.raw,
        .stable = handle->last_event.stable,
        .filtered = handle->last_event.filtered};

    for (int i = 0; i < handle->count; i++)
    {
        if (handle->listeners[i].cb)
        {
            handle->listeners[i].cb(handle->listeners[i].ctx, &event);
        }
        else
        {
            ESP_LOGW(TAG, "Notify callback at index %d is NULL", i);
        }
    }

    return ESP_OK;
}

esp_err_t esp_weight_calibration_set_zero(esp_weight_indicator_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");

    if (!handle->last_event.stable)
        return ESP_ERR_INVALID_STATE;

    handle->persistent.offset = handle->last_event.raw;
    
    return esp_nvs_save_blob(handle->esp_nvs_handle, &handle->persistent, sizeof(weight_persistent_t));
}

esp_err_t esp_weight_calibration_set_scale(esp_weight_indicator_handle_t handle, float known_mass)
{

    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    
     if (!handle->last_event.stable)
        return ESP_ERR_INVALID_STATE;

    int32_t delta = handle->last_event.raw - handle->persistent.offset;

    if (delta == 0)
        return ESP_ERR_INVALID_STATE;

    handle->persistent.scale = known_mass / (float)delta;

    return esp_nvs_save_blob(handle->esp_nvs_handle, &handle->persistent, sizeof(weight_persistent_t));
}


