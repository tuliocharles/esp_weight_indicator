#ifndef _esp_weight_indicator_H_
#define _esp_weight_indicator_H_

#include "esp_err.h"

// Here are the connection points with the weight indicator driver. 
// These are pointers to the functions of other components, such as the weight sensor driver,
// the display driver, the communication driver, etc. These pointers will be used to call the functions 
// of these components from the weight indicator code.

typedef struct
{
    float weight;
    float filtered;
    int32_t raw;
    bool stable;
} esp_weight_event_t;

typedef enum {
    WEIGHT_FILTER_NONE,
    WEIGHT_FILTER_MOVING_AVG,
    WEIGHT_FILTER_EMA
} esp_weight_filter_type_t;

typedef struct {
    esp_weight_filter_type_t type;
    int window_size; // for moving average
    float alpha; // for EMA
} weight_filter_config_t;

typedef struct {
    float threshold;
} stability_config_t;

typedef esp_err_t (*esp_weight_indicator_update_cb_t)(void *driver, int32_t *data_raw);

typedef esp_err_t (*esp_weight_indicator_notify_cb_t)(void *ctx , const esp_weight_event_t *e);

typedef struct esp_weight_indicator_t *esp_weight_indicator_handle_t;

typedef struct esp_weight_indicator_config_t
{
    uint8_t weight; // Weight indicator server address
    void *driver; // Pointer to the weight sensor driver, can be used to store a reference to the driver instance
    weight_filter_config_t filter_config; // Filter configuration
    stability_config_t stability_config; // Stability configuration
} esp_weight_indicator_config_t;

esp_err_t esp_weight_indicator_init(esp_weight_indicator_config_t *config, esp_weight_indicator_handle_t *handle);

esp_err_t esp_weight_indicator_update_cb_register(esp_weight_indicator_handle_t handle, esp_weight_indicator_update_cb_t cb);

esp_err_t esp_weight_indicator_notify_cb_register(esp_weight_indicator_handle_t handle, esp_weight_indicator_notify_cb_t cb, void *ctx );

esp_err_t esp_weight_indicator_update(esp_weight_indicator_handle_t handle);

esp_err_t esp_weight_indicator_notify(esp_weight_indicator_handle_t handle);

esp_err_t esp_weight_calibration_set_zero(esp_weight_indicator_handle_t handle);

esp_err_t esp_weight_calibration_set_scale(esp_weight_indicator_handle_t handle, float known_mass);

#endif