## Weight Indicator Middleware for ESP-IDF

This component provides a modular middleware layer for weight measurement systems based on load cells (e.g., HX711) on ESP32 platforms. It abstracts sensor acquisition, signal filtering, stability detection, and event notification into a unified interface.

The component supports multiple filtering strategies (moving average and exponential moving average), configurable stability thresholds, and a callback-based architecture for integration with external modules such as
