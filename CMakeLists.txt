idf_component_register(
        SRCS
            "ina228.cpp" "ina228.hpp"

        INCLUDE_DIRS
            "."

        REQUIRES
            "driver" "esp_event" "esp_driver_gpio" "esp_driver_i2c"
)
