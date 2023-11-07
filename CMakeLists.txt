idf_component_register(
        SRCS
            "ina228.cpp" "ina228.hpp"

        INCLUDE_DIRS
            "."

        REQUIRES
            "driver" "esp_event"
)
