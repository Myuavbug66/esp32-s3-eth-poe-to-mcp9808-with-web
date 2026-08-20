idf_component_register(SRCS "mcp9808_temp_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES app_update esp_driver_i2c esp_driver_spi esp_driver_gpio
                                esp_eth esp_netif esp_event esp_http_server)
