idf_component_register(SRCS "ds18b20_temp_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES app_update esp_driver_spi esp_driver_gpio
                                esp_eth esp_netif esp_event esp_http_server
                                onewire_bus ds18b20)
