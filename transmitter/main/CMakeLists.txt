idf_component_register(SRCS "landslide_transmitter.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_adc esp_driver_gpio esp_driver_i2c driver esp_timer)
