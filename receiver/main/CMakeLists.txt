idf_component_register(SRCS "landslide_receiver.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver esp_common freertos log esp_wifi esp_netif esp_event nvs_flash esp_http_client mbedtls)