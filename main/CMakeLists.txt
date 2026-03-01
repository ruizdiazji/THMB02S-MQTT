set(PROJECT_NAME "modbus_master")

idf_component_register(
    SRCS
        "main.c"
        "app_config.c"
        "ble_config.c"
        "network_manager.c"
        "modbus_service.c"
    INCLUDE_DIRS "."
    REQUIRES driver nvs_flash esp_wifi esp_netif esp_event mqtt bt
)
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
