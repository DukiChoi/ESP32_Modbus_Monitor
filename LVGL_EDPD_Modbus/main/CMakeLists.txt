set(SOURCES
    main.c
    modbus_src/lv_modbus.c
    modbus_src/lv_modbus_connection.c
    modbus_src/lv_edpd_widgets.c
    modbus_src/lv_sensor_profiles.c
    modbus_src/lv_app_font.c)
idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS . modbus_src
                    REQUIRES lvgl_esp32_drivers lvgl_touch lvgl_tft lvgl
                    PRIV_REQUIRES driver esp_event esp_timer freertos log)

target_compile_definitions(${COMPONENT_LIB} PRIVATE LV_CONF_INCLUDE_SIMPLE=1)
