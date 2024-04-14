idf_component_register(
    SRCS enviro_sensor.c
    INCLUDE_DIRS .
    REQUIRES bmp280 tsl2561
)