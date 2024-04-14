#pragma once

#include <esp_err.h>

// pressure, temperature, humidity, lux
typedef void (*enviro_ready_f)(float, float, float, uint32_t);

esp_err_t enviro_sensor_init(enviro_ready_f cb, int update_interval);