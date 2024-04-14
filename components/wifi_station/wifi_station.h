/* Functions to establish Wi-Fi connection.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
 */

#pragma once

#include <esp_wifi.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  esp_netif_t *sta;
} wifi_sta_params_t;

/**
 * @brief Initialize WiFi station
 *
 * @param p station parameters
 * 
 * @return `ESP_OK` on success
 */
esp_err_t init_wifi_sta(wifi_sta_params_t *p);

#ifdef __cplusplus
}
#endif
