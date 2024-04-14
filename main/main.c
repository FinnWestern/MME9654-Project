/* WiFi station example with additions for WPA2 Enterprise

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <stdint.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include "esp_spiffs.h"
#include "esp_http_server.h"
#include "wifi_station.h"
#include "enviro_sensor.h"

static const char *TAG = "example";

#define EXAMPLE_UPDATE_INTERVAL  CONFIG_UPDATE_INTERVAL

#ifdef CONFIG_SYNC_TIME
#define EXAMPLE_RESYNC_INTERVAL  CONFIG_SNTP_RESYNC_INTERVAL
#define EXAMPLE_SNTP_NUM_SERVERS CONFIG_SNTP_NUM_SERVERS
#define EXAMPLE_SNTP_SERVER1     CONFIG_SNTP_TIME_SERVER1
#define EXAMPLE_SNTP_SERVER2     CONFIG_SNTP_TIME_SERVER2
#define EXAMPLE_SNTP_SERVER3     CONFIG_SNTP_TIME_SERVER3
#endif

#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 48
#endif

#define INDEX_HTML_PATH "/spiffs/index.html"

// Global variables
static char index_html[4096];
static float pressure = 101.2;                                 // Pressure from BME280
static float temperature = 22.3;                              // Temperature from BME280
static float humidity = 33.4;                                 // Humidity from BME280
static bool enabled = false;

static httpd_handle_t server = NULL;
static int ws_fd = -1;

#ifdef CONFIG_SYNC_TIME
static bool sntp_initialized = false;

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Time synchronized to server");
    sntp_initialized = true;
}

static void print_servers(void)
{
    ESP_LOGI(TAG, "List of configured NTP servers:");

    for (uint8_t i = 0; i < EXAMPLE_SNTP_NUM_SERVERS; ++i){
        if (esp_sntp_getservername(i)){
            ESP_LOGI(TAG, "server %d: %s", i, esp_sntp_getservername(i));
        } else {
            // we have either IPv4 or IPv6 address, let's print it
            char buff[INET6_ADDRSTRLEN];
            ip_addr_t const *ip = esp_sntp_getserver(i);
            if (ipaddr_ntoa_r(ip, buff, INET6_ADDRSTRLEN) != NULL)
                ESP_LOGI(TAG, "server %d: %s", i, buff);
        }
    }
}

// Obtain current time from STNP server
static void obtain_time(void)
{
    ESP_LOGI(TAG, "Initializing and starting SNTP");
#if EXAMPLE_SNTP_NUM_SERVERS == 2
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(2,
                               ESP_SNTP_SERVER_LIST(EXAMPLE_SNTP_SERVER1, EXAMPLE_SNTP_SERVER2));
#elif EXAMPLE_SNTP_NUM_SERVERS == 3
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(3,
                               ESP_SNTP_SERVER_LIST(EXAMPLE_SNTP_SERVER1, EXAMPLE_SNTP_SERVER2, 
                                EXAMPLE_SNTP_SERVER3));
#else
    /*
     * This is the basic default config with one server and starting the service
     */
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(EXAMPLE_SNTP_SERVER1);
#endif
    config.sync_cb = time_sync_notification_cb;     // Note: This is only needed if we want
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    config.smooth_sync = true;
#endif
    esp_netif_sntp_init(&config);
    print_servers();

    int retry = 0;
    const int retry_count = 20;
    while (esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS) == ESP_ERR_TIMEOUT && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
    }

    esp_netif_sntp_deinit(); 
}
#endif

static void station_task(void *pvParameters)
{
    esp_netif_t *sta = (esp_netif_t *) pvParameters;
#ifdef CONFIG_SYNC_TIME
    time_t now = 0;
    struct tm timeinfo = { 0 };
    time_t last_resync = 0;
    char strftime_buf[64];
#endif
    esp_netif_ip_info_t ip;
    memset(&ip, 0, sizeof(esp_netif_ip_info_t));

    while (1) {
        vTaskDelay(EXAMPLE_UPDATE_INTERVAL * 1000 / portTICK_PERIOD_MS);

        if (esp_netif_get_ip_info(sta, &ip) == 0) {
            ESP_LOGI(TAG, "~~~~~~~~~~~");
            ESP_LOGI(TAG, "IP:"IPSTR, IP2STR(&ip.ip));
            ESP_LOGI(TAG, "MASK:"IPSTR, IP2STR(&ip.netmask));
            ESP_LOGI(TAG, "GW:"IPSTR, IP2STR(&ip.gw));
#ifdef CONFIG_SYNC_TIME
            time(&now);
            localtime_r(&now, &timeinfo);
            // Is time set? If not, tm_year will be (1970 - 1900).
            if (timeinfo.tm_year < (2016 - 1900)) {
                ESP_LOGI(TAG, "Time is not set yet. Getting time over NTP.");
                obtain_time();
                // update 'now' variable with current time
                time(&now);
                last_resync = now;
            }

            // Resync time if after resync interval has passed
            if (difftime(now, last_resync) >= EXAMPLE_RESYNC_INTERVAL) {
                ESP_LOGI(TAG, "Resyncing time. Getting time over NTP.");
                obtain_time();
                // update 'now' variable with current time
                time(&now);
                last_resync = now;

                if (sntp_get_sync_mode() == SNTP_SYNC_MODE_SMOOTH) {
                    struct timeval outdelta;
                    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_IN_PROGRESS) {
                        adjtime(NULL, &outdelta);
                        ESP_LOGI(TAG, "Waiting for time adjustment ... outdelta = %jd sec: %li ms: %li us",
                                    (intmax_t)outdelta.tv_sec,
                                    outdelta.tv_usec/1000,
                                    outdelta.tv_usec%1000);
                        vTaskDelay(1000 / portTICK_PERIOD_MS);
                    }
                }
            }
            // Set timezone to Eastern Standard Time and print local time
            setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
            tzset();
            localtime_r(&now, &timeinfo);
            strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
            ESP_LOGI(TAG, "Current date/time in London, ON: %s", strftime_buf);
#endif
            ESP_LOGI(TAG, "~~~~~~~~~~~");
        }
        else {
            ESP_LOGI(TAG, "Not connected");
        }
    }
}

static void init_html(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

    memset((void *)index_html, 0, sizeof(index_html));
    struct stat st;
    if (stat(INDEX_HTML_PATH, &st))
    {
        ESP_LOGE(TAG, "index.html not found");
        return;
    }

    FILE *fp = fopen(INDEX_HTML_PATH, "r");
    if (fread(index_html, st.st_size, 1, fp) != st.st_size)
    {
        ESP_LOGE(TAG, "fread failed");
    }
    fclose(fp);
}

static void send_async(void *arg)
{
    if (ws_fd < 0)
    {
        return;
    }

#ifdef CONFIG_SYNC_TIME    
    time_t now = 0;
    struct tm timeinfo = { 0 };
    time(&now);
    localtime_r(&now, &timeinfo);
#endif

    char buff[128];
    memset(buff, 0, sizeof(buff));
    // format JSON data
#ifdef CONFIG_SYNC_TIME
    sprintf(buff, "{\"state\": \"%s\", \"pres\": %.2f, \"temp\": %.2f, \"hum\": %.2f, \"time\": \"%02d:%02d:%02d\"}",
            enabled ? "ON" : "OFF", pressure, temperature, humidity,
            timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
#else
//    sprintf(buff, "{\"state\": \"%s\", \"pres\": %.2f, \"temp\": %.2f, \"hum\": %.2f, \"time\": \"00:00:00\"}",
    sprintf(buff, "{\"state\": \"%s\", \"pres\": %.2f, \"temp\": %.2f, \"hum\": %.2f}",
            enabled ? "ON" : "OFF", pressure, temperature, humidity);
#endif

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t *)buff;
    ws_pkt.len = strlen(buff);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_send_frame_async(server, ws_fd, &ws_pkt);
}

static esp_err_t handle_http_get(httpd_req_t *req)
{
    if (index_html[0] == 0)
    {
        httpd_resp_set_status(req, HTTPD_500);
        return httpd_resp_send(req, "no index.html", HTTPD_RESP_USE_STRLEN);
    }
    return httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handle_ws_req(httpd_req_t *req)
{
    enabled = !enabled;

    httpd_ws_frame_t ws_pkt;
    uint8_t buff[16];
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = buff;
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;

    httpd_ws_recv_frame(req, &ws_pkt, sizeof(buff));

    if (!enabled)
    {
        httpd_queue_work(server, send_async, NULL);
    }
    return ESP_OK;
}

static esp_err_t handle_socket_opened(httpd_handle_t hd, int sockfd)
{
    ws_fd = sockfd;
    return ESP_OK;
}

static void handle_socket_closed(httpd_handle_t hd, int sockfd)
{
    if (sockfd == ws_fd)
    {
        ws_fd = -1;
    }
}

static void start_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.open_fn = handle_socket_opened;
    config.close_fn = handle_socket_closed;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_uri_t uri_get = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = handle_http_get,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &uri_get);

        httpd_uri_t ws = {
            .uri = "/ws",
            .method = HTTP_GET,
            .handler = handle_ws_req,
            .user_ctx = NULL,
            .is_websocket = true};
        httpd_register_uri_handler(server, &ws);
    }
}

static void update_reading(float pres, float temp, float hum)
{
    pressure = pres;
    temperature = temp;
    humidity = hum;

    ESP_LOGI(TAG, "Pressure: %.2f kPa, Temperature: %.2f C, Humidity: %.2f", pressure, temperature, humidity);
    if (server != NULL && enabled)
    {
        httpd_queue_work(server, send_async, NULL);
    }
}

void app_main(void)
{
    init_html();

    wifi_sta_params_t p = {
        .sta = NULL
    };
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    ESP_ERROR_CHECK(init_wifi_sta(&p));
    xTaskCreate(&station_task, "station_task", 4096, (void *) p.sta, 5, NULL);
    start_server();
    while(1){
        update_reading(101.2, 22.3, 33.4);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    //enviro_sensor_init(update_reading, 5000);
}
