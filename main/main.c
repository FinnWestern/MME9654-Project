/*
    MME 9654 Project Code for ESP32-S3

    This program runs a web page to update pH values from BREAD Slices, 
    log data to an SD card, and accept user input to change the PID
    tuning of each Slice.

    Author: Finn Hafting
*/

#include <string.h>
#include <stdint.h>
#include <sys/stat.h>
#include <time.h>
#include <math.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"
#include "esp_spiffs.h"
#include "esp_http_server.h"
#include "wifi_station.h"

#include "driver/rmt_tx.h"  // for LED 
#include "led_strip.h"

#include <sys/unistd.h>     // for SD card
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "driver/i2c.h"     // to send I2C data

static const char *TAG = "MME 9654 Project - pH Controller";

#define EXAMPLE_UPDATE_INTERVAL  CONFIG_UPDATE_INTERVAL     // how often to sync time with SNTP server

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

#define INDEX_HTML_PATH "/spiffs/index.html"    // HTML code for web page

#define MOUNT_POINT "/sdcard"

// Pin assignments for SPI SD card reader
#define PIN_NUM_MISO         41
#define PIN_NUM_MOSI         40
#define PIN_NUM_CLK          39
#define PIN_NUM_CS           38

// Variables for BREAD Slices

#define NUM_PCHL_SLICES     3   // number of pH controller Slices

typedef union //Define a float that can be broken up and sent via I2C
{
  float number;
  uint8_t bytes[4];
} FLOATUNION_t;

typedef struct {
    int address;        // I2C address
    float pHSetpoint;   // pH setpoint
    float currentPH;    // current pH
    float Kp;           // Kp value for PID control
    float Ki;           // KI value for PID control
    float Kd;           // KD value for PID control
} PHCL_t;      // Structures for managing the data from each PHCL Slice

PHCL_t PHCL[NUM_PCHL_SLICES];

static char index_html[16384];  // buffer for writing web page data to SPIFFS
static bool enabled = false;    // to set if data is being updated on web page
static bool sdLog = false;      // to set it data is being logged to SD card

static httpd_handle_t server = NULL;
static int ws_fd = -1;

#ifdef CONFIG_SYNC_TIME
static bool sntp_initialized = false;
char strftime_buf[64];

// I2C
#define I2C_MASTER_SCL_IO           2      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           1      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          100000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// for initializing I2C master
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

#define LED_STRIP_BLINK_GPIO  48    // built in LED pid
#define LED_STRIP_LED_NUMBERS 1     // number of LEDs in a strip (1 in this case)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)    // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)

// configure built in LED
led_strip_handle_t configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_BLINK_GPIO,   // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_NUMBERS,        // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .rmt_channel = 0,
#else
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .flags.with_dma = false,               // DMA feature is available on ESP target like ESP32-S3
#endif
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(TAG, "Created LED strip object with RMT backend");
    return led_strip;
}

// notify serial port when time has been updated
void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Time synchronized to server");
    sntp_initialized = true;
}

// print the current SNTP servers
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

// Obtain current time from SNTP server
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

// function to update the current time with SNTP server
static void station_task(void *pvParameters)
{
    esp_netif_t *sta = (esp_netif_t *) pvParameters;
#ifdef CONFIG_SYNC_TIME
    struct tm timeinfo = { 0 };
    time_t last_resync = 0;
    time_t now = 0;
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

// initialize HTML web page
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

// to send JSON string via websockets to web page
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
    memset(buff, 0, sizeof(buff));  // set buffer array to zero
    // format JSON data
#ifdef CONFIG_SYNC_TIME
    // buffer string to send as JSON data to web page
    sprintf(buff, "{\"state\": \"%s\",\"sdLog\": \"%s\", \"pH0\": %.2f, \"pH1\": %.2f, \"pH2\": %.2f, \"time\": \"%02d:%02d:%02d\"}",
            enabled ? "ON" : "OFF", sdLog ? "LOGGING..." : "OFF", PHCL[0].currentPH, PHCL[1].currentPH, PHCL[2].currentPH,
            timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
#else
//    sprintf(buff, "{\"state\": \"%s\", \"pres\": %.2f, \"temp\": %.2f, \"hum\": %.2f, \"time\": \"00:00:00\"}",
    sprintf(buff, "{\"state\": \"%s\", \"pres\": %.2f, \"temp\": %.2f, \"hum\": %.2f}",
            enabled ? "ON" : "OFF", pressure, temperature, humidity);
#endif

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t *)buff;   // configure websocket payload
    ws_pkt.len = strlen(buff);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    httpd_ws_send_frame_async(server, ws_fd, &ws_pkt);  // send websocket payload
}

// function for handling HTTP GET requests (setup HTML page)
static esp_err_t handle_http_get(httpd_req_t *req)
{
    if (index_html[0] == 0)
    {
        httpd_resp_set_status(req, HTTPD_500);
        return httpd_resp_send(req, "no index.html", HTTPD_RESP_USE_STRLEN);
    }
    return httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
}

// for handling websocket requests
static esp_err_t handle_ws_req(httpd_req_t *req)
{
    // setup websocket packet
    httpd_ws_frame_t ws_pkt;
    uint8_t buff[64];
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    memset(buff, 0, sizeof(buff));
    ws_pkt.payload = buff;
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;

    httpd_ws_recv_frame(req, &ws_pkt, sizeof(buff));    // receive websocket packet

    ESP_LOGI(TAG, "%s", ws_pkt.payload);

    char data_byte[32];
    for(int i=0; i<sizeof(data_byte); i++){
        data_byte[i] = (char)ws_pkt.payload[i];     // convert payload to char array
    }
    ESP_LOGI(TAG, "%s", data_byte);

    int last_index = 4;         // start translation after id and first int
    char id = data_byte[0];     // message ID is first byte
    ESP_LOGI(TAG, "%d", id);    
    int int1 = data_byte[2] - 48;   // subtract ASCII '0' to get int value from second byte
    ESP_LOGI(TAG, "%d",int1);

    float float_values[4];
    int float_num = 0;
    if(id == 'P'){      // data incoming is PID and setpoint values
        char temp[8];
        float tempf;
        for(int i=4; i<sizeof(data_byte);i++){
            if(data_byte[i] == ','){    // stop at separating character
                tempf = atof(temp);     // translate previous bytes to floats

                if(!isnan(tempf)) float_values[float_num] = tempf;
                else float_values[float_num] = 0;       // assign float value to array

                last_index = i+1;       // set index of start of next value after comma
                memset(temp, 0, sizeof(temp));  // reset temp array for accepting new characters
                ESP_LOGI(TAG, "%.2f", float_values[float_num]);
                float_num++;    // increment float array
            }else{
                temp[i-last_index] = data_byte[i];      // assign current character to array
            }
        }
    }
    if(id == 'S'){      // data incoming is to set data update enable
        enabled = int1;
        httpd_queue_work(server, send_async, NULL);
    }
    if(id == 'D'){      // data incoming is to set SD logging enable
        sdLog = int1;
        httpd_queue_work(server, send_async, NULL);
    }
    if(id == 'P'){      // assign PID and setpoint values to Slice structures
        PHCL[int1].pHSetpoint = float_values[0];
        PHCL[int1].Kp = float_values[1];
        PHCL[int1].Ki = float_values[2];
        PHCL[int1].Kd = float_values[3];

        FLOATUNION_t float_union[4];
        for(int i=0;i<4;i++) float_union[i].number = float_values[i];   // write float values to float union

        uint8_t write_buff[17];

        write_buff[0] = 1;  // id of pH controller on Slice
        for(int i=0;i<4;i++){
            for(int j=0;j<4;j++){
                write_buff[i*4+j+1] = float_union[i].bytes[j];  // write individual bytes from float values to char array
            }
        }

        // write char array to specific Slice indicated by int1
        i2c_master_write_to_device(I2C_MASTER_NUM, int1+10, write_buff, sizeof(write_buff), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
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

// to configure server on startup
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

// to gather pH readings from Slices
static void update_reading()
{
    FLOATUNION_t float1, float2, float3;

    // request 4 bytes of data from each Slice
    i2c_master_read_from_device(I2C_MASTER_NUM, 0x0A, float1.bytes, 4, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_master_read_from_device(I2C_MASTER_NUM, 0x0B, float2.bytes, 4, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_master_read_from_device(I2C_MASTER_NUM, 0x0C, float3.bytes, 4, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    // assign Slice structure value to incoming float number
    PHCL[0].currentPH = float1.number;
    PHCL[1].currentPH = float2.number;
    PHCL[2].currentPH = float3.number;

    if (server != NULL && enabled)
    {
        ESP_LOGI(TAG, "PID0: %.2f, %.2f, %.2f, %.2f, pH0: %.2f", PHCL[0].pHSetpoint, PHCL[0].Kp, PHCL[0].Ki, PHCL[0].Kd, PHCL[0].currentPH);
        ESP_LOGI(TAG, "PID1: %.2f, %.2f, %.2f, %.2f, pH1: %.2f", PHCL[1].pHSetpoint, PHCL[1].Kp, PHCL[1].Ki, PHCL[1].Kd, PHCL[1].currentPH);
        ESP_LOGI(TAG, "PID2: %.2f, %.2f, %.2f, %.2f, pH2: %.2f", PHCL[2].pHSetpoint, PHCL[2].Kp, PHCL[2].Ki, PHCL[2].Kd, PHCL[2].currentPH);
        httpd_queue_work(server, send_async, NULL); // if data update is enabled, update web page with new data
    }
}

void app_main(void)
{
    int led_brightness = 50;    // brighness percentage (0-100%)
    int onE = 0, onSD = 0, onBoth = 0;     // LED toggle variables

    led_strip_handle_t led_strip = configure_led();

    // Indicate initialization (orange)
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 165 * led_brightness / 100,  //green
                                                      255 * led_brightness / 100,  //red
                                                      0));  //blue
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));

    // assign default values in pH Slice structures
    for(int i=0;i<NUM_PCHL_SLICES;i++){
        PHCL[i].address = 10+i;     // I2C addresses 10+
        PHCL[i].pHSetpoint = 7;     // default setpoint to start
        PHCL[i].Kp = 0;             // zero PID tunings
        PHCL[i].Ki = 0;
        PHCL[i].Kd = 0;
    }
    init_html();    // configure web page

    // configure WiFi
    wifi_sta_params_t p = {
        .sta = NULL
    };
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    ESP_ERROR_CHECK(init_wifi_sta(&p));
    xTaskCreate(&station_task, "station_task", 4096, (void *) p.sta, 5, NULL);
    start_server();     // start web server

//------------------------------I2C-----------------------------------
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

// ------------------------------SD CARD------------------------------------
    FILE *f;
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SPI peripheral");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                    "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } 
        else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                    "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    // Create a file
    const char *data_file = MOUNT_POINT"/pHData.csv";

    // Open file for writing
    ESP_LOGI(TAG, "Writing header to %s", data_file);
    f = fopen(data_file, "w");  // Create new file, overwriting existing
    fprintf(f, "\nTime,pH0,pH1,pH2\n");    // write header to SD card
    fclose(f);
    ESP_LOGI(TAG, "File written");

// ------------------------ TO RUN CONTINUOUSLY ---------------------------

    while(1){
        update_reading();   // update webpage with Slice pH readings

        if(enabled && sdLog){     // toggle blue
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0,  //green
                                                              0,  //red
                                                              255 * led_brightness * onBoth / 100));  //blue
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            onBoth ^= 1;    // toggle
        }
        else if(enabled){    // toggle red
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0,  //green
                                                              255 * led_brightness * onE / 100,  //red
                                                              0));  //blue
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            onE ^= 1;    // toggle
        }
        else if(sdLog){      // toggle green
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 255 * led_brightness * onSD / 100,  //green
                                                              0,  //red
                                                              0));  //blue
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            onSD ^= 1;    // toggle
        }
        else{       // off
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0,  //green
                                                              0,  //red
                                                              0));  //blue
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        }

        // if logging is enabled, write to SD card
        if(sdLog){
            // Open file for writing
            f = fopen(data_file, "a");  // Add to existing file
            if (f == NULL) {
                ESP_LOGE(TAG, "Failed to open file for writing");
                return;
            }

            // Write pH data to SD card        
            fprintf(f, "%s, %.2f, %.2f, %.2f\n", strftime_buf, PHCL[0].currentPH, PHCL[1].currentPH, PHCL[2].currentPH);

            // Close file
            fclose(f);
            ESP_LOGI(TAG, "File written");
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);  // repeat every second
    }
}
