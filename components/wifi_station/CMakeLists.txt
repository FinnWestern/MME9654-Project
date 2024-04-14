# Embed CA, certificate & key directly into binary
idf_component_register(
    SRCS wifi_station.c
    INCLUDE_DIRS .
    REQUIRES esp_wifi nvs_flash
    PRIV_REQUIRES wpa_supplicant
    EMBED_TXTFILES ca.pem client.crt client.key
)