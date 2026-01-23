idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_lcd
        esp_timer
        driver
        esp_psram
    PRIV_REQUIRES
        esp_lcd_touch_ft6336
)
