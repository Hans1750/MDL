idf_component_register(SRCS
    "main.c"
    "i2c_driver.c"
    "amoled_driver.c"
    "initSequence.c"
    "power_driver.cpp"
    "display_s3.c"
   
    "display_s3_pro.c"
    "display_s3_qt.c"
    "display_esp32.c"
    "display_long.c"
    "display_hmi.c"
    "display_rgb.cpp"
    "display_watch_s3.c"
    "touch_driver.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_lcd SensorLib )


