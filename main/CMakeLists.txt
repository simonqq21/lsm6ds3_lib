idf_component_register(SRCS "lsm6ds3_lib_example.c" "lsm6ds3.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_i2c)
