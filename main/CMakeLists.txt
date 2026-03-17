set(srcs "main.c"
        "u8g2_font_korean.c"
         "gatt_svr.c")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "."
                    REQUIRES nvs_flash nimble u8g2)