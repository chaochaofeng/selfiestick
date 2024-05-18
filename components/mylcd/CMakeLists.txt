set(srcs "esp_lcd_panel_st7735s.c")
set(priv_requires "esp_lcd")

idf_component_register(SRCS ${srcs}
                       PRIV_REQUIRES ${priv_requires})
