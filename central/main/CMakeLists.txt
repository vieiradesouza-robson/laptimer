idf_component_register(
    SRCS "waveshare_rgb_lcd_port.c" "central.c" "lvgl_port.c" "fsaeEvents.c" "../../lvgl/ui.c" "../../lvgl/ui_events.c" "../../lvgl/ui_helpers.c" "../../lvgl/screens/ui_accelScreen.c" "../../lvgl/screens/ui_skidScreen.c"  "../../lvgl/screens/ui_mainScreen.c"
    INCLUDE_DIRS ".")

idf_component_get_property(lvgl_lib lvgl__lvgl COMPONENT_LIB)
target_compile_options(${lvgl_lib} PRIVATE -Wno-format)
