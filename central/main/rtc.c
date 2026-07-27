#include "rtc.h"
#include "central.h"
#include "driver/i2c_master.h"

// DS1307 RTC on the TinyRTC I2C module
#define DS1307_I2C_ADDR     0x68
#define DS1307_REG_SECONDS  0x00

static i2c_master_bus_handle_t rtc_bus_handle = NULL;
static i2c_master_dev_handle_t rtc_dev_handle = NULL;

static uint8_t bcd_to_dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

void rtc_module_init(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = RTC_SDA_GPIO,
        .scl_io_num = RTC_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_new_master_bus(&bus_config, &rtc_bus_handle);

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DS1307_I2C_ADDR,
        .scl_speed_hz = 100000,
    };
    i2c_master_bus_add_device(rtc_bus_handle, &dev_config, &rtc_dev_handle);
}

rtc_datetime_t rtc_get_timestamp(void) {
    rtc_datetime_t dt = {0};

    if (rtc_dev_handle == NULL) {
        return dt;
    }

    uint8_t reg = DS1307_REG_SECONDS;
    uint8_t raw[7] = {0};

    esp_err_t err = i2c_master_transmit_receive(rtc_dev_handle, &reg, 1, raw, sizeof(raw), 100);
    if (err != ESP_OK) {
        return dt;
    }

    // DS1307 registers are BCD; assumes the RTC is kept in 24-hour mode
    dt.second = bcd_to_dec(raw[0] & 0x7F); // bit 7 is the clock-halt flag
    dt.minute = bcd_to_dec(raw[1] & 0x7F);
    dt.hour   = bcd_to_dec(raw[2] & 0x3F); // bit 6 selects 12/24-hour mode
    dt.day    = bcd_to_dec(raw[4] & 0x3F);
    dt.month  = bcd_to_dec(raw[5] & 0x1F);
    dt.year   = 2000 + bcd_to_dec(raw[6]);

    return dt;
}
