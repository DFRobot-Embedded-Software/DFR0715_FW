#include "main.h"
#include "hardDevInit.h"

static const char* TAG = "hardDevInit";

QueueHandle_t user_uart_queue;
static i2s_chan_handle_t rx_handle = NULL;   // I2S rx channel handler

const static audio_codec_data_if_t* record_data_if = NULL;
const static audio_codec_ctrl_if_t *record_ctrl_if = NULL;
const static audio_codec_if_t *record_codec_if = NULL;
esp_codec_dev_handle_t record_dev;

/* ----------------------------- gpio -------------------------- */

/**
 * @brief gpio 初始化
 * @n 1.上电检测拨码开关，通过拨码开关的电平确定是I2C还是UART（默认I2C）;
 * @n 2.按下按键，模块唤醒，发送ID 0;
 * @n 3.当按下按键或识别到唤醒词时，返回0，LED灯点亮;
 */
void dfr0715_gpio_init(void)
{
    // 零-初始化配置结构
    gpio_config_t io_conf = {};
    // 禁用中断
    io_conf.intr_type = GPIO_INTR_DISABLE;
    // 设置为输出模式
    io_conf.mode = GPIO_MODE_OUTPUT;
    // 引脚位的掩码
    io_conf.pin_bit_mask = (1ULL << GPIO_LED);
    // 禁用下拉模式
    io_conf.pull_down_en = 0;
    // 禁用上拉模式
    io_conf.pull_up_en = 0;
    // 用给定的设置配置GPIO
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // 设置为输入模式
    io_conf.mode = GPIO_MODE_INPUT;
    // 引脚位的掩码
    io_conf.pin_bit_mask = (1ULL << GPIO_COMM_MODE);
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // 上升沿中断   有硬件消抖
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    // 引脚位的掩码
    io_conf.pin_bit_mask = (1ULL << GPIO_KEY);
    // 启用上拉模式
    io_conf.pull_up_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
}

/* ----------------------------- communication interface -------------------------- */

void dfr0715_uart_init(void)
{
    /*
     * Configure parameters of an UART driver,
     * communication pins and install the driver
     */
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;
    
#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_DATA_LEN * 20, UART_DATA_LEN * 10, 30, &user_uart_queue, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_IO, UART_RX_IO, UART_RTS_IO, UART_CTS_IO));
}

void dfr0715_i2c_slave_init(void)
{
    int i2c_slave_port = I2C_SLAVE_NUM;
    i2c_config_t conf_slave = {
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .mode = I2C_MODE_SLAVE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = I2C_SLAVE_ADDR,
    };

    ESP_ERROR_CHECK(i2c_param_config(i2c_slave_port, &conf_slave));

    ESP_ERROR_CHECK(i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0));
}

/* ----------------------------- codec --------------------------- */

static void codec_i2c_init(i2c_port_t i2c_num, uint32_t clk_speed)
{
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .scl_io_num = CODEC_I2C_SCL,
        .sda_io_num = CODEC_I2C_SDA,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = clk_speed,
    };
    ESP_ERROR_CHECK(i2c_param_config(i2c_num, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_num, i2c_cfg.mode, 0, 0, 0));
}

static void codec_i2s_init(i2s_port_t i2s_num, uint32_t sample_rate, int channel_format, int bits_per_chan)
{
    i2s_slot_mode_t channel_fmt = I2S_SLOT_MODE_STEREO;
    if (channel_format == 1) {
        channel_fmt = I2S_SLOT_MODE_MONO;
    } else if (channel_format == 2) {
        channel_fmt = I2S_SLOT_MODE_STEREO;
    } else {
        ESP_LOGE(TAG, "Unable to configure channel_format %d", channel_format);
        channel_format = 1;
        channel_fmt = I2S_SLOT_MODE_MONO;
    }

    if (bits_per_chan != 16 && bits_per_chan != 32) {
        ESP_LOGE(TAG, "Unable to configure bits_per_chan %d", bits_per_chan);
        bits_per_chan = 16;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(i2s_num, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_handle));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bits_per_chan, channel_fmt),
        .gpio_cfg = {
            .mclk = CODEC_I2S_MCLK,
            .bclk = CODEC_I2S_SCLK,
            .ws   = CODEC_I2S_LRCK,
            .dout = CODEC_I2S_DOUT,
            .din  = CODEC_I2S_SDIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    // std_cfg.clk_cfg.mclk_multiple = EXAMPLE_MCLK_MULTIPLE;   //The default is I2S_MCLK_MULTIPLE_256. If not using 24-bit data width, 256 should be enough
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
}

void es7243e_codec_init(void)
{
    /* Initialize I2C peripheral */
    codec_i2c_init(CODEC_I2C_NUM, CODEC_I2C_CLK);
    /* Initialize I2S peripheral */
    codec_i2s_init(CODEC_I2S_NUM, 16000, I2S_SLOT_MODE_STEREO, I2S_DATA_BIT_WIDTH_32BIT);

    /* Initialize es7243e codec */
    // Do initialize of related interface: data_if, ctrl_if and gpio_if
    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = CODEC_I2S_NUM,
        .rx_handle = rx_handle,
        .tx_handle = NULL,
    };
    record_data_if = audio_codec_new_i2s_data(&i2s_cfg);

    // audio_codec_i2c_cfg_t i2c_cfg = {.addr = ES7243E_CODEC_DEFAULT_ADDR};
    audio_codec_i2c_cfg_t i2c_cfg = {.addr = 0x28};
    record_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    // New input codec interface
    es7243e_codec_cfg_t es7243e_cfg = {
        .ctrl_if = record_ctrl_if,
    };
    record_codec_if = es7243e_codec_new(&es7243e_cfg);
    // New input codec device
    esp_codec_dev_cfg_t dev_cfg = {
        .codec_if = record_codec_if,
        .data_if = record_data_if,
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
    };
    record_dev = esp_codec_dev_new(&dev_cfg);

    esp_codec_dev_sample_info_t fs = {
        .sample_rate = 16000,
        .channel = 2,
        .bits_per_sample = 32,
    };
    esp_codec_dev_open(record_dev, &fs);
    // esp_codec_dev_set_in_gain(record_dev, RECORD_VOLUME);
    esp_codec_dev_set_in_channel_gain(record_dev, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(0), RECORD_VOLUME);
    esp_codec_dev_set_in_channel_gain(record_dev, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(1), RECORD_VOLUME);
    esp_codec_dev_set_in_channel_gain(record_dev, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(2), 0.0);   // reference
    esp_codec_dev_set_in_channel_gain(record_dev, ESP_CODEC_DEV_MAKE_CHANNEL_MASK(3), RECORD_VOLUME);
}
