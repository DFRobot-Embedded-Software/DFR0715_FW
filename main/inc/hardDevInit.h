#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

 /* --------------------- 定义和静态变量 ------------------ */

#define GPIO_COMM_MODE   GPIO_NUM_42            // 上电检测拨码开关   GPIO42
#define GPIO_KEY         GPIO_NUM_38            // 唤醒按键   GPIO38
#define GPIO_LED         GPIO_NUM_11            // 唤醒指示灯   GPIO11

#define I2C_DATA_LEN           (128)                  /*!< Data buffer length of test buffer */
#define I2C_SLAVE_SCL_IO       GPIO_NUM_13          /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO       GPIO_NUM_12          /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM          I2C_NUM_1            /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN   (2 * I2C_DATA_LEN)   /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN   (2 * I2C_DATA_LEN)   /*!< I2C slave rx buffer size */
#define I2C_SLAVE_ADDR         (0x14)                 /*!< ESP32 slave address, you can set any 7bit value */

#define UART_TX_IO       GPIO_NUM_12            // UART接口的tx引脚号
#define UART_RX_IO       GPIO_NUM_13            // UART接口的rx引脚号
#define UART_RTS_IO      UART_PIN_NO_CHANGE
#define UART_CTS_IO      UART_PIN_NO_CHANGE
#define UART_DATA_LEN    (64)                   // UART 接收发送buf大小
#define UART_PORT_NUM    UART_NUM_1             // UART 端口号
#define UART_BAUD_RATE   (57600)                 // UART 波特率

#define CODEC_I2C_NUM    I2C_NUM_0
#define CODEC_I2C_CLK    (600000)
#define CODEC_I2C_SCL    GPIO_NUM_48
#define CODEC_I2C_SDA    GPIO_NUM_47
#define RECORD_VOLUME    (30.0)
#define CODEC_I2S_NUM    I2S_NUM_1
#define CODEC_I2S_LRCK   GPIO_NUM_18
#define CODEC_I2S_MCLK   GPIO_NUM_21
#define CODEC_I2S_SCLK   GPIO_NUM_19
#define CODEC_I2S_SDIN   GPIO_NUM_20
#define CODEC_I2S_DOUT   GPIO_NUM_NC

extern QueueHandle_t user_uart_queue;
extern esp_codec_dev_handle_t record_dev;

void dfr0715_gpio_init(void);
void dfr0715_uart_init(void);
void dfr0715_i2c_slave_init(void);
void es7243e_codec_init(void);

#ifdef __cplusplus
}
#endif
