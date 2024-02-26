#include "main.h"
#include "moduleFunc.h"
#include "hardDevInit.h"

// static const char* TAG = "moduleFunc";

// 长命令词处理
uint8_t longComID = 0;
char handLongpStr[128] = { 0 };

// 与用户主控通信
uint8_t comm_mode_flag = COMM_MODE_UART;
uint8_t reg_buf[6] = { 0xff, 10, 0 , 0 };

void dfr0715_multinet_init(void)
{
    // 从模型列表中筛选出与中文多关键词网络相关的模型名称。
    // char* mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    char* mn_name = "mn6_cn";
    if (MODEL_TYPE_EN == reg_buf[MODEL_TYPE_REG]) {
        mn_name = "mn6_en";
    }

    // 使用选定的模型名称创建多关键词网络接口 esp_mn_iface_t。
    multinet = esp_mn_handle_from_name(mn_name);

    // 通过 multinet->create 创建多关键词网络模型实例，并设置唤醒持续时间  ms
    if (model_data) {   // 防止之前创建的实例未销毁
        multinet->destroy(model_data);
        model_data = NULL;
    }
    if (6 >= reg_buf[WAKEUP_TIME_REG]) {   // 6s 下限
        model_data = multinet->create(mn_name, 6000);
    } else if (120 <= reg_buf[WAKEUP_TIME_REG]) {   // 目前测试最大支持 134s
        model_data = multinet->create(mn_name, 120000);
    } else {
        model_data = multinet->create(mn_name, reg_buf[WAKEUP_TIME_REG] * 1000);
    }

    // Add speech commands from sdkconfig
    // esp_mn_commands_update_from_sdkconfig(multinet, model_data);
    esp_mn_commands_alloc(multinet, model_data);
}

void comm_reply(uint8_t* buf, uint8_t size)
{
    if (COMM_MODE_UART == comm_mode_flag) {
        uart_write_bytes(UART_PORT_NUM, buf, size);
        uart_wait_tx_done(UART_PORT_NUM, 10);   // 确保 uart 发送完成
    } else {
        // i2c_reset_tx_fifo(I2C_SLAVE_NUM);
        uint8_t temp_buf[10] = { 0xFF };
        strlcat((char*)temp_buf, (char*)buf, sizeof(temp_buf));   // 无效第一个可能是错误数据的字节
        i2c_slave_write_buffer(I2C_SLAVE_NUM, temp_buf, size + 1, 10 / portTICK_PERIOD_MS);
    }
}

void long_command_handle(uint8_t id, uint8_t* data)
{
    if (longComID != id) {   // 如果和 上一次 不是 同一个id
        longComID = id;
        memset(handLongpStr, 0, strlen(handLongpStr));
    }
    // printf("strlen((char *)(data + 1)) = %u\n", strlen((char*)(data + 1)));
    strlcat(handLongpStr, (char*)(data + 1), sizeof(handLongpStr));   // 拼接字符串
    // printf("strlen(handLongpStr) = %u\n", strlen(handLongpStr));
    if (strlen(handLongpStr) == data[0]) {   // 命令词"完整" (长度正确)
        // reg_buf[CMD_ERROR_REG] = ESP_OK;
        if (0xFF == id) {   // 删除命令词
            reg_buf[CMD_ERROR_REG] = esp_mn_commands_remove(handLongpStr);  // remove a command
        } else if (0 != id) {   //添加命令词
            reg_buf[CMD_ERROR_REG] = esp_mn_commands_add(id, handLongpStr);  // add a command
        }
        memset(handLongpStr, 0, strlen(handLongpStr));
        longComID = 0;
    } else if (strlen(handLongpStr) >= 127) {
        reg_buf[CMD_ERROR_REG] = longComID;
        memset(handLongpStr, 0, strlen(handLongpStr));   // 放弃
        longComID = 0;
    } else {   // 命令词没有完成添加
        reg_buf[CMD_ERROR_REG] = longComID;
    }
}

void get_feed_data(bool is_get_raw_channel, int16_t* buffer, int buffer_len)
{
    // size_t bytes_read;
    int audio_chunksize = buffer_len / (sizeof(int16_t) * ADC_I2S_CHANNEL);

    ESP_ERROR_CHECK(esp_codec_dev_read(record_dev, (void*)buffer, buffer_len));
    if (!is_get_raw_channel) {
        for (int i = 0; i < audio_chunksize; i++) {
            int16_t ref = buffer[4 * i + 0];
            buffer[3 * i + 0] = buffer[4 * i + 1];
            buffer[3 * i + 1] = buffer[4 * i + 3];
            buffer[3 * i + 2] = ref;
        }
    }
}
