/*!
 * @file  main.c
 * @brief  用于DFR0715的固件
 * @details
 * @n    1.上电检测拨码开关，通过拨码开关的电平确定是I2C还是UART（默认I2C）
 * @n    2.当按下按键或识别到唤醒词时，返回0，LED灯点亮
 * @n    3.当退出唤醒时，LED灯熄灭
 * @n    4.识别到词条时，返回对应编号
 * @n    5.支持中英文识别
 * @n
 * @n    添加关键词(词条, 编号)
 * @n    删除关键词（编号or ALL）
 * @n    获取识别结果（ID）
 * @n    设置退出唤醒时间（0为不退出）
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2023-12-22
 * @url  https://github.com/DFRobot-Embedded-Software/DFR0715_FW
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "main.h"
#include "hardDevInit.h"

#include "esp_mn_speech_commands.h"
#include "esp_process_sdkconfig.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "model_path.h"

static volatile uint8_t key_flag = 0;
static uint8_t comm_mode_flag = COMM_MODE_UART;

uint8_t reg_buf[6] = { 0xff, 5, 0 , 0 };

srmodel_list_t* models = NULL;
static esp_afe_sr_iface_t* afe_handle = NULL;
static esp_afe_sr_data_t* afe_data = NULL;

esp_mn_iface_t* multinet;
model_iface_data_t* model_data;

int detect_flag = 0;
static volatile int model_task_flag = 0;

uint8_t addTempID = 0;
uint8_t delTempID = 0;
char addTempStr[128] = { 0 };
char delTempStr[128] = { 0 };
uint8_t longComID = 0;
char handLongpStr[128] = { 0 };

static void detect_Task(void* arg);
static void feed_Task(void* arg);

void dfr0715_multinet_init(void)
{
    // 从模型列表中筛选出与中文多关键词网络相关的模型名称。
    // char* mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_ENGLISH);
    // printf("multinet:%s\n", mn_name);
    // char* mn_name = "mn5q8_cn";
    char* mn_name = "mn6_cn";
    if (MODEL_TYPE_EN == reg_buf[MODEL_TYPE_REG]) {
        // mn_name = "mn5q8_en";
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
    // esp_mn_commands_clear();                       // Clear commands that already exist 
    // esp_mn_commands_add(1, "da kai deng guang");   // add a command
    // esp_mn_commands_add(1, "turn on the light");   // add a command
    // esp_mn_commands_add(2, "turn off the light");  // add a command
    // esp_mn_commands_update();                      // update commands

    //print active speech commands
    // esp_mn_commands_print();
    // multinet->print_active_speech_commands(model_data);
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

void long_command_handle(uint8_t id, uint8_t* data)   // 长命令词处理
{
    if (longComID != id) {   // 如果和 上一次 不是 同一个id
        longComID = id;
        memset(handLongpStr, 0, strlen(handLongpStr));
    }
    // printf("strlen((char *)(data + 1)) = %u\n", strlen((char*)(data + 1)));
    strlcat(handLongpStr, (char*)(data + 1), sizeof(handLongpStr));   // 拼接字符串
    // printf("strlen(handLongpStr) = %u\n", strlen(handLongpStr));
    if (strlen(handLongpStr) == data[0]) {   // 命令词"完整" (长度正确)
        reg_buf[CMD_ERROR_REG] = ESP_OK;
        if (0xFF == id) {   // 删除命令词
            esp_mn_commands_remove(handLongpStr);  // remove a command
        } else if (0 != id) {   //添加命令词
            esp_mn_commands_add(id, handLongpStr);  // add a command
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

/**
 * @brief 和主机通信数据的解析
 * @details 和主机通信数据分为如下读写命令两类:
 * @n 写操作 :
 * @n 发送 : CMD_WRITE_REGBUF + 寄存器地址 + 写入数据长度 + 对应长度的数据字节
 * @n 读操作 :
 * @n 发送 : CMD_READ_REGBUF + 寄存器地址 + 读取数据长度 ; 接收 : 读取长度的字节
 */
void annysis_command(uint8_t* data, size_t size)
{
    uint8_t type = data[0];
    uint8_t reg = data[1];
    uint8_t len = data[2];
    switch (type) {
    case CMD_READ_REGBUF: {
        if ((reg + len) <= CMD_ERROR_REG + 1) {
            // printf("reg_buf[%u] = %u; len = %u\n", reg, reg_buf[reg], len);
            comm_reply(&reg_buf[reg], len);
            if (VOICE_ID_REG == reg) {
                reg_buf[VOICE_ID_REG] = 0xFF;   // 读取后重置 语音识别ID的寄存器
            }
            if (CMD_ERROR_REG == reg) {
                // reg_buf[CMD_ERROR_REG] = 0;   // 读取后重置
                longComID = 0;
            }
        } else {
            uint8_t buf[2] = { 0 };
            buf[0] = MODULE_DFR0715_PID & 0xFF;
            buf[1] = (MODULE_DFR0715_PID >> 8) & 0xFF;
            comm_reply(buf, 2);
        }
        break;
    }
    case CMD_WRITE_REGBUF:
        if ((reg != 0x00) && ((reg + len) <= MODEL_TYPE_REG + 1)) {   // 防止uart数据不完整
            memcpy(&reg_buf[reg], &data[3], len);
            reg_buf[VOICE_ID_REG] = 0xFF;   // 退出后重置 语音识别ID的寄存器
            afe_handle->enable_wakenet(afe_data);
            detect_flag = 0;
            gpio_set_level(GPIO_LED, 0);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            model_task_flag = 0;   // 退出之前初始化模型的语音检测任务
            vTaskDelay(100 / portTICK_PERIOD_MS);
            dfr0715_multinet_init();
            model_task_flag = 1;
            xTaskCreatePinnedToCore(&feed_Task, "feed", 8 * 1024, (void*)afe_data, 5, NULL, 0);
            xTaskCreatePinnedToCore(&detect_Task, "detect", 16 * 1024, (void*)afe_data, 5, NULL, 1);

        } else if (ADD_CMD_REG == reg) {
            // printf("size = %u, len = %u\n", size, len);
            printf("------------ADD_CMD_REG------------");
            printf("id = %u; length = %u; str = %s\n", data[3], data[4], (char*)(&data[5]));
            // 超长命令词
            // if (addTempID != data[3]) {   // 如果不是上个id
            //     addTempID = data[3];
            //     memset(addTempStr, 0, strlen(addTempStr));
            // }
            // // printf("strlen((char *)(&data[5])) = %u\n", strlen((char*)(&data[5])));
            // strlcat(addTempStr, (char*)(&data[5]), sizeof(addTempStr));
            // // printf("strlen(addTempStr) = %u\n", strlen(addTempStr));
            // if (strlen(addTempStr) == data[4]) {   // 命令词完整
            //     reg_buf[CMD_ERROR_REG] = ESP_OK;
            //     esp_mn_commands_add(data[3], addTempStr);  // add a command
            //     memset(addTempStr, 0, strlen(addTempStr));   // 如果成功添加
            //     addTempID = 0;
            // } else if (strlen(addTempStr) > 127) {
            //     memset(addTempStr, 0, strlen(addTempStr));   // 放弃
            //     reg_buf[CMD_ERROR_REG] = 0xFF;
            //     addTempID = 0;
            // } else {   // 命令词没有完成添加
            //     reg_buf[CMD_ERROR_REG] = ADD_CMD_REG;
            // }
            long_command_handle(data[3], data + 4);
        } else if (DEL_CMD_BY_ID_REG == reg) {
            char* p = esp_mn_commands_get_string(data[3]);
            if (NULL != p) {
                esp_mn_commands_remove(p);
            }
        } else if (DEL_CMD_BY_STR_REG == reg) {
            printf("------------DEL_CMD_BY_STR_REG------------");
            printf("length = %u; str = %s\n", data[3], (char*)(&data[4]));
            // strlcat(delTempStr, (char*)(&data[4]), sizeof(delTempStr));
            // if (strlen(delTempStr) == data[3]) {   // 命令词完整
            //     reg_buf[CMD_ERROR_REG] = ESP_OK;
            //     esp_mn_commands_remove(delTempStr);  // remove a command
            //     memset(delTempStr, 0, strlen(delTempStr));   // 如果成功清除
            // } else if (strlen(delTempStr) > 127) {
            //     memset(delTempStr, 0, strlen(delTempStr));   // 放弃
            //     reg_buf[CMD_ERROR_REG] = 0xFF;
            // } else {   // 命令词没有完成删除
            //     reg_buf[CMD_ERROR_REG] = DEL_CMD_BY_STR_REG;
            // }
            long_command_handle(0xFF, data + 3);
            // if (ESP_OK != esp_mn_commands_remove((char*)(&data[3]))) {   // 是长命令词
            //     strcat(delTempStr, (char*)(&data[3]));
            //     if (ESP_OK == esp_mn_commands_remove(delTempStr)) {
            //         memset(delTempStr, 0, strlen(delTempStr));   // 如果成功清除
            //     }
            // }
        }
        esp_mn_commands_update();                      // update commands
        // DBG
        // esp_mn_commands_print();   // 在列表中的
        esp_mn_active_commands_print();   // 在模型中的
        break;
    default:
        break;
    }
    memset(data, 0, 64);   // 清除接收的信息

}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    if (GPIO_KEY == gpio_num) {
        key_flag = 1;
    }
}

static void gpio_task(void* arg)
{
    // gpio初始化
    dfr0715_gpio_init();
    ESP_ERROR_CHECK(gpio_isr_handler_add(GPIO_KEY, gpio_isr_handler, (void*)GPIO_KEY));

    gpio_set_level(GPIO_LED, 0);   // 避免复位时, 灯光不能自动熄灭

    for (;;) {
        if (key_flag) {
            // printf("[%d] gpio_task--------!\n", gpio_get_level(GPIO_KEY));
            if (model_task_flag) {
                multinet->clean(model_data);  // clean all status of multinet
                detect_flag = 1;
                gpio_set_level(GPIO_LED, 1);
                reg_buf[VOICE_ID_REG] = 0;   // 更新寄存器
                key_flag = 0;
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static void communication_task(void* arg)
{
    // int task_idx = (int)arg;
    static size_t r_size = 0;
    uint8_t* data_rd = (uint8_t*)malloc(64);
    // uint8_t* data_wr = (uint8_t*)malloc(64);

    // 用户通信接口初始化
    comm_mode_flag = gpio_get_level(GPIO_COMM_MODE);
    if (COMM_MODE_UART == comm_mode_flag) {
        // printf("COMM_MODE_UART\n");
        dfr0715_uart_init();
        uart_event_t event;
        while (true) {
            if (xQueueReceive(user_uart_queue, (void*)&event, 100 / portTICK_PERIOD_MS)) {
                switch (event.type) {
                case UART_DATA:
                    r_size = uart_read_bytes(UART_PORT_NUM, data_rd, event.size, 20 / portTICK_PERIOD_MS);
                    if (r_size > 0) {
                        annysis_command(data_rd, r_size);
                    }
                    break;
                default:
                    break;
                }
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    } else {
        // printf("COMM_MODE_I2C\n");
        dfr0715_i2c_slave_init();
        while (1) {
            size_t r_size = i2c_slave_read_buffer(I2C_SLAVE_NUM, data_rd, 32, 10 / portTICK_PERIOD_MS);
            // i2c_reset_rx_fifo(I2C_SLAVE_NUM);
            if (r_size > 0) {
                // if (data_rd[1] == 0xAA) {  // Assuming 0x00 is the register address for read operation
                //     i2c_reset_tx_fifo(I2C_SLAVE_NUM);
                //     data_wr[0] = data_rd[0];
                //     data_wr[1] = data_rd[1];
                //     i2c_slave_write_buffer(I2C_SLAVE_NUM, data_wr, 2, 100 / portTICK_PERIOD_MS);
                //     // printf("Sending register value: %02X\n", data_rd[0]);
                //     // printf("Sending register value: %02X\n", data_rd[1]);
                //     // size_t w_size = i2c_slave_write_buffer(I2C_SLAVE_NUM, data_rd, 64, 1000 / portTICK_PERIOD_MS);
                // } else {
                //     // printf("Invalid register address received\n");
                // }
                // i2c_reset_tx_fifo(I2C_SLAVE_NUM);
                annysis_command(data_rd, r_size);
            }

            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}

static void get_feed_data(bool is_get_raw_channel, int16_t* buffer, int buffer_len)
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

static void feed_Task(void* arg)
{
    esp_afe_sr_data_t* afe_data = arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);   // 获取音频块的大小
    int nch = afe_handle->get_channel_num(afe_data);   // 获取音频通道的数量
    // printf("nch:%d\n", nch);
    int feed_channel = ADC_I2S_CHANNEL;   // 获取用于喂养（feed）的音频通道数目
    assert(nch <= feed_channel);
    int16_t* i2s_buff = malloc(audio_chunksize * sizeof(int16_t) * feed_channel);
    assert(i2s_buff);

    while (model_task_flag) {
        // 获取音频数据并存储到分配的缓冲区中
        get_feed_data(false, i2s_buff, audio_chunksize * sizeof(int16_t) * feed_channel);
        //  将获取到的音频数据喂养给音频处理模块
        afe_handle->feed(afe_data, i2s_buff);
    }
    if (i2s_buff) {
        free(i2s_buff);
        i2s_buff = NULL;
    }
    vTaskDelete(NULL);
}

static void detect_Task(void* arg)
{
    // arg 被强制转换为 esp_afe_sr_data_t* 类型，表示这是一个 ESP32-S3 音频前端（Audio Front End，AFE）相关的数据结构
    esp_afe_sr_data_t* afe_data = arg;
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    int mu_chunksize = multinet->get_samp_chunksize(model_data);
    assert(mu_chunksize == afe_chunksize);

    // printf("------------detect start------------\n");
    while (model_task_flag) {
        afe_fetch_result_t* res = afe_handle->fetch(afe_data);
        if (!res || res->ret_value == ESP_FAIL) {
            printf("fetch error!\n");
            break;
        }
        if (res->wakeup_state == WAKENET_DETECTED) {
            printf("WAKEWORD DETECTED\n");
            multinet->clean(model_data);  // clean all status of multinet
        } else if (res->wakeup_state == WAKENET_CHANNEL_VERIFIED) {
            detect_flag = 1;
            reg_buf[VOICE_ID_REG] = 0;   // 更新寄存器
            gpio_set_level(GPIO_LED, 1);
            printf("AFE_FETCH_CHANNEL_VERIFIED, channel index: %d\n", res->trigger_channel_id);
        }

        if (detect_flag == 1) {
            // 检测是否有符合多关键词网络的 词
            printf("1\n");
            esp_mn_state_t mn_state = multinet->detect(model_data, res->data);
            printf("2\n");

            if (mn_state == ESP_MN_STATE_DETECTING) {
                continue;
            }

            if (mn_state == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t* mn_result = multinet->get_results(model_data);
                reg_buf[VOICE_ID_REG] = mn_result->command_id[0];   // 更新寄存器
                for (int i = 0; i < mn_result->num; i++) {
                    printf("TOP %d, command_id: %d, phrase_id: %d, string:%s prob: %f\n",
                        i + 1, mn_result->command_id[i], mn_result->phrase_id[i], mn_result->string, mn_result->prob[i]);
                }
                // printf("\n-----------listening-----------\n");
            }

            if (mn_state == ESP_MN_STATE_TIMEOUT) {
                // esp_mn_results_t* mn_result = multinet->get_results(model_data);
                // printf("timeout, string:%s\n", mn_result->string);
                if (0 != reg_buf[WAKEUP_TIME_REG]) {   // 如果唤醒时长设置为零, 要求持续唤醒
                    reg_buf[VOICE_ID_REG] = 0xFF;   // 退出后重置 语音识别ID的寄存器
                    afe_handle->enable_wakenet(afe_data);
                    detect_flag = 0;
                    gpio_set_level(GPIO_LED, 0);
                    // printf("\n-----------awaits to be waken up-----------\n");
                }
                continue;
            }
        }
    }
    if (model_data) {
        multinet->destroy(model_data);
        model_data = NULL;
    }
    // printf("detect exit\n");
    vTaskDelete(NULL);
}

void app_main(void)
{
    // vTaskDelay(500 / portTICK_PERIOD_MS);   // 避免出错时过快的重启

    // 麦克风语音解码器初始化
    es7243e_codec_init();

    // 初始化语音识别模型列表，其中 "model" 是模型的文件夹路径。后续代码通过遍历模型列表来找到唤醒词模型。
    models = esp_srmodel_init("model");

    afe_handle = (esp_afe_sr_iface_t*)&ESP_AFE_SR_HANDLE;
    // 设置默认配置，包括内存分配模式、是否初始化唤醒词模型、唤醒词模型的名称等。
    afe_config_t afe_config = AFE_CONFIG_DEFAULT();
    afe_config.wakenet_model_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);;
    afe_config.memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;
    afe_config.wakenet_init = true;
    afe_config.voice_communication_init = false;
    afe_config.aec_init = true;
    afe_config.pcm_config.total_ch_num = 3;
    afe_config.pcm_config.mic_num = 2;
    afe_config.pcm_config.ref_num = 1;
    // 创建一个音频处理接口，并将返回的接口指针存储在 afe_data 变量中。
    afe_data = afe_handle->create_from_config(&afe_config);

    dfr0715_multinet_init();

    model_task_flag = 1;
    xTaskCreatePinnedToCore(&feed_Task, "feed", 8 * 1024, (void*)afe_data, 5, NULL, 0);
    xTaskCreatePinnedToCore(&detect_Task, "detect", 16 * 1024, (void*)afe_data, 5, NULL, 1);

    xTaskCreate(gpio_task, "gpio_key", 4096, NULL, 4, NULL);
    xTaskCreate(communication_task, "communication", 4096, NULL, 4, NULL);

    // int i = 0;
    // while (1) {
    //     i++;
    //     // printf("[%d] Hello world!\n", i);
    //     vTaskDelay(5000 / portTICK_PERIOD_MS);
    // }
}
