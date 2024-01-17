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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "main.h"
#include "hardDevInit.h"
#include "moduleFunc.h"

 // 语音模型相关句柄
srmodel_list_t* models = NULL;
static esp_afe_sr_iface_t* afe_handle = NULL;
static esp_afe_sr_data_t* afe_data = NULL;

esp_mn_iface_t* multinet;
model_iface_data_t* model_data;

// 按键标志
static volatile uint8_t key_flag = 0;

// 语音检测相关
static volatile int detect_flag = 0;
static volatile int model_task_flag = 0;

static void detect_Task(void* arg);
static void feed_Task(void* arg);

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

            long_command_handle(data[3], data + 4);
        } else if (DEL_CMD_BY_ID_REG == reg) {
            if (0 != data[3]) {
                char* p = esp_mn_commands_get_string(data[3]);
                if (NULL != p) {
                    esp_mn_commands_remove(p);
                }
            } else {   // 删除id为零时, 清空所有命令词
                esp_mn_commands_clear();
            }
        } else if (DEL_CMD_BY_STR_REG == reg) {
            printf("------------DEL_CMD_BY_STR_REG------------");
            printf("length = %u; str = %s\n", data[3], (char*)(&data[4]));
            long_command_handle(0xFF, data + 3);
        }
        if (0 == detect_flag) {   // 唤醒模式下不能更新命令词
            esp_mn_commands_update();                      // update commands
        }
        // DBG
        // esp_mn_commands_print();   // 在列表中的
        // esp_mn_active_commands_print();   // 在模型中的
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
                // i2c_reset_tx_fifo(I2C_SLAVE_NUM);
                annysis_command(data_rd, r_size);
            }

            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
    if (data_rd) {
        free(data_rd);
        data_rd = NULL;
    }
    vTaskDelete(NULL);
}

static void feed_Task(void* arg)
{
    esp_afe_sr_data_t* afe_data = arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);   // 获取音频块的大小
    int nch = afe_handle->get_channel_num(afe_data);   // 获取音频通道的数量
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
            esp_mn_state_t mn_state = multinet->detect(model_data, res->data);

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
                esp_mn_results_t* mn_result = multinet->get_results(model_data);
                printf("timeout, string:%s\n", mn_result->string);
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
