/*!
 * @file  main.h
 * @brief  用于DFR0715的固件
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2023-12-22
 * @url  https://github.com/DFRobot-Embedded-Software/DFR0715_FW
 */
#pragma once

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/i2s_std.h"

// #include "es7243e.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_codec_dev_os.h"

#include "esp_mn_speech_commands.h"
#include "esp_process_sdkconfig.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "model_path.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------- 定义和静态变量 ------------------ */
#define DFR0715_FW_TAG          "DFR0715 FW"

#define COMM_MODE_I2C    1   // i2c 模式对应的 拨码开关io口值
#define COMM_MODE_UART   0   // uart 模式对应的 拨码开关io口值

#define ADC_I2S_CHANNEL   4

#define CMD_READ_REGBUF    0xBB
#define CMD_WRITE_REGBUF   0xCC

#define MODEL_TYPE_STDBY   0x00   // 模型待机状态
#define MODEL_TYPE_CN      0x01   // 启用中文模型
#define MODEL_TYPE_EN      0x02   // 启用英文模型

#define VOICE_ID_REG         0x00   // 识别到的语音ID
#define WAKEUP_TIME_REG      0x01   // 唤醒持续时长 (单位: s)
#define MODEL_TYPE_REG       0x02   // 语言模型种类 (cn or en)
#define ADD_CMD_REG          0x03   // 添加关键词 (编号, 词条)
#define DEL_CMD_BY_ID_REG    0x04   // 删除关键词 (通过编号)
#define DEL_CMD_BY_STR_REG   0x05   // 删除关键词 (通过词条)

#define CMD_ERROR_REG        0x03   // 错误寄存器 0 为没有错误
#define REG_ISR_PID          0xAAU
#define REG_ISR_VID          0xACU
#define REG_ISR_VERSION      0xAEU

#define MODULE_DFR0715_PID       0x42CB   ///< Sensor PID
#define MODULE_DFR0715_VID       0x3343   ///< Sensor VID
#define MODULE_DFR0715_VERSION   0x0100   ///< Sensor VERSION

extern esp_mn_iface_t* multinet;
extern model_iface_data_t* model_data;


#ifdef __cplusplus
}
#endif
