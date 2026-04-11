//在本文件中包含infrared.h中的函数没有使用，仅作为上位机开发的参考
//红外模块头文件
//具有解析can数据帧、ack确认、crc校验、超时重传等功能
//其中我使用的模块不具有高速发送的功能或者是高速接收的功能，总之在两个相同的模块之间通信时，
//                     没有完成高速通信（即设置红外时序的一系列时间不能过低）
#ifndef __INFRARED_H
#define __INFRARED_H

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdbool.h>

// 红外时序定义 (NEC标准的一半 - 高速模式)
#define IR_FREQUENCY    38000
#define START_PULSE_LEN 4500  // 起始信号：4.5ms高电平 (NEC标准9ms的一半)
#define START_SPACE_LEN 2250  // 起始信号：2.25ms低电平 (NEC标准4.5ms的一半)
#define BIT_ONE_HIGH    280   // 1的高电平：280us (NEC标准560us的一半)
#define BIT_ONE_LOW     840   // 1的低电平：840us (NEC标准1680us的一半)
#define BIT_ZERO_HIGH   280   // 0的高电平：280us (NEC标准560us的一半)
#define BIT_ZERO_LOW    280   // 0的低电平：280us (NEC标准560us的一半)

// 超时和容错定义
#define IR_RX_TIMEOUT_MS        50      // 接收超时时间（毫秒）
#define IR_TX_FRAME_INTERVAL_MS 30      // 帧间隔（毫秒）
#define IR_PULSE_TOLERANCE_US   150     // 脉冲宽度容差（微秒）
#define IR_MAX_PULSE_US         1500    // 最大有效脉冲宽度（微秒）
#define IR_MAX_RETRY_COUNT      3       // 最大重传次数

// ACK机制定义
#define IR_ACK_TIMEOUT_MS       100     // 等待ACK超时时间（毫秒）
#define IR_ACK_MAGIC            0xA5    // ACK帧魔术字节
#define IR_NACK_MAGIC           0x5A    // NACK帧魔术字节

// 模块ID定义（每个模块需要设置不同的ID，用于上位机区分）
#define IR_MODULE_ID            0x01   // 模块ID (1-255)

// 引脚定义
#define IR_RX_GPIO_PORT     GPIOA
#define IR_RX_GPIO_PIN      GPIO_PIN_15
#define IR_TX_GPIO_PORT     GPIOA
#define IR_TX_GPIO_PIN      GPIO_PIN_8

// 输入捕获通道
#define IC_CHANNEL          TIM_CHANNEL_1

extern uint8_t received_data[9]; // 接收数据缓冲区（供外部访问）
extern uint32_t rx_last_activity_time; // 上次接收活动时间
extern volatile uint8_t ir_rx_complete_flag; // 红外接收完成标志
extern volatile uint8_t ir_ack_received_flag; // ACK接收标志
extern volatile uint8_t ir_ack_status; // ACK状态: 0=无, 1=ACK, 2=NACK

// 发送状态机状态定义
typedef enum {
    IR_TX_IDLE = 0,         // 空闲状态
    IR_TX_START_PULSE,      // 发送起始脉冲(高电平)
    IR_TX_START_SPACE,      // 发送起始间隔(低电平)
    IR_TX_DATA_HIGH,        // 发送数据位高电平
    IR_TX_DATA_LOW,         // 发送数据位低电平
    IR_TX_STOP_PULSE,       // 发送结束脉冲(高电平)
    IR_TX_STOP,             // 发送结束
} IR_TX_State_t;

// 发送上下文结构体
typedef struct {
    IR_TX_State_t state;        // 当前状态
    uint8_t frame[9];           // 数据帧缓冲区 (8字节数据 + 1字节CRC)
    uint8_t byte_index;         // 当前发送的字节索引
    uint8_t bit_index;          // 当前发送的位索引 (0-7, 7是高位)
    uint8_t total_bytes;        // 总字节数
    bool busy;                  // 发送忙标志
    uint32_t last_tx_time;      // 上次发送完成时间
} IR_TX_Context_t;

void IR_Init(void);
bool IR_SendData(uint8_t *data, uint8_t length);
bool IR_SendDataWithRetry(uint8_t *data, uint8_t length, uint8_t max_retry);
bool IR_SendDataAndWaitAck(uint8_t *data, uint8_t length, uint8_t max_retry);
void IR_SendAck(uint8_t status);
void IR_TX_TimerCallback(TIM_HandleTypeDef *htim);
bool IR_IsTXBusy(void);
void IR_ReceiveData(void);
uint8_t IR_CRC8(uint8_t *data, uint8_t length);
void IR_ResetBuffer(void);
void IR_CheckRxTimeout(void);
void IR_ProcessReceivedFrame(uint8_t *data, uint8_t length);

#endif
