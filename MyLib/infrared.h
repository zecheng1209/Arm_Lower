#ifndef __INFRARED_H
#define __INFRARED_H

#include "stm32f4xx_hal.h"

// 红外时序定义
#define IR_FREQUENCY   38000
#define START_PULSE_LEN 9000  // 起始信号：9ms高电平
#define START_SPACE_LEN 4500  // 起始信号：4.5ms低电平
#define BIT_ONE_HIGH    560   // 1的高电平：560us
#define BIT_ONE_LOW     560   // 1的低电平：560us
#define BIT_ZERO_HIGH   560   // 0的高电平：560us
#define BIT_ZERO_LOW    1680  // 0的低电平：1680us

// 输入捕获的相关配置
#define IC_CHANNEL       TIM_CHANNEL_2

void IR_Init(void);
void IR_SendData(uint8_t *data, uint8_t length);
void IR_GeneratePulse(uint16_t high_time, uint16_t low_time);
void IR_ReceiveData(void);
uint8_t IR_CRC8(uint8_t *data, uint8_t length);
void IR_ResetBuffer(void);

#endif