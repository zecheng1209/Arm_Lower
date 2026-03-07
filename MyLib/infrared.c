#include "infrared.h"
#include "stm32f4xx_hal.h"

// 定时器句柄，假设已经通过CubeMX配置好TIM1
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

// 接收的数据缓冲区（9 字节：8 字节数据 + 1 字节 CRC）
uint8_t received_data[9];
uint8_t bit_index = 0;  // 当前接收的 bit 序号（0-71）
uint32_t last_capture_time = 0;  // 用于计算时间差
uint8_t receiving = 0;  // 接收标志位

// 初始化红外发射端
void IR_Init(void)
{
    // 启动PWM信号
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    
    // 配置输入捕获，用于接收信号
    HAL_TIM_IC_Start_IT(&htim2, IC_CHANNEL);
}

// 发送数据（带CRC校验）
void IR_SendData(uint8_t *data, uint8_t length)
{
    uint8_t crc = IR_CRC8(data, length); // 计算CRC校验

    // 发送起始信号
    IR_GeneratePulse(START_PULSE_LEN, START_SPACE_LEN);
    
    // 发送数据（8位数据+CRC）
    for (int i = 0; i < length; i++) {
        for (int j = 7; j >= 0; j--) {
            if ((data[i] >> j) & 0x01) {
                IR_GeneratePulse(BIT_ONE_HIGH, BIT_ONE_LOW);  // 发送1
            } else {
                IR_GeneratePulse(BIT_ZERO_HIGH, BIT_ZERO_LOW);  // 发送0
            }
        }
    }

    // 发送CRC（1字节）
    for (int i = 7; i >= 0; i--) {
        if ((crc >> i) & 0x01) {
            IR_GeneratePulse(BIT_ONE_HIGH, BIT_ONE_LOW);  // 发送1
        } else {
            IR_GeneratePulse(BIT_ZERO_HIGH, BIT_ZERO_LOW);  // 发送0
        }
    }
}

// 生成脉冲
void IR_GeneratePulse(uint16_t high_time, uint16_t low_time)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // 设置高电平
    __HAL_TIM_SET_COUNTER(&htim1, 0);  // 复位定时器计数器
    HAL_TIM_Base_Start(&htim1); // 启动定时器
    while (__HAL_TIM_GET_COUNTER(&htim1) < high_time); // 等待高电平时长
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // 设置低电平
    __HAL_TIM_SET_COUNTER(&htim1, 0);  // 复位计数器，重新开始计时
    while (__HAL_TIM_GET_COUNTER(&htim1) < low_time); // 等待低电平时长
}

// CRC8计算
uint8_t IR_CRC8(uint8_t *data, uint8_t length)
{
    uint8_t crc = 0xFF;  // 初始CRC值
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;  // CRC-8-CCITT
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// 输入捕获回调（用于接收数据）
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        uint32_t capture_value = HAL_TIM_ReadCapturedValue(htim, IC_CHANNEL);
        
        if (last_capture_time != 0) {
            // 处理定时器溢出
            uint32_t pulse_duration;
            if (capture_value < last_capture_time) {
                // 定时器溢出（计数器从最大值回到 0）
                pulse_duration = (0xFFFFFFFF - last_capture_time) + capture_value;
            } else {
                pulse_duration = capture_value - last_capture_time;
            }
            
            // 检测起始信号（9ms 高 +4.5ms 低 = 13.5ms）
            if (pulse_duration > 10000) {
                receiving = 1;
                bit_index = 0;
                for (int i = 0; i < 9; i++) {
                    received_data[i] = 0;
                }
                last_capture_time = capture_value;
                return;
            }
            
            if (receiving == 0) {
                last_capture_time = capture_value;
                return;
            }
            
            // 解析 bit（根据 NEC 协议：0=560us 高 +1680us 低=2240us 周期，1=560us 高 +560us 低=1120us 周期）
            uint8_t bit_value;
            if (pulse_duration >= 1500 && pulse_duration < 2500) {
                bit_value = 0;  // 0：低电平时间长
            } else if (pulse_duration >= 500 && pulse_duration < 1200) {
                bit_value = 1;  // 1：低电平时间短
            } else {
                last_capture_time = capture_value;
                return;  // 无效脉冲
            }
            
            // 将 bit 组装成字节
            if (bit_index < 72) {  // 8 字节数据 +1 字节 CRC = 72 bits
                uint8_t byte_index = bit_index / 8;
                uint8_t bit_pos = 7 - (bit_index % 8);
                received_data[byte_index] |= (bit_value << bit_pos);
                bit_index++;
                
                // 接收完成 72 个 bit（9 字节）
                if (bit_index >= 72) {
                    receiving = 0;
                    IR_ReceiveData();
                }
            }
        }

        last_capture_time = capture_value;  // 更新上一次捕获时间
    }
}

// 数据接收处理（根据捕获时间解析数据）
void IR_ReceiveData(void)
{
    // 8 字节数据 +1 字节 CRC 已接收完成
    uint8_t crc_received = received_data[8];
    uint8_t crc_calculated = IR_CRC8(received_data, 8);
    
    // 比对 CRC，若匹配则认为接收成功
    if (crc_received == crc_calculated) {
        // 数据校验成功
        // 在此处处理接收到的 8 字节 CAN 数据
        // received_data[0] 到 received_data[7] 是 CAN 数据
    } else {
        // CRC 错误，丢弃数据或进行重传
    }
}

// 重置接收数据缓冲区
void IR_ResetBuffer(void)
{
    for (int i = 0; i < 9; i++) {
        received_data[i] = 0;
    }
    bit_index = 0;
    receiving = 0;
}