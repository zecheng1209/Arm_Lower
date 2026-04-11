#ifndef __INFRARED_HOST_H
#define __INFRARED_HOST_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>
#include "cmsis_os.h"

#define IR_HOST_CAN_TIMEOUT_MS     500
#define IR_HOST_MAX_RETRY_COUNT     3
#define IR_HOST_FRAME_INTERVAL_MS   50

#define IR_ACK_MAGIC                0xA5
#define IR_NACK_MAGIC               0x5A

#define IR_HOST_CAN_ID_COMMAND      0x101
#define IR_HOST_CAN_ID_DATA         0x102
#define IR_HOST_CAN_ID_ACK          0x103

#define IR_HOST_RX_QUEUE_SIZE       20
#define IR_HOST_MAX_MODULES         8

typedef enum {
    IR_HOST_CMD_PING = 0x01,
    IR_HOST_CMD_SEND_DATA = 0x02,
    IR_HOST_CMD_READ_STATUS = 0x03,
    IR_HOST_CMD_RESET = 0x04
} IR_Host_Command_t;

typedef enum {
    IR_HOST_STATUS_IDLE = 0x00,
    IR_HOST_STATUS_SENDING = 0x01,
    IR_HOST_STATUS_WAIT_ACK = 0x02,
    IR_HOST_STATUS_SUCCESS = 0x03,
    IR_HOST_STATUS_TIMEOUT = 0x04,
    IR_HOST_STATUS_NACK = 0x05,
    IR_HOST_STATUS_ERROR = 0x06
} IR_Host_Status_t;

typedef enum {
    IR_HOST_FRAME_TYPE_COMMAND = 0x01,
    IR_HOST_FRAME_TYPE_DATA = 0x02,
    IR_HOST_FRAME_TYPE_ACK = 0x03,
    IR_HOST_FRAME_TYPE_UNKNOWN = 0xFF
} IR_Host_FrameType_t;

typedef struct {
    uint32_t can_id;
    uint8_t module_id;
    uint8_t data[8];
    uint8_t dlc;
    uint32_t timestamp;
} IR_Host_RxFrame_t;

typedef struct {
    uint8_t module_id;
    IR_Host_Status_t status;
    uint8_t data[8];
    uint8_t length;
    uint32_t timestamp;
    bool valid;
} IR_Host_ResponseFrame_t;

struct IR_Module_Node;

typedef struct IR_Module_Node {
    uint8_t module_id;
    IR_Host_Status_t status;
    IR_Host_ResponseFrame_t last_response;
    uint32_t last_rx_time;
    uint32_t last_tx_time;
    bool online;
    bool busy;
    struct IR_Module_Node *next;
} IR_Module_Node_t;

typedef struct {
    IR_Module_Node_t *head;
    uint8_t count;
    osMutexId_t mutex;
} IR_Module_List_t;

typedef struct {
    osMessageQueueId_t rx_queue;
    IR_Module_List_t module_list;
    bool initialized;
} IR_Host_Context_t;

extern IR_Host_Context_t ir_host_context;

void IR_Host_Init(void);
void IR_Host_StartTask(void);

bool IR_Host_AddModule(uint8_t module_id);
bool IR_Host_RemoveModule(uint8_t module_id);
IR_Module_Node_t* IR_Host_FindModule(uint8_t module_id);
IR_Module_Node_t* IR_Host_GetModuleByIndex(uint8_t index);
uint8_t IR_Host_GetModuleCount(void);
bool IR_Host_IsModuleOnline(uint8_t module_id);

bool IR_Host_SendCommand(uint8_t module_id, IR_Host_Command_t cmd, uint8_t *data, uint8_t length);
bool IR_Host_SendDataWithRetry(uint8_t module_id, uint8_t *data, uint8_t length, uint8_t max_retry);
bool IR_Host_Ping(uint8_t module_id, uint32_t timeout_ms);
bool IR_Host_ReadStatus(uint8_t module_id, uint8_t *status, uint32_t timeout_ms);
bool IR_Host_ResetModule(uint8_t module_id, uint32_t timeout_ms);

void IR_Host_Receive_DataFrame_Ocan(uint32_t can_id, uint8_t module_id, uint8_t *data, uint8_t dlc);
void IR_Host_Handle(uint8_t module_id, CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t dlc);
void IR_Host_ProcessRxFrame(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);
void IR_Host_Task(void *argument);

uint8_t IR_Host_CRC8(uint8_t *data, uint8_t length);

void IR_Host_ConfigCanFilter(void);
void IR_Host_StartCan(void);
void IR_Host_TxMailboxCompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif