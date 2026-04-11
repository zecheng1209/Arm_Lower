#include "infrared_host.h"
#include "can.h"

extern CAN_HandleTypeDef hcan1;

IR_Host_Context_t ir_host_context = {0};

static osThreadId_t ir_host_task_handle = NULL;
static const osThreadAttr_t ir_host_task_attr = {
    .name = "IRHostTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void IR_Host_Init(void)
{
    ir_host_context.rx_queue = osMessageQueueNew(IR_HOST_RX_QUEUE_SIZE, sizeof(IR_Host_RxFrame_t), NULL);
    ir_host_context.module_list.head = NULL;
    ir_host_context.module_list.count = 0;
    ir_host_context.module_list.mutex = osMutexNew(NULL);
    ir_host_context.initialized = true;

    IR_Host_AddModule(0x01);

    IR_Module_Node_t *module = IR_Host_FindModule(0x01);
    if (module != NULL) {
        module->last_tx_time = HAL_GetTick() - IR_HOST_FRAME_INTERVAL_MS - 1;
    }
}

void IR_Host_StartTask(void)
{
    if (ir_host_task_handle == NULL) {
        ir_host_task_handle = osThreadNew(IR_Host_Task, NULL, &ir_host_task_attr);
    }
}

bool IR_Host_AddModule(uint8_t module_id)
{
    if (!ir_host_context.initialized) return false;
    if (ir_host_context.module_list.count >= IR_HOST_MAX_MODULES) return false;

    osAcquire(ir_host_context.module_list.mutex);

    IR_Module_Node_t *current = ir_host_context.module_list.head;
    while (current != NULL) {
        if (current->module_id == module_id) {
            osRelease(ir_host_context.module_list.mutex);
            return false;
        }
        current = current->next;
    }

    IR_Module_Node_t *new_node = (IR_Module_Node_t *)pvPortMalloc(sizeof(IR_Module_Node_t));
    if (new_node == NULL) {
        osRelease(ir_host_context.module_list.mutex);
        return false;
    }

    new_node->module_id = module_id;
    new_node->status = IR_HOST_STATUS_IDLE;
    new_node->online = false;
    new_node->busy = false;
    new_node->last_rx_time = 0;
    new_node->last_tx_time = 0;
    memset(&new_node->last_response, 0, sizeof(IR_Host_ResponseFrame_t));
    new_node->next = ir_host_context.module_list.head;
    ir_host_context.module_list.head = new_node;
    ir_host_context.module_list.count++;

    osRelease(ir_host_context.module_list.mutex);
    return true;
}

bool IR_Host_RemoveModule(uint8_t module_id)
{
    if (!ir_host_context.initialized) return false;

    osAcquire(ir_host_context.module_list.mutex);

    IR_Module_Node_t **current = &ir_host_context.module_list.head;
    while (*current != NULL) {
        if ((*current)->module_id == module_id) {
            IR_Module_Node_t *to_remove = *current;
            *current = (*current)->next;
            vPortFree(to_remove);
            ir_host_context.module_list.count--;
            osRelease(ir_host_context.module_list.mutex);
            return true;
        }
        current = &(*current)->next;
    }

    osRelease(ir_host_context.module_list.mutex);
    return false;
}

IR_Module_Node_t* IR_Host_FindModule(uint8_t module_id)
{
    if (!ir_host_context.initialized) return NULL;

    IR_Module_Node_t *current = ir_host_context.module_list.head;
    while (current != NULL) {
        if (current->module_id == module_id) {
            return current;
        }
        current = current->next;
    }
    return NULL;
}

IR_Module_Node_t* IR_Host_GetModuleByIndex(uint8_t index)
{
    if (!ir_host_context.initialized || index >= ir_host_context.module_list.count) return NULL;

    IR_Module_Node_t *current = ir_host_context.module_list.head;
    for (uint8_t i = 0; i < index && current != NULL; i++) {
        current = current->next;
    }
    return current;
}

uint8_t IR_Host_GetModuleCount(void)
{
    return ir_host_context.module_list.count;
}

bool IR_Host_IsModuleOnline(uint8_t module_id)
{
    IR_Module_Node_t *module = IR_Host_FindModule(module_id);
    if (module == NULL) return false;
    return module->online;
}

uint8_t IR_Host_CRC8(uint8_t *data, uint8_t length)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void IR_Host_ConfigCanFilter(void)
{
    CAN_FilterTypeDef can_filter;

    can_filter.FilterBank = 0;
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh = 0x0000;
    can_filter.FilterIdLow = 0x0000;
    can_filter.FilterMaskIdHigh = 0x0000;
    can_filter.FilterMaskIdLow = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.FilterActivation = ENABLE;
    can_filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK) {
        Error_Handler();
    }
}

void IR_Host_StartCan(void)
{
    IR_Host_ConfigCanFilter();

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY |
                                              CAN_IT_RX_FIFO0_MSG_PENDING |
                                              CAN_IT_RX_FIFO0_FULL |
                                              CAN_IT_RX_FIFO0_OVERRUN) != HAL_OK) {
        Error_Handler();
    }
}

void IR_Host_TxMailboxCompleteCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) {
        // Transmission complete, can be used for flow control if needed
    }
}

void IR_Host_Receive_DataFrame_Ocan(uint32_t can_id, uint8_t module_id, uint8_t *data, uint8_t dlc)
{
    IR_Host_RxFrame_t rx_frame;

    rx_frame.can_id = can_id;
    rx_frame.module_id = module_id;
    rx_frame.dlc = dlc > 8 ? 8 : dlc;
    memcpy(rx_frame.data, data, rx_frame.dlc);
    rx_frame.timestamp = HAL_GetTick();

    if (ir_host_context.rx_queue != NULL) {
        osMessageQueuePut(ir_host_context.rx_queue, &rx_frame, 0, 0);
    }
}

void IR_Host_Handle(uint8_t module_id, CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t dlc)
{
    if (id == IR_HOST_CAN_ID_ACK) {
        if (dlc >= 2 && data[0] == IR_ACK_MAGIC && data[1] == IR_ACK_MAGIC) {
            IR_Module_Node_t *module = IR_Host_FindModule(module_id);
            if (module != NULL) {
                module->status = IR_HOST_STATUS_SUCCESS;
                module->busy = false;
            }
        }
        return;
    }

    if (id == IR_HOST_CAN_ID_DATA) {
        IR_Module_Node_t *module = IR_Host_FindModule(module_id);

        if (dlc >= 2) {
            uint8_t received_crc = data[dlc - 1];
            uint8_t calculated_crc = IR_Host_CRC8(data, dlc - 1);

            if (received_crc == calculated_crc) {
                if (module != NULL) {
                    module->last_response.module_id = data[0];
                    module->last_response.status = IR_HOST_STATUS_SUCCESS;
                    module->last_response.length = dlc - 2;
                    module->last_response.timestamp = HAL_GetTick();
                    module->last_response.valid = true;
                    if (module->last_response.length > 0) {
                        memcpy(module->last_response.data, &data[1], module->last_response.length);
                    }
                    module->last_rx_time = HAL_GetTick();
                    module->online = true;
                    module->busy = false;
                }
            }
        }
    }
}

void IR_Host_ProcessRxFrame(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    uint8_t module_id = rx_data[0];

    IR_Host_Receive_DataFrame_Ocan(rx_header->StdId, module_id, rx_data, rx_header->DLC);
    IR_Host_Handle(module_id, &hcan1, rx_header->StdId, rx_data, rx_header->DLC);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) {
        uint8_t rx_data[8];
        CAN_RxHeaderTypeDef rx_header;

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
            IR_Host_ProcessRxFrame(&rx_header, rx_data);
        }
    }
}

bool IR_Host_SendCommand(uint8_t module_id, IR_Host_Command_t cmd, uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];
    uint8_t crc_len;

    IR_Module_Node_t *module = IR_Host_FindModule(module_id);
    if (module == NULL) return false;
    if (module->busy) return false;
    if (length > 5) return false;

    uint32_t time_since_last_tx = HAL_GetTick() - module->last_tx_time;
    if (time_since_last_tx < IR_HOST_FRAME_INTERVAL_MS) return false;

    memset(tx_data, 0, 8);
    tx_data[0] = module_id;
    tx_data[1] = cmd;
    if (data != NULL && length > 0) {
        memcpy(&tx_data[2], data, length);
    }
    crc_len = 2 + length;
    tx_data[crc_len] = IR_Host_CRC8(tx_data, crc_len);

    tx_header.StdId = IR_HOST_CAN_ID_COMMAND;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = crc_len + 1;
    tx_header.TransmitGlobalTime = DISABLE;

    module->busy = true;
    module->status = IR_HOST_STATUS_SENDING;

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
        module->busy = false;
        module->status = IR_HOST_STATUS_ERROR;
        return false;
    }

    module->last_tx_time = HAL_GetTick();
    module->status = IR_HOST_STATUS_WAIT_ACK;
    return true;
}

bool IR_Host_SendDataWithRetry(uint8_t module_id, uint8_t *data, uint8_t length, uint8_t max_retry)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];
    uint8_t total_len;

    IR_Module_Node_t *module = IR_Host_FindModule(module_id);
    if (module == NULL) return false;
    if (length > 6) return false;

    for (uint8_t retry = 0; retry < max_retry; retry++) {
        while (module->busy) {
            osDelay(10);
        }

        module->status = IR_HOST_STATUS_IDLE;

        memset(tx_data, 0, 8);
        tx_data[0] = module_id;
        memcpy(&tx_data[1], data, length);
        total_len = length + 1;
        tx_data[total_len] = IR_Host_CRC8(tx_data, total_len);
        total_len++;

        tx_header.StdId = IR_HOST_CAN_ID_DATA;
        tx_header.ExtId = 0;
        tx_header.IDE = CAN_ID_STD;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = total_len;
        tx_header.TransmitGlobalTime = DISABLE;

        module->busy = true;
        module->status = IR_HOST_STATUS_SENDING;

        if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) == HAL_OK) {
            module->last_tx_time = HAL_GetTick();

            uint32_t start_time = HAL_GetTick();
            while ((HAL_GetTick() - start_time) < IR_HOST_CAN_TIMEOUT_MS) {
                if (!module->busy) {
                    if (module->status == IR_HOST_STATUS_SUCCESS) {
                        return true;
                    } else if (module->status == IR_HOST_STATUS_NACK) {
                        break;
                    }
                }
                osDelay(10);
            }
        } else {
            module->busy = false;
            module->status = IR_HOST_STATUS_ERROR;
        }

        osDelay(50);
    }

    return false;
}

bool IR_Host_Ping(uint8_t module_id, uint32_t timeout_ms)
{
    if (!IR_Host_SendCommand(module_id, IR_HOST_CMD_PING, NULL, 0)) {
        return false;
    }

    uint32_t start_time = HAL_GetTick();
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        IR_Module_Node_t *module = IR_Host_FindModule(module_id);
        if (module != NULL && !module->busy && module->status == IR_HOST_STATUS_SUCCESS) {
            return true;
        }
        osDelay(10);
    }

    return false;
}

bool IR_Host_ReadStatus(uint8_t module_id, uint8_t *status, uint32_t timeout_ms)
{
    if (!IR_Host_SendCommand(module_id, IR_HOST_CMD_READ_STATUS, NULL, 0)) {
        return false;
    }

    uint32_t start_time = HAL_GetTick();
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        IR_Module_Node_t *module = IR_Host_FindModule(module_id);
        if (module != NULL && !module->busy) {
            if (status != NULL && module->last_response.length > 0) {
                *status = module->last_response.data[0];
            }
            return true;
        }
        osDelay(10);
    }

    return false;
}

bool IR_Host_ResetModule(uint8_t module_id, uint32_t timeout_ms)
{
    if (!IR_Host_SendCommand(module_id, IR_HOST_CMD_RESET, NULL, 0)) {
        return false;
    }

    uint32_t start_time = HAL_GetTick();
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        IR_Module_Node_t *module = IR_Host_FindModule(module_id);
        if (module != NULL && !module->busy) {
            return true;
        }
        osDelay(10);
    }

    return false;
}

void IR_Host_Task(void *argument)
{
    IR_Host_RxFrame_t rx_frame;
    (void)argument;

    for (;;) {
        if (osMessageQueueGet(ir_host_context.rx_queue, &rx_frame, NULL, 100) == osOK) {
            IR_Module_Node_t *module = IR_Host_FindModule(rx_frame.module_id);

            if (module != NULL) {
                if (rx_frame.can_id == IR_HOST_CAN_ID_ACK) {
                    if (rx_frame.dlc >= 2 &&
                        rx_frame.data[0] == IR_ACK_MAGIC &&
                        rx_frame.data[1] == IR_ACK_MAGIC) {
                        module->status = IR_HOST_STATUS_SUCCESS;
                        module->busy = false;
                    }
                }
                else if (rx_frame.can_id == IR_HOST_CAN_ID_DATA) {
                    if (rx_frame.dlc >= 2) {
                        uint8_t received_crc = rx_frame.data[rx_frame.dlc - 1];
                        uint8_t calculated_crc = IR_Host_CRC8(rx_frame.data, rx_frame.dlc - 1);

                        if (received_crc == calculated_crc) {
                            module->last_response.module_id = rx_frame.data[0];
                            module->last_response.status = IR_HOST_STATUS_SUCCESS;
                            module->last_response.length = rx_frame.dlc - 2;
                            module->last_response.timestamp = rx_frame.timestamp;
                            module->last_response.valid = true;

                            if (module->last_response.length > 0) {
                                memcpy(module->last_response.data,
                                       &rx_frame.data[1],
                                       module->last_response.length);
                            }

                            module->last_rx_time = rx_frame.timestamp;
                            module->online = true;
                            module->busy = false;
                        }
                    }
                }
            }
        }

        IR_Module_Node_t *current = ir_host_context.module_list.head;
        while (current != NULL) {
            if (current->online &&
                (HAL_GetTick() - current->last_rx_time) > 3000) {
                current->online = false;
            }
            current = current->next;
        }
    }
}