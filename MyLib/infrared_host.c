#include "infrared_host.h"
#include "can.h"

extern CAN_HandleTypeDef hcan1;

IR_Host_Context_t ir_host_context = {0};

static TaskHandle_t ir_host_task_handle = NULL;

void IR_Host_Init(void)
{
    ir_host_context.rx_queue = xQueueCreate(IR_HOST_RX_QUEUE_SIZE, sizeof(IR_Host_RxFrame_t));
    ir_host_context.module_list.head = NULL;
    ir_host_context.module_list.count = 0;
    ir_host_context.module_list.mutex = xSemaphoreCreateMutex();
    ir_host_context.initialized = true;
    ir_host_context.task_state = IR_HOST_TASK_STATE_INIT;
    ir_host_context.current_poll_index = 0;
    ir_host_context.last_poll_time = 0;
    ir_host_context.discovery_start_time = 0;

    IR_Host_AddModule(0x01);

    IR_Module_Node_t *module = IR_Host_FindModule(0x01);
    if (module != NULL) {
        module->last_tx_time = HAL_GetTick() - IR_HOST_FRAME_INTERVAL_MS - 1;
    }
}

void IR_Host_StartTask(void)
{
    if (ir_host_task_handle == NULL) {
        xTaskCreate(IR_Host_Task, "IRHostTask", 1024, NULL, 3, &ir_host_task_handle);
    }
}

bool IR_Host_AddModule(uint8_t module_id)
{
    if (!ir_host_context.initialized) return false;
    if (ir_host_context.module_list.count >= IR_HOST_MAX_MODULES) return false;

    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);

    IR_Module_Node_t *current = ir_host_context.module_list.head;
    while (current != NULL) {
        if (current->module_id == module_id) {
            xSemaphoreGive(ir_host_context.module_list.mutex);
            return false;
        }
        current = current->next;
    }

    IR_Module_Node_t *new_node = (IR_Module_Node_t *)pvPortMalloc(sizeof(IR_Module_Node_t));
    if (new_node == NULL) {
        xSemaphoreGive(ir_host_context.module_list.mutex);
        return false;
    }

    new_node->module_id = module_id;
    new_node->status = IR_HOST_STATUS_IDLE;
    new_node->online = false;
    new_node->busy = false;
    new_node->discovered = false;
    new_node->last_rx_time = 0;
    new_node->last_tx_time = 0;
    new_node->poll_fail_count = 0;
    new_node->consecutive_errors = 0;
    memset(&new_node->last_response, 0, sizeof(IR_Host_ResponseFrame_t));
    memset(&new_node->data_cache, 0, sizeof(IR_Module_DataCache_t));
    new_node->next = ir_host_context.module_list.head;
    ir_host_context.module_list.head = new_node;
    ir_host_context.module_list.count++;

    xSemaphoreGive(ir_host_context.module_list.mutex);
    return true;
}

bool IR_Host_RemoveModule(uint8_t module_id)
{
    if (!ir_host_context.initialized) return false;

    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);

    IR_Module_Node_t **current = &ir_host_context.module_list.head;
    while (*current != NULL) {
        if ((*current)->module_id == module_id) {
            IR_Module_Node_t *to_remove = *current;
            *current = (*current)->next;
            vPortFree(to_remove);
            ir_host_context.module_list.count--;
            xSemaphoreGive(ir_host_context.module_list.mutex);
            return true;
        }
        current = &(*current)->next;
    }

    xSemaphoreGive(ir_host_context.module_list.mutex);
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

uint8_t IR_Host_GetOnlineCount(void)
{
    uint8_t count = 0;
    IR_Module_Node_t *current = ir_host_context.module_list.head;
    while (current != NULL) {
        if (current->online && current->discovered) {
            count++;
        }
        current = current->next;
    }
    return count;
}

IR_Host_TaskState_t IR_Host_GetTaskState(void)
{
    return ir_host_context.task_state;
}

void IR_Host_ForceRediscover(void)
{
    IR_Module_Node_t *current = ir_host_context.module_list.head;
    while (current != NULL) {
        current->discovered = false;
        current->online = false;
        current->poll_fail_count = 0;
        current->consecutive_errors = 0;
        memset(&current->data_cache, 0, sizeof(IR_Module_DataCache_t));
        current = current->next;
    }
    ir_host_context.task_state = IR_HOST_TASK_STATE_DISCOVERY;
    ir_host_context.discovery_start_time = HAL_GetTick();
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
        xQueueSendFromISR(ir_host_context.rx_queue, &rx_frame, NULL);
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
                module->consecutive_errors = 0;
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
            vTaskDelay(pdMS_TO_TICKS(10));
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
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        } else {
            module->busy = false;
            module->status = IR_HOST_STATUS_ERROR;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return false;
}

bool IR_Host_Ping(uint8_t module_id, uint32_t timeout_ms)
{
    IR_Module_Node_t *module = IR_Host_FindModule(module_id);
    if (module == NULL) return false;

    if (module->busy) return false;

    uint32_t time_since_last_tx = HAL_GetTick() - module->last_tx_time;
    if (time_since_last_tx < IR_HOST_FRAME_INTERVAL_MS) return false;

    if (!IR_Host_SendCommand(module_id, IR_HOST_CMD_PING, NULL, 0)) {
        return false;
    }

    uint32_t start_time = HAL_GetTick();
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        if (!module->busy) {
            if (module->status == IR_HOST_STATUS_SUCCESS) {
                return true;
            }
            if (module->status == IR_HOST_STATUS_ERROR ||
                module->status == IR_HOST_STATUS_NACK ||
                module->status == IR_HOST_STATUS_TIMEOUT) {
                return false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    module->busy = false;
    module->status = IR_HOST_STATUS_TIMEOUT;
    return false;
}

bool IR_Host_ReadStatus(uint8_t module_id, uint8_t *status, uint32_t timeout_ms)
{
    IR_Module_Node_t *module = IR_Host_FindModule(module_id);
    if (module == NULL) return false;

    if (module->busy) return false;

    uint32_t time_since_last_tx = HAL_GetTick() - module->last_tx_time;
    if (time_since_last_tx < IR_HOST_FRAME_INTERVAL_MS) return false;

    if (!IR_Host_SendCommand(module_id, IR_HOST_CMD_READ_STATUS, NULL, 0)) {
        return false;
    }

    uint32_t start_time = HAL_GetTick();
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        if (!module->busy) {
            if (status != NULL && module->last_response.length > 0) {
                *status = module->last_response.data[0];
            }
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    module->busy = false;
    module->status = IR_HOST_STATUS_TIMEOUT;
    return false;
}

bool IR_Host_ResetModule(uint8_t module_id, uint32_t timeout_ms)
{
    IR_Module_Node_t *module = IR_Host_FindModule(module_id);
    if (module == NULL) return false;

    if (module->busy) return false;

    uint32_t time_since_last_tx = HAL_GetTick() - module->last_tx_time;
    if (time_since_last_tx < IR_HOST_FRAME_INTERVAL_MS) return false;

    if (!IR_Host_SendCommand(module_id, IR_HOST_CMD_RESET, NULL, 0)) {
        return false;
    }

    uint32_t start_time = HAL_GetTick();
    while ((HAL_GetTick() - start_time) < timeout_ms) {
        if (!module->busy) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    module->busy = false;
    module->status = IR_HOST_STATUS_TIMEOUT;
    return false;
}

IR_Data_CheckResult_t IR_Host_CheckDataConsistency(uint8_t module_id)
{
    IR_Module_Node_t *module = IR_Host_FindModule(module_id);

    if (module == NULL) {
        return IR_DATA_CHECK_NO_MODULE;
    }

    if (!module->online || !module->discovered) {
        return IR_DATA_CHECK_OFFLINE;
    }

    if (!module->data_cache.valid) {
        return IR_DATA_CHECK_CRC_ERR;
    }

    uint32_t time_since_update = HAL_GetTick() - module->data_cache.update_timestamp;
    if (time_since_update > IR_HOST_DATA_STALE_MS) {
        return IR_DATA_CHECK_STALE;
    }

    if (module->data_cache.consistent_count < IR_HOST_CONSISTENT_COUNT) {
        return IR_DATA_CHECK_INCONSISTENT;
    }

    return IR_DATA_CHECK_OK;
}

IR_Data_CheckResult_t IR_Host_CheckAllModules(void)
{
    IR_Data_CheckResult_t worst_result = IR_DATA_CHECK_OK;
    bool any_module_found = false;

    IR_Module_Node_t *current = ir_host_context.module_list.head;
    while (current != NULL) {
        any_module_found = true;
        IR_Data_CheckResult_t result = IR_Host_CheckDataConsistency(current->module_id);
        if (result > worst_result) {
            worst_result = result;
        }
        current = current->next;
    }

    if (!any_module_found) {
        return IR_DATA_CHECK_NO_MODULE;
    }

    return worst_result;
}

bool IR_Host_GetModuleData(uint8_t module_id, uint8_t *data, uint8_t *length)
{
    IR_Module_Node_t *module = IR_Host_FindModule(module_id);

    if (module == NULL || !module->data_cache.valid) {
        return false;
    }

    if (data != NULL) {
        memcpy(data, module->data_cache.raw_data, 6);
    }
    if (length != NULL) {
        *length = module->last_response.length > 6 ? 6 : module->last_response.length;
    }

    return true;
}

static void IR_Host_DiscoveryPhase(void)
{
    static uint8_t discover_index = 0;
    uint32_t now = HAL_GetTick();

    if (now - ir_host_context.discovery_start_time > 5000) {
        ir_host_context.task_state = IR_HOST_TASK_STATE_RUNNING;
        discover_index = 0;
        return;
    }

    IR_Module_Node_t *module = IR_Host_GetModuleByIndex(discover_index);
    if (module != NULL && !module->discovered) {
        if (!module->busy && (now - module->last_tx_time) > IR_HOST_POLL_INTERVAL_MS) {
            if (IR_Host_Ping(module->module_id, 200)) {
                module->discovered = true;
                module->online = true;
            } else {
                module->poll_fail_count++;
            }
        }
    }

    discover_index++;
    if (discover_index >= ir_host_context.module_list.count) {
        discover_index = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
}

static void IR_Host_PollModule(IR_Module_Node_t *module)
{
    if (module == NULL) return;

    if (!module->busy) {
        uint32_t now = HAL_GetTick();

        if (!module->online || !module->discovered) {
            if (now - module->last_tx_time > IR_HOST_POLL_INTERVAL_MS * 2) {
                if (IR_Host_Ping(module->module_id, 300)) {
                    module->discovered = true;
                    module->online = true;
                    module->consecutive_errors = 0;
                } else {
                    module->poll_fail_count++;
                    module->consecutive_errors++;
                    if (module->consecutive_errors > 10) {
                        module->online = false;
                        module->discovered = false;
                        module->data_cache.valid = false;
                    }
                }
            }
        } else {
            if (now - module->last_tx_time >= IR_HOST_POLL_INTERVAL_MS) {
                bool read_ok = IR_Host_ReadStatus(module->module_id, NULL, 300);
                if (read_ok) {
                    module->consecutive_errors = 0;
                } else {
                    module->poll_fail_count++;
                    module->consecutive_errors++;
                    if (module->consecutive_errors > 5) {
                        module->online = false;
                        module->data_cache.valid = false;
                    }
                }
            }
        }
    }
}

static void IR_Host_UpdateOnlineStatus(void)
{
    uint32_t now = HAL_GetTick();
    IR_Module_Node_t *current = ir_host_context.module_list.head;

    while (current != NULL) {
        if (current->online &&
            (now - current->last_rx_time) > IR_HOST_ONLINE_TIMEOUT_MS) {
            current->online = false;
            current->data_cache.valid = false;
        }

        if (current->data_cache.valid &&
            (now - current->data_cache.update_timestamp) > IR_HOST_DATA_STALE_MS * 3) {
            current->data_cache.valid = false;
        }

        current = current->next;
    }
}

void IR_Host_Task(void *argument)
{
    IR_Host_RxFrame_t rx_frame;
    (void)argument;

    ir_host_context.task_state = IR_HOST_TASK_STATE_DISCOVERY;
    ir_host_context.discovery_start_time = HAL_GetTick();
    ir_host_context.current_poll_index = 0;
    ir_host_context.last_poll_time = 0;

    for (;;) {
        while (xQueueReceive(ir_host_context.rx_queue, &rx_frame, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (rx_frame.can_id == IR_HOST_CAN_ID_DATA) {
                IR_Module_Node_t *module = IR_Host_FindModule(rx_frame.module_id);

                if (module != NULL && rx_frame.dlc >= 2) {
                    uint8_t received_crc = rx_frame.data[rx_frame.dlc - 1];
                    uint8_t calculated_crc = IR_Host_CRC8(rx_frame.data, rx_frame.dlc - 1);

                    module->data_cache.total_rx_count++;

                    if (received_crc == calculated_crc) {
                        uint8_t data_len = rx_frame.dlc - 2;
                        bool data_changed = false;

                        if (module->data_cache.valid &&
                            data_len > 0 &&
                            data_len <= 6 &&
                            memcmp(module->data_cache.raw_data,
                                   &rx_frame.data[1],
                                   data_len) != 0) {
                            data_changed = true;
                        }

                        if (data_len > 0) {
                            memcpy(module->data_cache.raw_data,
                                   &rx_frame.data[1],
                                   data_len > 6 ? 6 : data_len);
                        }
                        module->data_cache.update_timestamp = rx_frame.timestamp;
                        module->data_cache.valid = true;
                        module->data_cache.crc_error_count = 0;

                        if (!data_changed) {
                            if (module->data_cache.consistent_count < 255) {
                                module->data_cache.consistent_count++;
                            }
                        } else {
                            module->data_cache.consistent_count = 1;
                        }

                        module->last_response.module_id = rx_frame.data[0];
                        module->last_response.status = IR_HOST_STATUS_SUCCESS;
                        module->last_response.length = data_len;
                        module->last_response.timestamp = rx_frame.timestamp;
                        module->last_response.valid = true;

                        if (data_len > 0) {
                            memcpy(module->last_response.data,
                                   &rx_frame.data[1],
                                   data_len);
                        }

                        module->last_rx_time = rx_frame.timestamp;
                        module->online = true;
                        module->busy = false;
                        module->discovered = true;
                        module->poll_fail_count = 0;
                    } else {
                        module->data_cache.total_crc_error_count++;
                        module->data_cache.crc_error_count++;
                        if (module->data_cache.crc_error_count >= IR_HOST_MAX_CRC_ERRORS) {
                            module->data_cache.valid = false;
                        }
                    }
                }
            }
        }

        switch (ir_host_context.task_state) {
            case IR_HOST_TASK_STATE_DISCOVERY:
                IR_Host_DiscoveryPhase();
                break;

            case IR_HOST_TASK_STATE_RUNNING:
            {
                uint32_t now = HAL_GetTick();

                if (now - ir_host_context.last_poll_time >= IR_HOST_POLL_INTERVAL_MS) {
                    ir_host_context.last_poll_time = now;

                    IR_Module_Node_t *module = IR_Host_GetModuleByIndex(
                        ir_host_context.current_poll_index);

                    if (module != NULL) {
                        IR_Host_PollModule(module);
                    }

                    ir_host_context.current_poll_index++;
                    if (ir_host_context.current_poll_index >=
                        ir_host_context.module_list.count) {
                        ir_host_context.current_poll_index = 0;
                        IR_Host_UpdateOnlineStatus();
                    }
                } else {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                break;
            }

            case IR_HOST_TASK_STATE_ERROR:
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            default:
                ir_host_context.task_state = IR_HOST_TASK_STATE_DISCOVERY;
                ir_host_context.discovery_start_time = HAL_GetTick();
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
