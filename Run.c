#include "Run.h"
#include "usb_trans.h"
#include "usbd_cdc_if.h"
#include "PID.h"
#include "math.h"
#include "Task_Init.h"

extern uint8_t ready;

// 定义关节数组
Joint_t Joint[5]; 
int16_t can_buf[4] = {0};

TaskHandle_t Motor_Drive_Handle;

float M0 = 7.7f; // rs03 (Joint[0]) 水平位置前馈
int16_t Can1_TxData[4] = {0};

typedef enum {
    init,         // 初始化
    moving1,      // 行进
    moving2,      // 抓杆
    moving22,     // 抓杆2
    moving3,      // 对接
    moving32      // 对接2
} STATE;

STATE task_state = init;

void RampToTarget();
void Motor_Drive(void *param) {
    TickType_t Last_wake_time = xTaskGetTickCount();

    for(;;) {
        // 获取反馈：Joint[0]是rs03, Joint[1]是rs02
        float curr_rad03 = Joint[0].Rs_motor.state.rad;
        float curr_omega03 = Joint[0].Rs_motor.state.omega;
        float curr_rad02 = Joint[1].Rs_motor.state.rad;
        float curr_omega02 = Joint[1].Rs_motor.state.omega;

        switch(task_state) {
            case init:
                RampToTarget(&Joint[0].exp_rad, 4.17f, 0.001f); // rs03
                RampToTarget(&Joint[1].exp_rad, 0.3f, 0.001f);  // rs02
						    RampToTarget(&Joint[2].exp_rad, 0.0f, 1.0f); // 3508
                RampToTarget(&Joint[3].exp_rad, 0.0f, 1.0f); // 2006
                break;
            case moving1:
                RampToTarget(&Joint[0].exp_rad, 4.6f, 0.01f);   // rs03
                RampToTarget(&Joint[1].exp_rad, -1.7f, 0.01f);  // rs02
					    	RampToTarget(&Joint[2].exp_rad, 0.0f, 1.0f); // 3508
                RampToTarget(&Joint[3].exp_rad, 0.0f, 1.0f); // 2006
                break;
            case moving2:
                RampToTarget(&Joint[0].exp_rad, 4.25f, 0.01f);  // rs03
                RampToTarget(&Joint[1].exp_rad, 1.47f, 0.01f);  // rs02
					    	RampToTarget(&Joint[2].exp_rad, 0.0f, 1.0f); // 3508
                RampToTarget(&Joint[3].exp_rad, 0.0f, 1.0f); // 2006
                break;
            case moving22:
                RampToTarget(&Joint[0].exp_rad, 4.30f, 0.01f);  // rs03
                RampToTarget(&Joint[1].exp_rad, 1.50f, 0.01f);  // rs02
    			    	RampToTarget(&Joint[2].exp_rad, 0.0f, 1.0f); // 3508
                RampToTarget(&Joint[3].exp_rad, 0.0f, 1.0f); // 2006
                break;
            case moving3:
                RampToTarget(&Joint[0].exp_rad, 4.7f, 0.01f);   // rs03
                RampToTarget(&Joint[1].exp_rad, -0.25f, 0.01f); // rs02
								RampToTarget(&Joint[2].exp_rad, -38500.0f, 50.0f);// 3508
                RampToTarget(&Joint[3].exp_rad, 0.0f, 1.0f);      // 2006
                break;
            case moving32: 
                RampToTarget(&Joint[0].exp_rad, 4.64f, 0.01f);  // rs03
                RampToTarget(&Joint[1].exp_rad, -0.25f, 0.01f); // rs02
								RampToTarget(&Joint[2].exp_rad, -39000.0f, 50.0f);// 3508
                RampToTarget(&Joint[3].exp_rad, 0.0f, 1.0f);      // 2006
                break;
            default: break;
        }

        // 计算 rs03 (Joint[0]) 的前馈
        float M1 = M0 * cos(Joint[0].Rs_motor.state.rad - 4.12f);

        PID_Control(curr_rad03, Joint[0].exp_rad + Joint[0].pos_offset, &Joint[0].pos_pid);
        PID_Control(curr_omega03, Joint[0].pos_pid.pid_out, &Joint[0].vel_pid);

        PID_Control(curr_rad02, Joint[1].exp_rad + Joint[1].pos_offset, &Joint[1].pos_pid);
        PID_Control(curr_omega02, Joint[1].pos_pid.pid_out, &Joint[1].vel_pid);

        float rs03_torque = M1 + Joint[0].vel_pid.pid_out;
        float rs02_torque = Joint[1].vel_pid.pid_out;

        RobStrideTorqueControl(&Joint[0].Rs_motor, rs03_torque);
        RobStrideTorqueControl(&Joint[1].Rs_motor, rs02_torque);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			  PID_Control(Joint[2].RM_3508.actual_pos, Joint[2].exp_rad, &Joint[2].pos_pid);
        PID_Control(Joint[2].RM_3508.motor.Speed, Joint[2].pos_pid.pid_out, &Joint[2].vel_pid);

        PID_Control(Joint[3].RM_2006.actual_pos, Joint[3].exp_rad, &Joint[3].pos_pid);
        PID_Control(Joint[3].RM_2006.motor.Speed, Joint[3].pos_pid.pid_out, &Joint[3].vel_pid);

        Can1_TxData[0] = (int16_t)Joint[2].vel_pid.pid_out;
        Can1_TxData[1] = (int16_t)Joint[3].vel_pid.pid_out;
        MotorSend(&hcan1, 0x200, Can1_TxData);

        vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(2));
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    uint8_t buf[8];
    uint16_t ID = CAN_Receive_DataFrame(&hcan1, buf);
  	Motor3508Recv(&Joint[2].RM_3508, &hcan1, ID, buf);
    Motor2006Recv(&Joint[3].RM_2006, &hcan1, ID, buf);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    uint8_t buf[8];
    uint32_t ID = CAN_Receive_DataFrame(hcan, buf);
    // Joint[0]是rs03, Joint[1]是rs02
    RobStrideRecv_Handle(&Joint[0].Rs_motor, &hcan2, ID, buf);
    RobStrideRecv_Handle(&Joint[1].Rs_motor, &hcan2, ID, buf);
}