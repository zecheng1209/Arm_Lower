#include "Task_Init.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "Run.h"
#include "math.h"
#include "stdbool.h"

extern Joint_t Joint[5];
float Motor_Init[4] = {0};

extern TaskHandle_t Motor_Drive_Handle;
extern TaskHandle_t Motor_RM_Handle;
//extern TaskHandle_t MotorRecTask_Handle;

//TaskHandle_t Motor_Reset_Handle;

void MotorInit(void);
void Motor_RM(void *param);
void Motor_reset(void *param);

bool Float_S(float a, float b)
{
		return fabsf(a - b) < 0.03f;
}

extern RobStride_t rs03;
extern RobStride_t rs02;
extern float rs03_torque;
extern float rs02_torque;

void Task_Init(void)
{
	  
    CanFilter_Init(&hcan1);
    CanFilter_Init(&hcan2);
    HAL_CAN_Start(&hcan1); 
    HAL_CAN_Start(&hcan2);
	  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);//4.2黄绿绿绿板子的can1口是坏的。另附：使用can口记得在该总线上加一个终端电阻。
    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);
//    HAL_CAN_ActivateNotification(&hcan1,CAN_IT_TX_MAILBOX_EMPTY);
//    HAL_CAN_ActivateNotification(&hcan2,CAN_IT_TX_MAILBOX_EMPTY);
    vTaskDelay(2000);
    RobStride_t rs02={.hcan=&hcan2,.motor_id=0x03,.type= RobStride_02,};
    RobStride_t rs03={.hcan=&hcan2,.motor_id=0x02,.type= RobStride_03,};
	  RobStrideInit(&rs03, &hcan2, 0x02, RobStride_03);
	  RobStrideInit(&rs02, &hcan2, 0x03, RobStride_02);
	  RobStrideSetMode(&rs03, RobStride_Torque);
	  RobStrideSetMode(&rs02, RobStride_Torque);
		vTaskDelay(100);
  	RobStrideEnable(&rs03);
	  RobStrideEnable(&rs02);

	  //vTaskDelay(100);
    //vTaskDelay(2000);
    MotorInit();
    
	xTaskCreate(Motor_Drive, "Motor_Drive", 256, NULL, 4, &Motor_Drive_Handle);
	xTaskCreate(Motor_RM, "Motor_RM", 256, NULL, 4, &Motor_RM_Handle);//驱动
//		xTaskCreate(Motor_reset, "Motor_reset", 300, NULL, 4, &Motor_Reset_Handle);//复位
    //xTaskCreate(MotorSendTask, "MotorSendTask", 128, NULL, 4, &MotorSendTask_Handle);//将数据发送到PC
}


void MotorInit() {
    Joint[0].Rs_motor.hcan = &hcan2;
    Joint[0].Rs_motor.motor_id = 0x02;
    Joint[0].Rs_motor.type = RobStride_03;
    // PID rs03
    Joint[0].vel_pid.Kp = 6.0f; 
	  Joint[0].vel_pid.Ki = 0.0f;  
	  Joint[0].vel_pid.Kd = 3.0f;
    Joint[0].vel_pid.limit = 20.0f; 
	  Joint[0].vel_pid.output_limit = 35.0f;
	
    Joint[0].pos_pid.Kp = 100.0f; 
	  Joint[0].pos_pid.Ki = 0.0f; 
	  Joint[0].pos_pid.Kd = 5.0f;
    Joint[0].pos_pid.limit = 15.0f; 
	  Joint[0].pos_pid.output_limit = 5.0f;

    Joint[1].Rs_motor.hcan = &hcan2;
    Joint[1].Rs_motor.motor_id = 0x03;
    Joint[1].Rs_motor.type = RobStride_02;
    // PID rs02
    Joint[1].vel_pid.Kp = 3.0f;  
		Joint[1].vel_pid.Ki = 0.0f;  
		Joint[1].vel_pid.Kd = 1.0f;
    Joint[1].vel_pid.limit = 4.0f; 
		Joint[1].vel_pid.output_limit = 25.0f;
	
    Joint[1].pos_pid.Kp = 11.5f; 
		Joint[1].pos_pid.Ki = 0.0f;  
		Joint[1].pos_pid.Kd = 1.0f;
    Joint[1].pos_pid.limit = 5.0f; 
		Joint[1].pos_pid.output_limit = 2.3f;

    //Joint[2].type = TYPE_RM3508;
    Joint[2].RM_3508.ID = 0x201;
    Joint[2].RM_3508.hcan = &hcan1;
    Joint[2].pos_pid.Kp = 0.4f;  
		Joint[2].pos_pid.Ki = 0.0f; 
		Joint[2].pos_pid.Kd = 0.005f;
    Joint[2].pos_pid.limit = 1000.0f;
		Joint[2].pos_pid.output_limit = 5000.0f;
    Joint[2].vel_pid.Kp = 18.0f; 
		Joint[2].vel_pid.Kd = 0.1f;
    Joint[2].vel_pid.output_limit = 16384.0f;

    //Joint[3].type = TYPE_RM2006;
    Joint[3].RM_2006.ID = 0x202;
    Joint[3].RM_2006.hcan = &hcan1;
    Joint[3].pos_pid.Kp = 1.0f; 
    Joint[3].pos_pid.Ki = 0.0f; 
		Joint[3].pos_pid.Kd = 0.0f;		
		Joint[3].pos_pid.output_limit = 2000.0f;
    Joint[3].vel_pid.Kp = 10.0f; 
		Joint[3].vel_pid.Ki = 0.01f;
    Joint[3].vel_pid.output_limit = 16384.0f;
}

void RampToTarget(float *val, float target, float step)//斜坡
{
    float diff = target - *val;

    if (fabsf(diff) < step)
    {
        *val = target;
    }
    else
    {
        *val += (diff > 0 ? step : -step);
    }
}

uint8_t ready=0;

void Motor_reset(void *param)
{
    TickType_t Last_wake_time = xTaskGetTickCount();
		
		vTaskDelay(200);
		
		Motor_Init[0] = Joint[0].Rs_motor.state.rad;
		Motor_Init[1] = Joint[1].Rs_motor.state.rad;
//		Motor_Init[2] = Joint[2].RM_3508.motor.Angle;
	//	Motor_Init[3] = Joint[3].RM_2006.motor.Angle;
		
		Joint[0].exp_rad = Motor_Init[0] - Joint[0].pos_offset;
		Joint[1].exp_rad = Motor_Init[1] - Joint[1].pos_offset;
		Joint[2].exp_rad = Motor_Init[2] - Joint[2].pos_offset;
		Joint[3].exp_rad = Motor_Init[3] - Joint[3].pos_offset;
    for (;;)
    {
/*        RampToTarget(&Joint[0].exp_rad, 0, 0.0005f);
				RampToTarget(&Joint[1].exp_rad, 0, 0.0005f);
				RampToTarget(&Joint[2].exp_rad, -1.57, 0.0002f);
				RampToTarget(&Joint[3].exp_rad, 0, 0.005f);     */
			  
				
//				if(Joint_FinInit())
//				{
//					ready=1;
//						xTaskCreate(MotorRecTask, "MotorRecTask", 200, NULL, 4, &MotorRecTask_Handle);//PC接收数据
//						vTaskDelete(NULL);
//				}
				
				vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(5));
    }
}

void PID_Init_Pos(Joint_t *Joint, float kp, float ki, float kd, float limit, float pid_out)
{
	Joint->pos_pid.Kp = kp;
	Joint->pos_pid.Ki = ki;
	Joint->pos_pid.Kd = kd;
	Joint->pos_pid.limit = limit;
	Joint->pos_pid.output_limit = pid_out;
}

void PID_Init_Vel(Joint_t *Joint, float kp, float ki, float kd, float limit, float pid_out)
{
	Joint->vel_pid.Kp = kp;
	Joint->vel_pid.Ki = ki;
	Joint->vel_pid.Kd = kd;
	Joint->vel_pid.limit = limit;
	Joint->vel_pid.output_limit = pid_out;
}

void RS_Offest_inv(Joint_t *Joint, int8_t inv_motor, float pos_offset)
{
	Joint->inv_motor = inv_motor;
	Joint->pos_offset = pos_offset;
}

