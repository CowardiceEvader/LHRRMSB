/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"


#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#define ROS_CAN_CHASSIS_BROADCAST_DEBUG 1
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
�������, 0:���̵��1 3508���,  1:���̵��2 3508���,2:���̵��3 3508���,3:���̵��4 3508���;
4:yaw��̨��� 6020���; 5:pitch��̨��� 6020���; 6:������� 2006���*/
static motor_measure_t motor_chassis[10];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

//add some private variables
static CAN_TxHeaderTypeDef  gimbal_dual_tx_message;
static uint8_t              gimbal_dual_can_send_data[8];
	
static CAN_TxHeaderTypeDef  gimbal_dual_pitch_tx_message;
static uint8_t              gimbal_dual_pitch_can_send_data[8];

static volatile uint32_t can_rx_total_count = 0;
static volatile uint32_t can_rx_can1_count = 0;
static volatile uint32_t can_rx_can2_count = 0;
static volatile uint32_t can_tx_chassis_cmd_count = 0;
static volatile uint32_t can_tx_ok_count = 0;
static volatile uint32_t can_tx_fail_count = 0;
static volatile uint16_t can_last_rx_std_id = 0;
	
/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

//	if (hcan->Instance == CAN1)
//    {
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    can_rx_total_count++;
    can_last_rx_std_id = rx_header.StdId;
    if (hcan->Instance == CAN1)
    {
      can_rx_can1_count++;
    }
    else if (hcan->Instance == CAN2)
    {
      can_rx_can2_count++;
    }

		switch (rx_header.StdId)
		{
			case CAN_3508_M1_ID:
			case CAN_3508_M2_ID:
			case CAN_3508_M3_ID:
			case CAN_3508_M4_ID:
			case CAN_YAW_MOTOR_ID:
			case CAN_PIT_MOTOR_ID:
			case CAN_TRIGGER_MOTOR_ID:
			case CAN_YAW_1_MOTOR_ID:
            case CAN_PIT_1_MOTOR_ID:
			case CAN_TRIGGER_1_MOTOR_ID:	
			{
				static uint8_t i = 0;
				//get motor id
				i = rx_header.StdId - CAN_3508_M1_ID;
				get_motor_measure(&motor_chassis[i], rx_data);
				detect_hook(CHASSIS_MOTOR1_TOE + i);
				break;
			}

			default:
			{
				break;
			}
		}
//	}
//	if (hcan->Instance == CAN2)
//    {
//		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
//		
//		switch(rx_header.StdId)
//		{
//			
//            {
//                static uint8_t i = 0;
//                i = rx_header.StdId - CAN_3508_M1_ID;
//                get_motor_measure(&motor_chassis[i], rx_data);
////                detect_hook(CHASSIS_MOTOR1_TOE + i);
//                break;
//            }
//			
//			default:
//			{
//				break;
//			}
//		}
//	}
}



/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      shoot: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rev: (0x208) ������������Ƶ���
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
  HAL_StatusTypeDef tx_status;
  can_tx_chassis_cmd_count++;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    tx_status = HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
    if (tx_status == HAL_OK)
    {
      can_tx_ok_count++;
    }
    else
    {
      can_tx_fail_count++;
    }

  #if ROS_CAN_CHASSIS_BROADCAST_DEBUG
    tx_status = HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
    if (tx_status == HAL_OK)
    {
      can_tx_ok_count++;
    }
    else
    {
      can_tx_fail_count++;
    }

    tx_status = HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
    if (tx_status == HAL_OK)
    {
      can_tx_ok_count++;
    }
    else
    {
      can_tx_fail_count++;
    }
  #endif
}


/**
  * @brief          send control current of motor (0x208, 0x209, 0x206)
  * @param[in]      yaw: (0x208) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x209) 6020 motor control current, range [-30000,30000] 
  * @param[in]      trriger: (0x206) 2006 motor control current, range [-10000,10000] 
  * @param[in]      rev: (0x208) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x208) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      pitch: (0x209) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      trriger: (0x206) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      rev: (0x208) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
void CAN_cmd_gimbal_dual(int16_t rev1, int16_t reve, int16_t rev, int16_t trigger)
{
    uint32_t send_mail_box;
    gimbal_dual_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_dual_tx_message.IDE = CAN_ID_STD;
    gimbal_dual_tx_message.RTR = CAN_RTR_DATA;
    gimbal_dual_tx_message.DLC = 0x08;
    gimbal_dual_can_send_data[0] = rev1 >> 8;
    gimbal_dual_can_send_data[1] = rev1;
    gimbal_dual_can_send_data[2] = reve >> 8;
    gimbal_dual_can_send_data[3] = reve;
    gimbal_dual_can_send_data[4] = rev >> 8;
    gimbal_dual_can_send_data[5] = rev;
    gimbal_dual_can_send_data[6] = trigger >> 8;
    gimbal_dual_can_send_data[7] = trigger;

    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_dual_tx_message, gimbal_dual_can_send_data, &send_mail_box);
}



/**
  * @brief          send control current of motor (0x208, 0x209, 0x206)
  * @param[in]      pitch: (0x209) 6020 motor control current, range [-30000,30000] 
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      pitch: (0x208) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
void CAN_cmd_gimbal_dual_pitch(int16_t pitch, int16_t yaw, int16_t rev2, int16_t rev3)
{
    uint32_t send_mail_box;
    gimbal_dual_tx_message.StdId = CAN_GIMBAL_1_ALL_ID;
    gimbal_dual_tx_message.IDE = CAN_ID_STD;
    gimbal_dual_tx_message.RTR = CAN_RTR_DATA;
    gimbal_dual_tx_message.DLC = 0x08;
    gimbal_dual_can_send_data[0] = pitch >> 8;
    gimbal_dual_can_send_data[1] = pitch;
    gimbal_dual_can_send_data[2] = yaw >> 8;
    gimbal_dual_can_send_data[3] = yaw;
    gimbal_dual_can_send_data[4] = rev2 >> 8;
    gimbal_dual_can_send_data[5] = rev2;
    gimbal_dual_can_send_data[6] = rev3 >> 8;
    gimbal_dual_can_send_data[7] = rev3;

    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_dual_pitch_tx_message, gimbal_dual_pitch_can_send_data, &send_mail_box);
}


/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����yaw 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����pitch 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}


/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}


/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����yaw 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_yaw_gimbal_dual_motor_measure_point(void)
{
    return &motor_chassis[7];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����pitch 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_pitch_gimbal_dual_motor_measure_point(void)
{
    return &motor_chassis[8];
}

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_trigger_dual_motor_measure_point(void)
{
    return &motor_chassis[9];
}

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

can_debug_info_t get_can_debug_info(void)
{
  can_debug_info_t info;
  info.rx_total_count = can_rx_total_count;
  info.rx_can1_count = can_rx_can1_count;
  info.rx_can2_count = can_rx_can2_count;
  info.tx_chassis_cmd_count = can_tx_chassis_cmd_count;
  info.tx_ok_count = can_tx_ok_count;
  info.tx_fail_count = can_tx_fail_count;
  info.last_rx_std_id = can_last_rx_std_id;
  info.can1_error_code = HAL_CAN_GetError(&hcan1);
  info.can2_error_code = HAL_CAN_GetError(&hcan2);
  info.can1_tx_free_level = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
  info.can2_tx_free_level = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);
  return info;
}
