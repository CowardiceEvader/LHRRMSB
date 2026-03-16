/**
  ******************************************************************************
  * @FileName       aim_assist_control.c/h
  * @Description    Aiming Assistance Control for Visual serial communication
  * @author         Xiao TY
  * @note
  ******************************************************************************
  *
  * Copyright (c) 2024 Yangtze University
  * All rights reserved.
  *
  ******************************************************************************
**/

#include "aim_assist_control.h"

#include "main.h"
#include "gimbal_task.h"

#include "string.h"
#include "detect_task.h"
#include "referee.h"
#include "bsp_rc.h"
#include "stdlib.h"
#include "bsp_usart.h"

#define DEBUG_UART1_RX_FORWARD_TO_UART6 1
#define DEBUG_UART1_RX_FORWARD_TO_CDC 0

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern RC_ctrl_t rc_ctrl;

uint8_t  usart1_dma_rx_buffer[USART1_DMA_BUF_NUM];
static uint8_t uart1_rx_forward_buf[RX_DATA_LEN];
//uint8_t  usart1_dma_tx_buffer[2][USART1_DMA_BUF_NUM];

uint16_t HP = 0;

uint16_t yaw1,pitch1;
//usart1 dma data string


get_data_t get_data;
send_data_t send_data;

//RC_ctrl_t rc_ctrl;

const gimbal_control_t *data;
 
int assistant_flag;
unsigned char pitch_bit,yaw_bit;

void aim_assistant_control_init(void)
{
    assistant_flag = 0;

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, usart1_dma_rx_buffer, USART1_DMA_BUF_NUM);
    data = get_gimbal_control_point();
//	Assitant_Init(usart1_dma_rx_buffer[0],usart1_dma_rx_buffer[1],USART1_DMA_BUF_NUM);
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart == &huart1)
//    {

//        memset(usart1_dma_tx_buffer, 0, TX_DATA_LEN);

//        if (data->gimbal_pitch_motor.absolute_angle < 0)
//        {
//            pitch_bit = 0x00;
//            send_data.pitch_data.f = -data->gimbal_pitch_motor.absolute_angle;
//        }
//        else
//        {
//            pitch_bit = 0x01;
//            send_data.pitch_data.f = data->gimbal_pitch_motor.absolute_angle;
//        }

//        if (data->gimbal_yaw_motor.absolute_angle < 0)
//        {
//            yaw_bit = 0x00;
//            send_data.yaw_data.f = -data->gimbal_yaw_motor.absolute_angle;
//        }
//        else
//        {
//            yaw_bit = 0x01;
//            send_data.yaw_data.f = data->gimbal_yaw_motor.absolute_angle;
//        }

//        usart1_dma_tx_buffer[0] = 0xFF;
//        usart1_dma_tx_buffer[1] = send_data.pitch_data.c[0];
//        usart1_dma_tx_buffer[2] = send_data.pitch_data.c[1];
//        usart1_dma_tx_buffer[3] = send_data.pitch_data.c[2];
//        usart1_dma_tx_buffer[4] = send_data.pitch_data.c[3];

//        usart1_dma_tx_buffer[6] = send_data.yaw_data.c[0];
//        usart1_dma_tx_buffer[7] = send_data.yaw_data.c[1];
//        usart1_dma_tx_buffer[8] = send_data.yaw_data.c[2];
//        usart1_dma_tx_buffer[9] = send_data.yaw_data.c[3];
//        usart1_dma_tx_buffer[10] = yaw_bit;

//        usart1_dma_tx_buffer[11] = get_robot_id();
//        usart1_dma_tx_buffer[12] = data->shoot_mode;
//        usart1_dma_tx_buffer[13] = 0xFF;
////        HAL_UART_Receive_DMA(&huart1, usart1_dma_rx_buffer, USART1_DMA_BUF_NUM);
//    }
//}

void Usart1Receive_IDLE(void)
{
    HAL_UART_DMAStop(&huart1);
    static uint8_t this_time_data_rx_len = 0;
    this_time_data_rx_len = USART1_DMA_BUF_NUM - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
//    if (this_time_data_rx_len == RX_DATA_LEN)
//    {
//        if (usart1_dma_rx_buffer[0] == 0xAA)
//        {
//            HAL_UART_Transmit_DMA(&huart1, usart1_dma_tx_buffer, TX_DATA_LEN);
//            get_data.pitch_data.c[0] = usart1_dma_rx_buffer[1];
//            get_data.pitch_data.c[1] = usart1_dma_rx_buffer[2];
//            get_data.pitch_data.c[2] = usart1_dma_rx_buffer[3];
//            get_data.pitch_data.c[3] = usart1_dma_rx_buffer[4];

//            get_data.yaw_data.c[0] = usart1_dma_rx_buffer[6];
//            get_data.yaw_data.c[1] = usart1_dma_rx_buffer[7];
//            get_data.yaw_data.c[2] = usart1_dma_rx_buffer[8];
//            get_data.yaw_data.c[3] = usart1_dma_rx_buffer[9];

//            get_data.distance = usart1_dma_rx_buffer[12];
//            
//            if (usart1_dma_rx_buffer[5] == 0x00)
//            {
//                get_data.pitch_data.f = -__fabs(get_data.pitch_data.f);
//            }
//            else
//            {
//                get_data.pitch_data.f = __fabs(get_data.pitch_data.f);
//            }
//            if (usart1_dma_rx_buffer[10] == 0x00)
//            {
//                get_data.yaw_data.f = -__fabs(get_data.yaw_data.f);
//            }
//            else
//            {
//                get_data.yaw_data.f = __fabs(get_data.yaw_data.f);
//            }

//            if (usart1_dma_rx_buffer[11] == 0x00)
//            {
//                get_data.isThereArmor = 0;
//            }
//            else
//            {
//                get_data.isThereArmor = 1;
//            }
//            if (usart1_dma_rx_buffer[13] == 0x00)
//            {
//                get_data.isShoot = 0;
//            }
//            else
//            {
//                get_data.isShoot = 1;
//            }
//        }
//    }
//    if(this_time_data_rx_len == RX_DATA_LEN)
//    {
////        char tempdatarx[2] = {usart1_dma_rx_buffer[2], usart1_dma_rx_buffer[4]};
////        char tempdatary[2] = {usart1_dma_rx_buffer[7], usart1_dma_rx_buffer[9]};
//        if(usart1_dma_rx_buffer[2] == 0x30)
//        {
////            get_data.yaw_data.f = atof(tempdatarx);//rx101ry002f0
//			yaw1 = 500;
//			
//        }
//        else
//        {
//            get_data.yaw_data.f = -atof(tempdatarx);
//		}
//        if(usart1_dma_rx_buffer[7] == 0x30)
//        {
//            get_data.pitch_data.f = atof(tempdatary);
//        }
//        else
//        {
//            get_data.pitch_data.f = -atof(tempdatary);
//        }
//		UART_SendString(&huart1, usart1_dma_rx_buffer);  // �����ַ���


		if(this_time_data_rx_len == RX_DATA_LEN)
            {
        memcpy(uart1_rx_forward_buf, usart1_dma_rx_buffer, RX_DATA_LEN);
            #if DEBUG_UART1_RX_FORWARD_TO_UART6
            usart6_tx_dma_enable(uart1_rx_forward_buf, RX_DATA_LEN);
            #endif
            #if DEBUG_UART1_RX_FORWARD_TO_CDC
            CDC_Transmit_FS(uart1_rx_forward_buf, RX_DATA_LEN);
            #endif
			memcpy(sbus_rx_buf, usart1_dma_rx_buffer, 18);
            sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            detect_hook(DBUS_TOE);
            if (rc_ctrl.rc.s[0] != RC_SW_UP && rc_ctrl.rc.s[0] != RC_SW_MID && rc_ctrl.rc.s[0] != RC_SW_DOWN)
            {
                rc_ctrl.rc.s[0] = RC_SW_UP;
            }
            if (rc_ctrl.rc.s[1] != RC_SW_UP && rc_ctrl.rc.s[1] != RC_SW_MID && rc_ctrl.rc.s[1] != RC_SW_DOWN)
            {
                rc_ctrl.rc.s[1] = RC_SW_UP;
            }
			HP = get_robot_remain_HP();
			send_uint16_t(HP);				
//          sbus_to_usart1(sbus_rx_buf[0]);
//			UART_SendString(&huart1, usart1_dma_rx_buffer);  // �����ַ���
            }
//		HAL_UART_Transmit_DMA(&huart1, usart1_dma_tx_buffer, USART1_DMA_BUF_NUM);
//		UART_SendString(&huart1, usart1_dma_rx_buffer);  // �����ַ���
        memset(usart1_dma_rx_buffer, 0, USART1_DMA_BUF_NUM);
		
        HAL_UART_Receive_DMA(&huart1, usart1_dma_rx_buffer, USART1_DMA_BUF_NUM);
}

void USART1_IRQHandler(void)
{
    if (RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart1);
        //into our type function
        Usart1Receive_IDLE();
    }
    HAL_UART_IRQHandler(&huart1);
}

//void USART1_IRQHandler(void)
//{
//    if(huart1.Instance->SR & UART_FLAG_RXNE)//���յ�����
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart1);
//    }
//    else if(USART1->SR & UART_FLAG_IDLE)
//    {
//        static uint16_t this_time_rx_len = 0;

//        __HAL_UART_CLEAR_PEFLAG(&huart1);

//        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            /* Current memory buffer used is Memory 0 */

//            //disable DMA
//            //ʧЧDMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
//            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //�����趨���ݳ���
//            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

//            //set memory buffer 1
//            //�趨������1
//            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
//            
//            //enable DMA
//            //ʹ��DMA
//            __HAL_DMA_ENABLE(&hdma_usart1_rx);

//            if(this_time_rx_len == RC_FRAME_LENGTH)
//            {
//				memcpy(sbus_rx_buf, usart1_dma_rx_buffer, 18);
//                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);

//                sbus_to_usart1(sbus_rx_buf[0]);

//            }
//        }
//        else
//        {
//            /* Current memory buffer used is Memory 1 */
//            //disable DMA
//            //ʧЧDMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
//            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //�����趨���ݳ���
//            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

//            //set memory buffer 0
//            //�趨������0
//            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
//            
//            //enable DMA
//            //ʹ��DMA
//            __HAL_DMA_ENABLE(&hdma_usart1_rx);

//            if(this_time_rx_len == RC_FRAME_LENGTH)
//            {
//				memcpy(sbus_rx_buf, usart1_dma_rx_buffer, 18);
//                //����ң��������
//                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
//				sbus_to_usart1(sbus_rx_buf[1]);
//            }
//        }
//    }

//}


const get_data_t  *get_aim_assistant_control_point(void)
{
    return &get_data;
}

// �����ַ���
void UART_SendString(UART_HandleTypeDef *huart, uint8_t *str)
{
    while (*str) {
        HAL_UART_Transmit(huart, str++, 1, 1000);  // ����һ���ֽ�
    }
}

void send_uint16_t(uint16_t data) 
{
    uint8_t buffer[2];  // �洢 uint16_t �ĸ��ֽں͵��ֽ�

    // �� uint16_t ��������ת��Ϊ�ֽ�����
    buffer[0] = (uint8_t)(data >> 8);  // ���ֽ�
    buffer[1] = (uint8_t)(data & 0xFF); // ���ֽ�

    // ͨ�� UART �����ֽ�����
    HAL_UART_Transmit(&huart1, buffer, 2, HAL_MAX_DELAY);  // ���� 2 ���ֽ�
}
