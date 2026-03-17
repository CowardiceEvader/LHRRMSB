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
#include "usb_task.h"  // Include so we can use usb_printf
//#include "bsp_usart.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern RC_ctrl_t rc_ctrl;

uint8_t  usart1_dma_rx_buffer[USART1_DMA_BUF_NUM];
//uint8_t  usart1_dma_tx_buffer[2][USART1_DMA_BUF_NUM];

uint16_t HP = 0;

uint16_t yaw1,pitch1;
//usart1 dma data string
#include <stdarg.h>

// 给 UART1 准备一个专用的 DMA 打印输出函数
// 不会影响 UART1 的高速 RX 收包，硬件级隔离
void uart1_printf(const char *fmt, ...)
{
    static uint8_t uart1_tx_buf[256];
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);
    len = vsprintf((char *)uart1_tx_buf, fmt, ap);
    va_end(ap);

    // 确保长度不超过缓冲区，防止溢出越界
    if(len > 255) len = 255; 
    
    // 如果想要完全安全，可以先检查上一次 DMA 是否发完了
    // while (huart1.gState != HAL_UART_STATE_READY && huart1.gState != HAL_UART_STATE_BUSY_RX);

    // 调用 DMA 直接往外射，彻底不卡死 CPU
    HAL_UART_Transmit_DMA(&huart1, uart1_tx_buf, len);
}

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

    // [调试代码] 通过 USB (虚拟串口) 打印接收到的数据，以供电脑端独立查看
    if(this_time_data_rx_len > 0)
    {
        // ------------- 开始解析逻辑的地方 ---------------
        // 你可以在这里做协议拆包：
        // 比如 if(usart1_dma_rx_buffer[0] == 0xAA) ...等

        // ------------- 以下给 Linux 发送调试数据 ---------------
        // 为了防止发送缓冲区被连续快速踩踏，先拼写在这个函数的局部大数组里，然后一次过抛出去
        static char debug_str[128];
        int pos = 0;
        pos += sprintf(debug_str + pos, "I got %d bytes: ", this_time_data_rx_len);
        
        for(uint8_t i = 0; i < this_time_data_rx_len && pos < 120; i++)
        {
            pos += sprintf(debug_str + pos, "%02X ", usart1_dma_rx_buffer[i]);
        }
        sprintf(debug_str + pos, "\r\n");
        
        // 用新的函数，顺着 UART1_TX 的线发给你的 Linux Minicom!
        uart1_printf("%s", debug_str);
    }
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
//		UART_SendString(&huart1, usart1_dma_rx_buffer);  // 发送字符串


		if(this_time_data_rx_len == RX_DATA_LEN)
            {
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
//			UART_SendString(&huart1, usart1_dma_rx_buffer);  // 发送字符串
            }
//		HAL_UART_Transmit_DMA(&huart1, usart1_dma_tx_buffer, USART1_DMA_BUF_NUM);
//		UART_SendString(&huart1, usart1_dma_rx_buffer);  // 发送字符串
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
//    if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
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
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //重新设定数据长度
//            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

//            //set memory buffer 1
//            //设定缓冲区1
//            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
//            
//            //enable DMA
//            //使能DMA
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
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

//            //reset set_data_lenght
//            //重新设定数据长度
//            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

//            //set memory buffer 0
//            //设定缓冲区0
//            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
//            
//            //enable DMA
//            //使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart1_rx);

//            if(this_time_rx_len == RC_FRAME_LENGTH)
//            {
//				memcpy(sbus_rx_buf, usart1_dma_rx_buffer, 18);
//                //处理遥控器数据
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

// 发送字符串
void UART_SendString(UART_HandleTypeDef *huart, uint8_t *str)
{
    while (*str) {
        HAL_UART_Transmit(huart, str++, 1, 1000);  // 发送一个字节
    }
}

void send_uint16_t(uint16_t data) 
{
    uint8_t buffer[2];  // 存储 uint16_t 的高字节和低字节

    // 将 uint16_t 类型数据转换为字节数组
    buffer[0] = (uint8_t)(data >> 8);  // 高字节
    buffer[1] = (uint8_t)(data & 0xFF); // 低字节

    // 通过 UART 发送字节数据
    HAL_UART_Transmit(&huart1, buffer, 2, HAL_MAX_DELAY);  // 发送 2 个字节
}
