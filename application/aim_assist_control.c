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
#include "stdio.h"
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

typedef struct
{
    uint32_t idle_irq_count;
    uint32_t ok18_count;
    uint32_t len_err_count;
    uint32_t rc_data_err_count;
    uint32_t fe_count;
    uint32_t ne_count;
    uint32_t ore_count;
    uint32_t pe_count;
    uint16_t last_len;
    uint8_t last_head[6];
    uint8_t first_ok_reported;
    uint32_t last_report_tick;
} uart1_diag_info_t;

static uart1_diag_info_t uart1_diag = {0};

// �� UART1 ׼��һ��ר�õ� DMA ��ӡ�������
// ����Ӱ�� UART1 �ĸ��� RX �հ���Ӳ��������
void uart1_printf(const char *fmt, ...)
{
    static uint8_t uart1_tx_buf[256];
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);
    len = vsprintf((char *)uart1_tx_buf, fmt, ap);
    va_end(ap);

    // ȷ�����Ȳ���������������ֹ���Խ��
    if(len > 255) len = 255; 
    
    // �����Ҫ��ȫ��ȫ�������ȼ����һ�� DMA �Ƿ�����
    // while (huart1.gState != HAL_UART_STATE_READY && huart1.gState != HAL_UART_STATE_BUSY_RX);

    if (huart1.gState == HAL_UART_STATE_READY || huart1.gState == HAL_UART_STATE_BUSY_RX)
    {
        HAL_UART_Transmit_DMA(&huart1, uart1_tx_buf, len);
    }
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
    uart1_printf("[U1 DIAG] armed baud=%lu word=%lu parity=%lu stop=%lu rxbuf=%d\r\n",
                 (unsigned long)huart1.Init.BaudRate,
                 (unsigned long)huart1.Init.WordLength,
                 (unsigned long)huart1.Init.Parity,
                 (unsigned long)huart1.Init.StopBits,
                 USART1_DMA_BUF_NUM);
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
    // 1. 强制停止 DMA 接收，防止在处理期间被覆盖
    HAL_UART_DMAStop(&huart1);
    
    static uint8_t this_time_rx_len = 0;
    this_time_rx_len = USART1_DMA_BUF_NUM - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    uart1_diag.last_len = this_time_rx_len;
    
    // 2. 如果且仅如果长度正好是18字节，说明收到了一帧完整的 DBUS 数据
    if(this_time_rx_len == 18)
    {
        uart1_diag.ok18_count++;
        for(uint8_t i = 0; i < 6; i++)
        {
            uart1_diag.last_head[i] = usart1_dma_rx_buffer[i];
        }

        // 先把裸数据打印到串口，确认物理链路（切忌如果在外部循环打印，可能会印出过期或全0的缓冲区）
        if(uart1_diag.first_ok_reported == 0)
        {
            static char debug_str[128];
            int pos = 0;
            pos += sprintf(debug_str + pos, "[U1 OK] len=%d raw: ", this_time_rx_len);
            for(uint8_t i = 0; i < 18; i++) {
                pos += sprintf(debug_str + pos, "%02X ", usart1_dma_rx_buffer[i]);
            }
            sprintf(debug_str + pos, "\r\n");
            uart1_printf("%s", debug_str);
            uart1_diag.first_ok_reported = 1;
        }

        // 3. 数据解包与控制逻辑
        memcpy(sbus_rx_buf[0], usart1_dma_rx_buffer, 18);
        sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);

        if (RC_data_is_error())
        {
            uart1_diag.rc_data_err_count++;
            uart1_printf("[U1 RC_ERR] ch=%d,%d,%d,%d s=%d,%d\r\n",
                         rc_ctrl.rc.ch[0], rc_ctrl.rc.ch[1], rc_ctrl.rc.ch[2], rc_ctrl.rc.ch[3],
                         rc_ctrl.rc.s[0], rc_ctrl.rc.s[1]);
        }
        
        // 维持安全看门狗状态
        detect_hook(DBUS_TOE);
        
        // 防止未对齐或错误数据导致拨杆状态不合法、强行修正以免触发离线死锁
        if (rc_ctrl.rc.s[0] != RC_SW_UP && rc_ctrl.rc.s[0] != RC_SW_MID && rc_ctrl.rc.s[0] != RC_SW_DOWN) {
            rc_ctrl.rc.s[0] = RC_SW_UP;
        }
        if (rc_ctrl.rc.s[1] != RC_SW_UP && rc_ctrl.rc.s[1] != RC_SW_MID && rc_ctrl.rc.s[1] != RC_SW_DOWN) {
            rc_ctrl.rc.s[1] = RC_SW_UP;
        }

        HP = get_robot_remain_HP();
        send_uint16_t(HP);
    }
    else if(this_time_rx_len > 0)
    {
        uart1_diag.len_err_count++;
        // 长度不对（断帧/粘包），仅仅打印提示，不作解包，防止将错误数据送入底盘
        uart1_printf("[U1 LEN_ERR] len=%d (expect 18), head=%02X %02X %02X %02X %02X %02X\r\n",
                     this_time_rx_len,
                     usart1_dma_rx_buffer[0], usart1_dma_rx_buffer[1], usart1_dma_rx_buffer[2],
                     usart1_dma_rx_buffer[3], usart1_dma_rx_buffer[4], usart1_dma_rx_buffer[5]);
    }

    {
        uint32_t now = HAL_GetTick();
        if ((now - uart1_diag.last_report_tick) >= 1000U)
        {
            uart1_diag.last_report_tick = now;
            uart1_printf("[U1 SUM] idle=%lu ok18=%lu len_err=%lu rc_err=%lu FE=%lu NE=%lu ORE=%lu PE=%lu last_len=%u\r\n",
                         (unsigned long)uart1_diag.idle_irq_count,
                         (unsigned long)uart1_diag.ok18_count,
                         (unsigned long)uart1_diag.len_err_count,
                         (unsigned long)uart1_diag.rc_data_err_count,
                         (unsigned long)uart1_diag.fe_count,
                         (unsigned long)uart1_diag.ne_count,
                         (unsigned long)uart1_diag.ore_count,
                         (unsigned long)uart1_diag.pe_count,
                         uart1_diag.last_len);
        }
    }

    // [致命漏洞修复]：严禁在此处执行 memset(usart1_dma_rx_buffer, 0, USART1_DMA_BUF_NUM);
    // 因为这会强制覆盖 DMA 缓冲区。如果下次中断还没来得及收满数据，内存就被你写死了全为 0。

    // 4. 重启 DMA 接收，准备收下一帧
    HAL_UART_Receive_DMA(&huart1, usart1_dma_rx_buffer, USART1_DMA_BUF_NUM);
    
    // 5. 关闭半满中断 (HT)，减少无用的 CPU 干扰
    hdma_usart1_rx.Instance->CR &= ~(1U << 3);
}

void USART1_IRQHandler(void)
{
    uint32_t sr = huart1.Instance->SR;

    if (sr & (1U << 1))
    {
        uart1_diag.fe_count++;
    }
    if (sr & (1U << 2))
    {
        uart1_diag.ne_count++;
    }
    if (sr & (1U << 3))
    {
        uart1_diag.ore_count++;
    }
    if (sr & (1U << 0))
    {
        uart1_diag.pe_count++;
    }

    if (sr & (1U << 4))
    {
        uart1_diag.idle_irq_count++;
        (void)huart1.Instance->SR;
        (void)huart1.Instance->DR;
        Usart1Receive_IDLE();
        return;
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
    uint8_t buffer[2];  // �洢 uint16_t �ĸ��ֽן͵��ֽ�

    // �� uint16_t ��������ת��Ϊ�ֽ�����
    buffer[0] = (uint8_t)(data >> 8);  // ���ֽ�
    buffer[1] = (uint8_t)(data & 0xFF); // ���ֽ�

    // ͨ�� UART �����ֽ�����
    HAL_UART_Transmit(&huart1, buffer, 2, HAL_MAX_DELAY);  // ���� 2 ���ֽ�
}
