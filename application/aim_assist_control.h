/**
  ******************************************************************************
  * @FileName		    aim_assist_control.c/h
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
#ifndef AIM_ASSIST_CONTROL_H
#define AIM_ASSIST_CONTROL_H

#include "struct_typedef.h"
#include "main.h"
#include "remote_control.h"
#include "bsp_rc.h"


extern uint16_t yaw1,pitch1;


#define USART1_DMA_BUF_NUM 18
#define RX_DATA_LEN        18         //16
#define TX_DATA_LEN        14
#define YAW_ASSIST_SEN     0.0015f;
#define PITCH_ASSIST_SEN   0.0015f;


extern uint8_t  usart1_dma_rx_buffer[USART1_DMA_BUF_NUM];

typedef union
{
    float f;
    unsigned char c[4];
}float2uchar;

typedef struct
{
    float2uchar pitch_data;
    float2uchar yaw_data;
    unsigned char isThereArmor;
    unsigned char distance;
    unsigned char isShoot;
}get_data_t;

typedef struct
{
    float2uchar pitch_data;
    float2uchar yaw_data;
    uint8_t shoot_speed;
    uint8_t shoot_mode;
}send_data_t;

extern void Usart1Receive_IDLE(void);
extern void aim_assistant_control_init(void);
extern const get_data_t *get_aim_assistant_control_point(void);
extern get_data_t get_data;
extern void UART_SendString(UART_HandleTypeDef *huart, uint8_t *str);
extern void send_uint16_t(uint16_t data);
#endif
