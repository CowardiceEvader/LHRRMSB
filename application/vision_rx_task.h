/**
  ******************************************************************************
  * @FileName       vision_rx_task.h
  * @Description    ROS<->STM32 UART1 binary communication protocol
  * @author         Xiao TY
  * @note           Replaces aim_assist_control.h — clean protocol, no SBUS
  ******************************************************************************
  *
  * Copyright (c) 2024 Yangtze University
  * All rights reserved.
  *
  ******************************************************************************
**/
#ifndef VISION_RX_TASK_H
#define VISION_RX_TASK_H

#include "struct_typedef.h"
#include "main.h"

/* ======================== protocol constants ======================== */
#define ROS_SOF_H               0xA5U
#define ROS_SOF_L               0x5AU

/* --- ROS -> STM32 (downstream) --- */
#define CMD_AIM_DATA            0x01U   /* 14 bytes payload */
#define CMD_NAV_DATA            0x02U   /* 21 bytes payload */
#define CMD_HEARTBEAT           0x10U   /*  0 bytes payload */

/* --- STM32 -> ROS (upstream) --- */
#define CMD_STATUS_REPORT       0x81U   /* 14 bytes payload */

#define ROS_FRAME_MAX_PAYLOAD   32U     /* max payload we accept */
#define ROS_FRAME_OVERHEAD      5U      /* SOF_H + SOF_L + CMD + LEN + CRC */

/* legacy compat */
#define VISION_RX_BUF_NUM       36U     /* DMA buffer size (>= max frame) */
#define VISION_FIFO_BUF_LENGTH  512U

#define YAW_ASSIST_SEN          0.0015f
#define PITCH_ASSIST_SEN        0.0015f

/* ======================== data structures ======================== */

/* float <-> bytes union */
typedef union
{
    float f;
    unsigned char c[4];
} float2uchar;

/* --- CMD_AIM_DATA (0x01) payload: 14 bytes --- */
typedef struct
{
    float       yaw_offset;         /* rad */
    float       pitch_offset;       /* rad */
    uint8_t     target_valid;       /* flags bit0 */
    uint8_t     shoot_suggest;      /* flags bit1 */
    uint16_t    distance_mm;        /* little-endian */
    uint8_t     target_id;
    uint8_t     reserved[3];
} ros_aim_data_t;

/* --- CMD_NAV_DATA (0x02) payload: 21 bytes --- */
#define NAV_FLAG_CHASSIS_VALID    0x01U
#define NAV_FLAG_GIMBAL_ABS_VALID 0x02U

typedef struct
{
    float    vx;            /* m/s, chassis forward/backward */
    float    vy;            /* m/s, chassis left/right */
    float    vz;            /* rad/s, chassis rotation */
    float    yaw_abs;       /* rad, gimbal yaw absolute angle */
    float    pitch_abs;     /* rad, gimbal pitch absolute angle */
    uint8_t  nav_ctrl_flags;/* bit0: chassis_valid, bit1: gimbal_abs_valid */
} ros_nav_cmd_t;

/* --- CMD_STATUS_REPORT (0x81) payload: 14 bytes --- */
typedef struct
{
    uint16_t    hp;
    float       yaw_abs;            /* rad */
    float       pitch_abs;          /* rad */
    uint8_t     robot_id;
    uint8_t     shoot_speed_limit;
    uint16_t    reserved;
} ros_status_report_t;

/* legacy alias — keeps downstream code (auto_aim_task, gimbal_task) compiling */
typedef ros_aim_data_t get_data_t;

/* ======================== public API ======================== */
extern void              vision_rx_task(void const *argument);
extern const ros_aim_data_t *get_vision_data_point(void);
extern const ros_nav_cmd_t  *get_ros_nav_cmd_point(void);

#endif /* VISION_RX_TASK_H */
