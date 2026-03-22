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
#define CMD_NAV_DATA            0x02U   /* 27 bytes payload */
#define CMD_HEARTBEAT           0x10U   /*  0 bytes payload */

/* --- STM32 -> ROS (upstream) --- */
#define CMD_STATUS_REPORT       0x81U   /* 21 bytes payload */

#define ROS_FRAME_MAX_PAYLOAD   32U     /* max payload we accept */
#define ROS_FRAME_OVERHEAD      5U      /* SOF_H + SOF_L + CMD + LEN + CRC */
#define ROS_NAV_CMD_TIMEOUT_MS  100U    /* stale NAV_DATA timeout */

/* legacy compat */
#define VISION_RX_BUF_NUM       40U     /* DMA buffer size (>= max frame) */
#define VISION_FIFO_BUF_LENGTH  512U

/* ======================== data structures ======================== */

/* float <-> bytes union */
typedef union
{
    float f;
    unsigned char c[4];
} float2uchar;

/* --- CMD_NAV_DATA (0x02) payload: 27 bytes --- */
#define NAV_FLAG_CHASSIS_VALID    0x01U
#define NAV_FLAG_GIMBAL_ABS_VALID 0x02U
#define NAV_FLAG_FRIC_ON          0x04U   /* bit2: spin up friction wheels */
#define NAV_FLAG_SHOOT            0x08U   /* bit3: fire (single / continuous) */

#define ROS_STATUS_NAV_FRESH      0x01U
#define ROS_STATUS_VISION_ONLINE  0x02U
#define ROS_STATUS_REFEREE_ONLINE 0x04U
#define ROS_STATUS_HEAT_BLOCKED   0x08U
#define ROS_STATUS_GIMBAL_HOLD    0x10U

typedef struct
{
    float    vx;            /* m/s, chassis forward/backward */
    float    vy;            /* m/s, chassis left/right */
    float    vz;            /* rad/s, chassis rotation */
    float    yaw_abs;       /* rad, gimbal yaw absolute angle */
    float    pitch_abs;     /* rad, gimbal pitch absolute angle */
    uint8_t  nav_ctrl_flags;/* bit0: chassis_valid, bit1: gimbal_abs_valid */
    uint16_t seq;           /* upper-computer command sequence */
    uint32_t timestamp_ms;  /* upper-computer monotonic timestamp */
} ros_nav_cmd_t;

/* --- CMD_STATUS_REPORT (0x81) payload: 21 bytes --- */
typedef struct
{
    uint16_t    hp;
    float       yaw_abs;            /* rad */
    float       pitch_abs;          /* rad */
    uint8_t     robot_id;
    uint8_t     shoot_speed_limit;
    uint16_t    last_nav_seq;
    uint32_t    last_nav_timestamp_ms;
    uint16_t    nav_age_ms;
    uint8_t     status_flags;
} ros_status_report_t;

/* ======================== public API ======================== */
extern void              vision_rx_task(void const *argument);
extern const ros_nav_cmd_t  *get_ros_nav_cmd_point(void);
extern uint8_t           ros_nav_cmd_received(void);
extern uint8_t           ros_nav_cmd_fresh(void);
extern uint16_t          ros_nav_cmd_age_ms(void);

#endif /* VISION_RX_TASK_H */
