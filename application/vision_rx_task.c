/**
  ******************************************************************************
  * @FileName       vision_rx_task.c
  * @Description    ROS<->STM32 UART1 communication — state-machine parser
  * @author         Xiao TY
  ******************************************************************************
  *
  * Copyright (c) 2024 Yangtze University
  * All rights reserved.
  *
  ******************************************************************************
**/

#include "vision_rx_task.h"

#include "main.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"

#include "fifo.h"
#include "detect_task.h"
#include "referee.h"
#include "gimbal_task.h"

/* ---------- extern hardware handles ---------- */
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef  hdma_usart1_rx;

/* ---------- DMA buffer ---------- */
static uint8_t vision_dma_rx_buf[VISION_RX_BUF_NUM];

/* ---------- FIFO ---------- */
static fifo_s_t  vision_fifo;
static uint8_t   vision_fifo_buf[VISION_FIFO_BUF_LENGTH];

/* ---------- data instances ---------- */
static ros_aim_data_t ros_aim_data = {0};
static ros_nav_cmd_t  ros_nav_cmd  = {0};

/* ---------- diagnostics ---------- */
typedef struct
{
    uint32_t idle_irq_count;
    uint32_t frame_ok_count;
    uint32_t crc_err_count;
    uint32_t len_err_count;
    uint32_t fe_count;
    uint32_t ne_count;
    uint32_t ore_count;
    uint32_t pe_count;
    uint32_t last_report_tick;
} uart1_diag_t;

static uart1_diag_t diag = {0};

/* ================================================================
 *  CRC8 — XOR over CMD_ID + DATA_LEN + PAYLOAD
 * ================================================================ */
static uint8_t ros_crc8(const uint8_t *buf, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= buf[i];
    }
    return crc;
}

/* ================================================================
 *  uart1_dma_send — non-blocking DMA transmit helper
 * ================================================================ */
static void uart1_dma_send(const uint8_t *buf, uint16_t len)
{
    if (huart1.gState == HAL_UART_STATE_READY ||
        huart1.gState == HAL_UART_STATE_BUSY_RX)
    {
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len);
    }
}

/* ================================================================
 *  ros_send_status_report — CMD_STATUS_REPORT (0x81), 14 bytes payload
 * ================================================================ */
static void ros_send_status_report(void)
{
    /* frame: SOF_H SOF_L CMD LEN [14 payload] CRC = 19 bytes */
    static uint8_t tx[19];

    const gimbal_motor_t *yaw_motor   = get_yaw_motor_point();
    const gimbal_motor_t *pitch_motor = get_pitch_motor_point();

    uint16_t hp      = get_robot_remain_HP();
    float yaw_abs    = (yaw_motor != NULL) ? yaw_motor->absolute_angle : 0.0f;
    float pitch_abs  = (pitch_motor != NULL) ? pitch_motor->absolute_angle : 0.0f;
    uint8_t robot_id = get_robot_id();

    tx[0] = ROS_SOF_H;
    tx[1] = ROS_SOF_L;
    tx[2] = CMD_STATUS_REPORT;
    tx[3] = 14;

    /* payload at [4..17] */
    tx[4] = (uint8_t)(hp & 0xFF);
    tx[5] = (uint8_t)(hp >> 8);
    memcpy(&tx[6],  &yaw_abs,   4);
    memcpy(&tx[10], &pitch_abs,  4);
    tx[14] = robot_id;
    tx[15] = 0;  /* shoot_speed_limit */
    tx[16] = 0;  /* reserved */
    tx[17] = 0;

    /* CRC8 over CMD + LEN + PAYLOAD = tx[2..17] */
    tx[18] = ros_crc8(&tx[2], 16);

    uart1_dma_send(tx, 19);
}

/* ================================================================
 *  State machine parser — processes bytes from FIFO
 * ================================================================ */
typedef enum
{
    PARSE_WAIT_SOF_H = 0,
    PARSE_WAIT_SOF_L,
    PARSE_WAIT_CMD,
    PARSE_WAIT_LEN,
    PARSE_RECV_DATA,
    PARSE_WAIT_CRC,
} parse_state_e;

static parse_state_e parse_state = PARSE_WAIT_SOF_H;
static uint8_t  parse_cmd    = 0;
static uint8_t  parse_len    = 0;
static uint8_t  parse_idx    = 0;
static uint8_t  parse_buf[ROS_FRAME_MAX_PAYLOAD];

static void ros_handle_frame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    if (cmd == CMD_AIM_DATA && len == 14)
    {
        memcpy(&ros_aim_data.yaw_offset,   &payload[0], 4);
        memcpy(&ros_aim_data.pitch_offset, &payload[4], 4);

        uint8_t flags = payload[8];
        ros_aim_data.target_valid  = (flags & 0x01U) ? 1U : 0U;
        ros_aim_data.shoot_suggest = (flags & 0x02U) ? 1U : 0U;

        ros_aim_data.distance_mm = (uint16_t)(payload[9] | ((uint16_t)payload[10] << 8));
        ros_aim_data.target_id   = payload[11];

        detect_hook(VISION_TOE);
        diag.frame_ok_count++;
    }
    else if (cmd == CMD_HEARTBEAT && len == 0)
    {
        detect_hook(VISION_TOE);
        diag.frame_ok_count++;
    }
    else if (cmd == CMD_NAV_DATA && len == 21)
    {
        memcpy(&ros_nav_cmd.vx,        &payload[0],  4);
        memcpy(&ros_nav_cmd.vy,        &payload[4],  4);
        memcpy(&ros_nav_cmd.vz,        &payload[8],  4);
        memcpy(&ros_nav_cmd.yaw_abs,   &payload[12], 4);
        memcpy(&ros_nav_cmd.pitch_abs, &payload[16], 4);
        ros_nav_cmd.nav_ctrl_flags = payload[20];

        detect_hook(VISION_TOE);
        diag.frame_ok_count++;
    }
}

static void ros_parse_fifo(void)
{
    uint8_t byte;

    while (fifo_s_used(&vision_fifo) > 0)
    {
        fifo_s_gets(&vision_fifo, (char *)&byte, 1);

        switch (parse_state)
        {
        case PARSE_WAIT_SOF_H:
            if (byte == ROS_SOF_H)
                parse_state = PARSE_WAIT_SOF_L;
            break;

        case PARSE_WAIT_SOF_L:
            if (byte == ROS_SOF_L)
                parse_state = PARSE_WAIT_CMD;
            else
                parse_state = (byte == ROS_SOF_H) ? PARSE_WAIT_SOF_L : PARSE_WAIT_SOF_H;
            break;

        case PARSE_WAIT_CMD:
            parse_cmd = byte;
            parse_state = PARSE_WAIT_LEN;
            break;

        case PARSE_WAIT_LEN:
            parse_len = byte;
            parse_idx = 0;
            if (parse_len > ROS_FRAME_MAX_PAYLOAD)
            {
                diag.len_err_count++;
                parse_state = PARSE_WAIT_SOF_H;
            }
            else if (parse_len == 0)
            {
                parse_state = PARSE_WAIT_CRC;
            }
            else
            {
                parse_state = PARSE_RECV_DATA;
            }
            break;

        case PARSE_RECV_DATA:
            parse_buf[parse_idx++] = byte;
            if (parse_idx >= parse_len)
                parse_state = PARSE_WAIT_CRC;
            break;

        case PARSE_WAIT_CRC:
        {
            /* compute expected CRC: XOR of CMD + LEN + PAYLOAD */
            uint8_t crc = parse_cmd ^ parse_len;
            for (uint8_t i = 0; i < parse_len; i++)
                crc ^= parse_buf[i];

            if (crc == byte)
            {
                ros_handle_frame(parse_cmd, parse_buf, parse_len);
            }
            else
            {
                diag.crc_err_count++;
            }
            parse_state = PARSE_WAIT_SOF_H;
            break;
        }

        default:
            parse_state = PARSE_WAIT_SOF_H;
            break;
        }
    }
}

/* ================================================================
 *  get_vision_data_point — accessor for downstream consumers
 * ================================================================ */
const ros_aim_data_t *get_vision_data_point(void)
{
    return &ros_aim_data;
}

/* ================================================================
 *  get_ros_nav_cmd_point — accessor for chassis/gimbal consumers
 * ================================================================ */
const ros_nav_cmd_t *get_ros_nav_cmd_point(void)
{
    return &ros_nav_cmd;
}

/* ================================================================
 *  USART1_IRQHandler — IDLE line → push raw bytes into FIFO
 * ================================================================ */
void USART1_IRQHandler(void)
{
    uint32_t sr = huart1.Instance->SR;

    if (sr & (1U << 1)) diag.fe_count++;
    if (sr & (1U << 2)) diag.ne_count++;
    if (sr & (1U << 3)) diag.ore_count++;
    if (sr & (1U << 0)) diag.pe_count++;

    if (sr & (1U << 4))
    {
        diag.idle_irq_count++;
        (void)huart1.Instance->SR;
        (void)huart1.Instance->DR;

        HAL_UART_DMAStop(&huart1);
        uint16_t rx_len = VISION_RX_BUF_NUM
                        - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

        if (rx_len > 0)
        {
            fifo_s_puts_noprotect(&vision_fifo,
                                  (char *)vision_dma_rx_buf, rx_len);
        }

        HAL_UART_Receive_DMA(&huart1, vision_dma_rx_buf, VISION_RX_BUF_NUM);
        hdma_usart1_rx.Instance->CR &= ~(1U << 3);
        return;
    }

    HAL_UART_IRQHandler(&huart1);
}

/* ================================================================
 *  vision_rx_task — RTOS task entry point
 * ================================================================ */
void vision_rx_task(void const *argument)
{
    fifo_s_init(&vision_fifo, vision_fifo_buf, VISION_FIFO_BUF_LENGTH);

    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, vision_dma_rx_buf, VISION_RX_BUF_NUM);

    uint32_t last_tx_tick = 0;

    while (1)
    {
        /* parse all available bytes */
        ros_parse_fifo();

        /* send status report every 50ms */
        uint32_t now = HAL_GetTick();
        if ((now - last_tx_tick) >= 50U)
        {
            last_tx_tick = now;
            ros_send_status_report();
        }

        osDelay(1);
    }
}
