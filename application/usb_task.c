/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usb���������Ϣ
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "usb_task.h"

#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "detect_task.h"
#include "voltage_task.h"
#include "CAN_receive.h"


static void usb_printf(const char *fmt,...);

static uint8_t usb_buf[512];
static const char status[2][7] = {"OK", "ERROR!"};
const error_t *error_list_usb_local;



void usb_task(void const * argument)
{
    MX_USB_DEVICE_Init();
    error_list_usb_local = get_error_list_point();


    while(1)
    {
  can_debug_info_t can_dbg = get_can_debug_info();
        osDelay(1000);
        usb_printf(
"******************************\r\n\
voltage percentage:%d%% \r\n\
DBUS:%s\r\n\
chassis motor1:%s\r\n\
chassis motor2:%s\r\n\
chassis motor3:%s\r\n\
chassis motor4:%s\r\n\
yaw motor:%s\r\n\
pitch motor:%s\r\n\
trigger motor:%s\r\n\
gyro sensor:%s\r\n\
accel sensor:%s\r\n\
mag sensor:%s\r\n\
referee usart:%s\r\n\
CAN RX total:%lu\r\n\
CAN RX can1:%lu\r\n\
CAN RX can2:%lu\r\n\
CAN RX last id:0x%03X\r\n\
CAN TX chassis cmd:%lu\r\n\
CAN TX ok:%lu\r\n\
CAN TX fail:%lu\r\n\
CAN1 err:0x%08lX free_mb:%lu\r\n\
CAN2 err:0x%08lX free_mb:%lu\r\n\
******************************\r\n",
            get_battery_percentage(), 
            status[error_list_usb_local[DBUS_TOE].error_exist],
            status[error_list_usb_local[CHASSIS_MOTOR1_TOE].error_exist],
            status[error_list_usb_local[CHASSIS_MOTOR2_TOE].error_exist],
            status[error_list_usb_local[CHASSIS_MOTOR3_TOE].error_exist],
            status[error_list_usb_local[CHASSIS_MOTOR4_TOE].error_exist],
            status[error_list_usb_local[YAW_GIMBAL_MOTOR_TOE].error_exist],
            status[error_list_usb_local[PITCH_GIMBAL_MOTOR_TOE].error_exist],
            status[error_list_usb_local[TRIGGER_MOTOR_TOE].error_exist],
//			status[error_list_usb_local[YAW_GIMBAL_DUAL_MOTOR_TOE].error_exist],
//            status[error_list_usb_local[PITCH_GIMBAL_DUAL_MOTOR_TOE].error_exist],
            status[error_list_usb_local[BOARD_GYRO_TOE].error_exist],
            status[error_list_usb_local[BOARD_ACCEL_TOE].error_exist],
            status[error_list_usb_local[BOARD_MAG_TOE].error_exist],
                status[error_list_usb_local[REFEREE_TOE].error_exist],
                (unsigned long)can_dbg.rx_total_count,
                (unsigned long)can_dbg.rx_can1_count,
                (unsigned long)can_dbg.rx_can2_count,
                can_dbg.last_rx_std_id,
                (unsigned long)can_dbg.tx_chassis_cmd_count,
                (unsigned long)can_dbg.tx_ok_count,
                (unsigned long)can_dbg.tx_fail_count,
                (unsigned long)can_dbg.can1_error_code,
                (unsigned long)can_dbg.can1_tx_free_level,
                (unsigned long)can_dbg.can2_error_code,
                (unsigned long)can_dbg.can2_tx_free_level);

    }

}

static void usb_printf(const char *fmt,...)
{
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);

    len = vsprintf((char *)usb_buf, fmt, ap);

    va_end(ap);


    CDC_Transmit_FS(usb_buf, len);
}
