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

uint8_t usb_buf[256];
const error_t *error_list_usb_local;



void usb_task(void const * argument)
{
    MX_USB_DEVICE_Init();
    error_list_usb_local = get_error_list_point();


    while(1)
    {
        osDelay(1000);
        // [???????????] ??????? usb_printf ??? usb_buf ???????????????
        /*
        usb_printf(
"******************************\r\n\
voltage percentage:%d%% \r\n\
DBUS:%s\r\n\
...
******************************\r\n", ... );
        */
    }

}

void usb_printf(const char *fmt,...)
{
    va_list ap;
    int len = 0;

    va_start(ap, fmt);

    len = vsnprintf((char *)usb_buf, sizeof(usb_buf), fmt, ap);

    va_end(ap);

    if (len <= 0)
    {
        return;
    }

    if (len > (int)sizeof(usb_buf))
    {
        len = sizeof(usb_buf);
    }

    CDC_Transmit_FS(usb_buf, (uint16_t)len);
}
