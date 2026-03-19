/**
  ******************************************************************************
  * @FileName       auto_aim_task.c
  * @Description    Auto-aim control task — reads vision data + rc_ctrl,
  *                 outputs yaw/pitch offsets for gimbal consumption
  * @author         Xiao TY
  ******************************************************************************
  *
  * Copyright (c) 2024 Yangtze University
  * All rights reserved.
  *
  ******************************************************************************
**/

#include "auto_aim_task.h"
#include "vision_rx_task.h"
#include "cmsis_os.h"
#include "string.h"

static auto_aim_output_t auto_aim_output = {0};

const auto_aim_output_t *get_auto_aim_output_point(void)
{
    return &auto_aim_output;
}

void auto_aim_task(void const *argument)
{
    const ros_aim_data_t *vision = get_vision_data_point();

    while (1)
    {
        if (vision->target_valid)
        {
            auto_aim_output.yaw_offset   = vision->yaw_offset;
            auto_aim_output.pitch_offset  = vision->pitch_offset;
            auto_aim_output.aim_valid     = 1;
        }
        else
        {
            auto_aim_output.yaw_offset   = 0.0f;
            auto_aim_output.pitch_offset  = 0.0f;
            auto_aim_output.aim_valid     = 0;
        }

        osDelay(1);
    }
}
