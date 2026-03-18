/**
  ******************************************************************************
  * @FileName       auto_aim_task.h
  * @Description    Auto-aim control task — consumes vision data, outputs offsets
  * @author         Xiao TY
  ******************************************************************************
  *
  * Copyright (c) 2024 Yangtze University
  * All rights reserved.
  *
  ******************************************************************************
**/
#ifndef AUTO_AIM_TASK_H
#define AUTO_AIM_TASK_H

#include "struct_typedef.h"

typedef struct
{
    fp32    yaw_offset;
    fp32    pitch_offset;
    uint8_t aim_valid;
} auto_aim_output_t;

extern void auto_aim_task(void const *argument);
extern const auto_aim_output_t *get_auto_aim_output_point(void);

#endif /* AUTO_AIM_TASK_H */
