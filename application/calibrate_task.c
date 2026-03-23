/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       calibrate_task.c/h
  * @brief      calibrate these device��include gimbal, gyro, accel, magnetometer,
  *             chassis. gimbal calibration is to calc the midpoint, max/min 
  *             relative angle. gyro calibration is to calc the zero drift.
  *             accel and mag calibration have not been implemented yet, because
  *             accel is not necessary to calibrate, mag is not used. chassis 
  *             calibration is to make motor 3508 enter quick reset ID mode.
  *             У׼�豸��������̨,������,���ٶȼ�,������,����.��̨У׼����Ҫ�������
  *             �������С��ԽǶ�.������У׼����Ҫ������Ư.���ٶȼƺʹ�����У׼��û��ʵ��
  *             ��Ϊ���ٶȼƻ�û�б�ҪȥУ׼,�������ƻ�û����.����У׼��ʹM3508�������
  *             ����IDģʽ.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-25-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis clabration
  *
  @verbatim
  ==============================================================================
  *             use the remote control begin calibrate,
  *             first: two switchs of remote control are down
  *             second:hold for 2 seconds, two rockers set to V, like \../;  \. means the letf rocker go bottom right.
  *             third:hold for 2 seconds, two rockers set to ./\., begin the gyro calibration
  *                     or set to '\/', begin the gimbal calibration
  *                     or set to /''\, begin the chassis calibration
  *
  *             data in flash, include cali data and name[3] and cali_flag
  *             for example, head_cali has 8 bytes, and it need 12 bytes in flash. if it starts in 0x080A0000
  *             0x080A0000-0x080A0007: head_cali data
  *             0x080A0008: name[0]
  *             0x080A0009: name[1]
  *             0x080A000A: name[2]
  *             0x080A000B: cali_flag, when cali_flag == 0x55, means head_cali has been calibrated.
  *             if add a sensor
  *             1.add cail sensro name in cali_id_e at calibrate_task.h, like
  *             typedef enum
  *             {
  *                 ...
  *                 //add more...
  *                 CALI_XXX,
  *                 CALI_LIST_LENGHT,
  *             } cali_id_e;
  *             2. add the new data struct in calibrate_task.h, must be 4 four-byte mulitple  like
  *
  *             typedef struct
  *             {
  *                 uint16_t xxx;
  *                 uint16_t yyy;
  *                 fp32 zzz;
  *             } xxx_cali_t; //size: 8 bytes, must be 4, 8, 12, 16...
  *             3.in "FLASH_WRITE_BUF_LENGHT", add "sizeof(xxx_cali_t)", and implement new function.
  *             bool_t cali_xxx_hook(uint32_t *cali, bool_t cmd), and add the name in "cali_name[CALI_LIST_LENGHT][3]"
  *             and declare variable xxx_cali_t xxx_cail, add the data address in cali_sensor_buf[CALI_LIST_LENGHT]
  *             and add the data lenght in cali_sensor_size, at last, add function in cali_hook_fun[CALI_LIST_LENGHT]
  *             ʹ��ң�������п�ʼУ׼
  *             ��һ��:ң�������������ض�����
  *             �ڶ���:����ҡ�˴��\../,��������.\.������ҡ�������´�.
  *             ������:ҡ�˴��./\. ��ʼ������У׼
  *                    ����ҡ�˴��'\/' ��ʼ��̨У׼
  *                    ����ҡ�˴��/''\ ��ʼ����У׼
  *
  *             ������flash�У�����У׼���ݺ����� name[3] �� У׼��־λ cali_flag
  *             ����head_cali�а˸��ֽ�,������Ҫ12�ֽ���flash,�������0x080A0000��ʼ
  *             0x080A0000-0x080A0007: head_cali����
  *             0x080A0008: ����name[0]
  *             0x080A0009: ����name[1]
  *             0x080A000A: ����name[2]
  *             0x080A000B: У׼��־λ cali_flag,��У׼��־λΪ0x55,��ζ��head_cali�Ѿ�У׼��
  *             �������豸
  *             1.�����豸����calibrate_task.h��cali_id_e, ��
  *             typedef enum
  *             {
  *                 ...
  *                 //add more...
  *                 CALI_XXX,
  *                 CALI_LIST_LENGHT,
  *             } cali_id_e;
  *             2. �������ݽṹ�� calibrate_task.h, ����4�ֽڱ�������
  *
  *             typedef struct
  *             {
  *                 uint16_t xxx;
  *                 uint16_t yyy;
  *                 fp32 zzz;
  *             } xxx_cali_t; //����:8�ֽ� 8 bytes, ������ 4, 8, 12, 16...
  *             3.�� "FLASH_WRITE_BUF_LENGHT",����"sizeof(xxx_cali_t)", ��ʵ���º���
  *             bool_t cali_xxx_hook(uint32_t *cali, bool_t cmd), ������������ "cali_name[CALI_LIST_LENGHT][3]"
  *             ���������� xxx_cali_t xxx_cail, ���ӱ�����ַ��cali_sensor_buf[CALI_LIST_LENGHT]
  *             ��cali_sensor_size[CALI_LIST_LENGHT]�������ݳ���, �����cali_hook_fun[CALI_LIST_LENGHT]���Ӻ���
  *
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "calibrate_task.h"
#include "string.h"
#include "cmsis_os.h"

#include "bsp_adc.h"
#include "bsp_buzzer.h"
#include "bsp_flash.h"

#include "can_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "gimbal_task.h"
#include "vision_rx_task.h"


//include head,gimbal,gyro,accel,mag. gyro,accel and mag have the same data struct. total 5(CALI_LIST_LENGHT) devices, need data lenght + 5 * 4 bytes(name[3]+cali)
#define FLASH_WRITE_BUF_LENGHT  (sizeof(head_cali_t) + sizeof(gimbal_cali_t) + sizeof(imu_cali_t) * 3  + CALI_LIST_LENGHT * 4)




/**
  * @brief          use remote control to begin a calibrate,such as gyro, gimbal, chassis
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ʹ��ң������ʼУ׼�����������ǣ���̨������
  * @param[in]      none
  * @retval         none
  */
static void RC_cmd_to_calibrate(void);

/**
  * @brief          read cali data from flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ��flash��ȡУ׼����
  * @param[in]      none
  * @retval         none
  */
static void cali_data_read(void);

/**
  * @brief          write the data to flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ��flashд��У׼����
  * @param[in]      none
  * @retval         none
  */
static bool_t cali_data_write(void);

/**
  * @brief          verify calibration data already written in flash matches the
  *                 expected buffer.
  * @param[in]      none
  * @retval         1: flash contents match expected calibration buffer
  *                 0: flash contents do not match expected calibration buffer
  */
static bool_t cali_data_verify(void);

/**
  * @brief          validate gimbal calibration payload loaded from flash.
  * @param[in]      cali: gimbal calibration data pointer
  * @retval         1: valid
  *                 0: invalid
  */
static bool_t gimbal_flash_cali_data_valid(const gimbal_cali_t *cali);

/**
  * @brief          judge whether a dependency has provided data recently enough
  *                 for calibration startup.
  * @param[in]      toe: detect_task index
  * @param[in]      now: current tick
  * @param[in]      max_age: max allowed age in ticks/ms
  * @retval         1: recent data exists 0: no recent data
  */
static bool_t cali_dependency_recent(uint8_t toe, uint32_t now, uint32_t max_age);

static uint32_t cali_dependency_age_ms(uint8_t toe, uint32_t now);
static void update_gimbal_cali_debug_snapshot(uint32_t now, uint32_t ready_tick, uint32_t active_max_age, uint32_t active_stable_time);

/**
  * @brief          automatically request gimbal calibration once after boot when
  *                 no valid gimbal calibration data exists in flash.
  * @param[in]      none
  * @retval         none
  */
static void auto_start_gimbal_calibrate(void);


/**
  * @brief          "head" sensor cali function
  * @param[in][out] cali:the point to head data. when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          "head"�豸У׼
  * @param[in][out] cali:ָ��ָ��head����,��cmdΪCALI_FUNC_CMD_INIT, ����������,CALI_FUNC_CMD_ON,���������
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: ������У׼���ݳ�ʼ��ԭʼ����
                    CALI_FUNC_CMD_ON: ������ҪУ׼
  * @retval         0:У׼����û����
                    1:У׼�����Ѿ����
  */
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd);   //header device cali function

/**
  * @brief          gyro cali function
  * @param[in][out] cali:the point to gyro data, when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          �������豸У׼
  * @param[in][out] cali:ָ��ָ������������,��cmdΪCALI_FUNC_CMD_INIT, ����������,CALI_FUNC_CMD_ON,���������
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: ������У׼���ݳ�ʼ��ԭʼ����
                    CALI_FUNC_CMD_ON: ������ҪУ׼
  * @retval         0:У׼����û����
                    1:У׼�����Ѿ����
  */
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd);   //gyro device cali function

/**
  * @brief          gimbal cali function
  * @param[in][out] cali:the point to gimbal data, when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          ��̨�豸У׼
  * @param[in][out] cali:ָ��ָ����̨����,��cmdΪCALI_FUNC_CMD_INIT, ����������,CALI_FUNC_CMD_ON,���������
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: ������У׼���ݳ�ʼ��ԭʼ����
                    CALI_FUNC_CMD_ON: ������ҪУ׼
  * @retval         0:У׼����û����
                    1:У׼�����Ѿ����
  */
static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd); //gimbal device cali function



#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t calibrate_task_stack;
#endif


static head_cali_t     head_cali;       //head cali data
static gimbal_cali_t   gimbal_cali;     //gimbal cali data
static imu_cali_t      accel_cali;      //accel cali data
static imu_cali_t      gyro_cali;       //gyro cali data
static imu_cali_t      mag_cali;        //mag cali data


static uint8_t flash_write_buf[FLASH_WRITE_BUF_LENGHT];
static uint8_t flash_read_verify_buf[FLASH_WRITE_BUF_LENGHT];

static bool_t gimbal_auto_cali_pending = 0;
static bool_t gimbal_host_cali_pending = 0;
static bool_t cali_data_dirty = 0;

volatile uint32_t dbg_gimbal_cali_request_count = 0;
volatile uint32_t dbg_gimbal_cali_last_req_tick = 0;
volatile uint32_t dbg_gimbal_cali_now_tick = 0;
volatile uint32_t dbg_gimbal_cali_ready_tick = 0;
volatile uint32_t dbg_gimbal_cali_ready_elapsed_ms = 0;
volatile uint32_t dbg_gimbal_cali_yaw_age_ms = 0xFFFFFFFFU;
volatile uint32_t dbg_gimbal_cali_pitch_age_ms = 0xFFFFFFFFU;
volatile uint32_t dbg_gimbal_cali_imu_age_ms = 0xFFFFFFFFU;
volatile uint32_t dbg_gimbal_cali_dependency_max_age_ms = AUTO_GIMBAL_CALI_DEPENDENCY_MAX_AGE;
volatile uint32_t dbg_gimbal_cali_stable_time_ms = AUTO_GIMBAL_CALI_READY_STABLE_TIME;
volatile uint8_t dbg_gimbal_cali_gate_state = DBG_GIMBAL_CALI_GATE_IDLE;
volatile uint8_t dbg_gimbal_cali_host_pending = 0;
volatile uint8_t dbg_gimbal_cali_auto_pending = 0;
volatile uint8_t dbg_gimbal_cali_pending = 0;
volatile uint8_t dbg_gimbal_cali_running = 0;
volatile uint8_t dbg_gimbal_cali_valid = 0;
volatile uint8_t dbg_gimbal_cali_cmd = 0;
volatile uint8_t dbg_gimbal_cali_block_nav_fresh = 0;
volatile uint8_t dbg_gimbal_cali_yaw_recent = 0;
volatile uint8_t dbg_gimbal_cali_pitch_recent = 0;
volatile uint8_t dbg_gimbal_cali_imu_recent = 0;

cali_sensor_t cali_sensor[CALI_LIST_LENGHT]; 

static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"HD", "GM", "GYR", "ACC", "MAG"};

//cali data address
static uint32_t *cali_sensor_buf[CALI_LIST_LENGHT] = {
        (uint32_t *)&head_cali, (uint32_t *)&gimbal_cali,
        (uint32_t *)&gyro_cali, (uint32_t *)&accel_cali,
        (uint32_t *)&mag_cali};


static uint8_t cali_sensor_size[CALI_LIST_LENGHT] =
    {
        sizeof(head_cali_t) / 4, sizeof(gimbal_cali_t) / 4,
        sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4};

void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_head_hook, cali_gimbal_hook, cali_gyro_hook, NULL, NULL};

/**
  * @brief          calibrate task, created by main function
  * @param[in]      pvParameters: null
  * @retval         none
  */
/**
  * @brief          У׼������main��������
  * @param[in]      pvParameters: ��
  * @retval         none
  */
void calibrate_task(void const *pvParameters)
{
    static uint8_t i = 0;

    while (1)
    {

    if (cali_data_dirty)
    {
      if (cali_data_write())
      {
        cali_data_dirty = 0;
      }
    }

        RC_cmd_to_calibrate();
      auto_start_gimbal_calibrate();

        for (i = 0; i < CALI_LIST_LENGHT; i++)
        {
            if (cali_sensor[i].cali_cmd)
            {
                if (cali_sensor[i].cali_hook != NULL)
                {

                    if (cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_ON))
                    {
                        //done
                        cali_sensor[i].name[0] = cali_name[i][0];
                        cali_sensor[i].name[1] = cali_name[i][1];
                        cali_sensor[i].name[2] = cali_name[i][2];
                        //set 0x55
                        cali_sensor[i].cali_done = CALIED_FLAG;

                        cali_sensor[i].cali_cmd = 0;
                        cali_data_dirty = 1;
                        //write immediately and keep retrying in task loop until verified
                        if (cali_data_write())
                        {
                          cali_data_dirty = 0;
                        }
                    }
                }
            }
        }
        osDelay(CALIBRATE_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        calibrate_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

  void request_gimbal_calibration(void)
  {
    dbg_gimbal_cali_request_count++;
    dbg_gimbal_cali_last_req_tick = xTaskGetTickCount();
    gimbal_host_cali_pending = 1;
    cali_sensor[CALI_GIMBAL].cali_done = 0;
    cali_sensor[CALI_GIMBAL].cali_cmd = 0;
    dbg_gimbal_cali_gate_state = DBG_GIMBAL_CALI_GATE_WAIT_NAV_CLEAR;
  }

  uint8_t gimbal_calibration_is_valid(void)
  {
    return (cali_sensor[CALI_GIMBAL].cali_done == CALIED_FLAG) ? 1U : 0U;
  }

  uint8_t gimbal_calibration_is_running(void)
  {
    return (cali_sensor[CALI_GIMBAL].cali_cmd != 0U) ? 1U : 0U;
  }

  uint8_t gimbal_calibration_is_pending(void)
  {
    return ((gimbal_auto_cali_pending != 0U) || (gimbal_host_cali_pending != 0U)) ? 1U : 0U;
  }

/**
  * @brief          get imu control temperature, unit ��
  * @param[in]      none
  * @retval         imu control temperature
  */
/**
  * @brief          ��ȡimu�����¶�, ��λ��
  * @param[in]      none
  * @retval         imu�����¶�
  */
int8_t get_control_temperature(void)
{

    return head_cali.temperature;
}

/**
  * @brief          get latitude, default 22.0f
  * @param[out]     latitude: the point to fp32 
  * @retval         none
  */
/**
  * @brief          ��ȡγ��,Ĭ��22.0f
  * @param[out]     latitude:fp32ָ�� 
  * @retval         none
  */
void get_flash_latitude(float *latitude)
{

    if (latitude == NULL)
    {

        return;
    }
    if (cali_sensor[CALI_HEAD].cali_done == CALIED_FLAG)
    {
        *latitude = head_cali.latitude;
    }
    else
    {
        *latitude = 22.0f;
    }
}

/**
  * @brief          use remote control to begin a calibrate,such as gyro, gimbal, chassis
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ʹ��ң������ʼУ׼�����������ǣ���̨������
  * @param[in]      none
  * @retval         none
  */
static void RC_cmd_to_calibrate(void)
{
  // RC-based calibration trigger has been removed.
}

/**
  * @brief          use remote control to begin a calibrate,such as gyro, gimbal, chassis
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ʹ��ң������ʼУ׼�����������ǣ���̨������
  * @param[in]      none
  * @retval         none
  */
void cali_param_init(void)
{
    uint8_t i = 0;

  gimbal_auto_cali_pending = 0;
  gimbal_host_cali_pending = 0;
  cali_data_dirty = 0;

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        cali_sensor[i].flash_len = cali_sensor_size[i];
        cali_sensor[i].flash_buf = cali_sensor_buf[i];
        cali_sensor[i].cali_hook = (bool_t(*)(uint32_t *, bool_t))cali_hook_fun[i];
    }

    cali_data_read();

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        if (cali_sensor[i].cali_done == CALIED_FLAG)
        {
            if (cali_sensor[i].cali_hook != NULL)
            {
                //if has been calibrated, set to init 
                cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_INIT);
            }
        }
    }
}

/**
  * @brief          read cali data from flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ��flash��ȡУ׼����
  * @param[in]      none
  * @retval         none
  */
static void cali_data_read(void)
{
    uint8_t flash_read_buf[CALI_SENSOR_HEAD_LEGHT * 4];
    uint8_t i = 0;
    uint16_t offset = 0;
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {

        //read the data in flash, 
        cali_flash_read(FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);
        
        offset += cali_sensor[i].flash_len * 4;

        //read the name and cali flag,
        cali_flash_read(FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_SENSOR_HEAD_LEGHT);
        
        cali_sensor[i].name[0] = flash_read_buf[0];
        cali_sensor[i].name[1] = flash_read_buf[1];
        cali_sensor[i].name[2] = flash_read_buf[2];
        cali_sensor[i].cali_done = flash_read_buf[3];

        if (i == CALI_GIMBAL && cali_sensor[i].cali_done == CALIED_FLAG)
        {
          if (!gimbal_flash_cali_data_valid((const gimbal_cali_t *)cali_sensor[i].flash_buf))
          {
            cali_sensor[i].cali_done = 0;
          }
        }
        
        offset += CALI_SENSOR_HEAD_LEGHT * 4;

        if (cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook != NULL)
        {
          if (i == CALI_GIMBAL)
          {
              cali_sensor[i].cali_cmd = 0;
#if AUTO_GIMBAL_CALI_ENABLE
              gimbal_auto_cali_pending = 1;
#else
              gimbal_auto_cali_pending = 0;
#endif
          }
          else
          {
              cali_sensor[i].cali_cmd = 1;
          }
        }
    }
}

static void auto_start_gimbal_calibrate(void)
{
  static uint32_t gimbal_ready_tick = 0;
  const uint32_t now = xTaskGetTickCount();
  const uint32_t active_max_age = gimbal_host_cali_pending ? HOST_GIMBAL_CALI_DEPENDENCY_MAX_AGE : AUTO_GIMBAL_CALI_DEPENDENCY_MAX_AGE;
  const uint32_t active_stable_time = gimbal_host_cali_pending ? HOST_GIMBAL_CALI_READY_STABLE_TIME : AUTO_GIMBAL_CALI_READY_STABLE_TIME;

  update_gimbal_cali_debug_snapshot(now, gimbal_ready_tick, active_max_age, active_stable_time);

  if (!gimbal_auto_cali_pending && !gimbal_host_cali_pending)
  {
    dbg_gimbal_cali_gate_state = gimbal_calibration_is_valid() ? DBG_GIMBAL_CALI_GATE_VALID_READY : DBG_GIMBAL_CALI_GATE_IDLE;
    update_gimbal_cali_debug_snapshot(now, gimbal_ready_tick, active_max_age, active_stable_time);
    return;
  }

  if (cali_sensor[CALI_GIMBAL].cali_done == CALIED_FLAG)
  {
    gimbal_auto_cali_pending = 0;
    gimbal_host_cali_pending = 0;
    gimbal_ready_tick = 0;
    dbg_gimbal_cali_gate_state = DBG_GIMBAL_CALI_GATE_VALID_READY;
    update_gimbal_cali_debug_snapshot(now, gimbal_ready_tick, active_max_age, active_stable_time);
    return;
  }

  if (cali_sensor[CALI_GIMBAL].cali_cmd)
  {
    dbg_gimbal_cali_gate_state = DBG_GIMBAL_CALI_GATE_RUNNING;
    update_gimbal_cali_debug_snapshot(now, gimbal_ready_tick, active_max_age, active_stable_time);
    return;
  }

#if AUTO_GIMBAL_CALI_START_DELAY > 0
  if (now < AUTO_GIMBAL_CALI_START_DELAY)
  {
    dbg_gimbal_cali_gate_state = DBG_GIMBAL_CALI_GATE_WAIT_START_DELAY;
    update_gimbal_cali_debug_snapshot(now, gimbal_ready_tick, active_max_age, active_stable_time);
    return;
  }
#endif

  if (ros_nav_cmd_fresh())
  {
    gimbal_ready_tick = 0;
    dbg_gimbal_cali_gate_state = DBG_GIMBAL_CALI_GATE_WAIT_NAV_CLEAR;
    update_gimbal_cali_debug_snapshot(now, gimbal_ready_tick, active_max_age, active_stable_time);
    return;
  }

  if (!cali_dependency_recent(YAW_GIMBAL_MOTOR_TOE, now, active_max_age))
  {
    gimbal_ready_tick = 0;
    dbg_gimbal_cali_gate_state = DBG_GIMBAL_CALI_GATE_WAIT_YAW_FEEDBACK;
    update_gimbal_cali_debug_snapshot(now, gimbal_ready_tick, active_max_age, active_stable_time);
    return;
  }

  if (!cali_dependency_recent(PITCH_GIMBAL_MOTOR_TOE, now, active_max_age))
  {
    gimbal_ready_tick = 0;
    dbg_gimbal_cali_gate_state = DBG_GIMBAL_CALI_GATE_WAIT_PITCH_FEEDBACK;
    update_gimbal_cali_debug_snapshot(now, gimbal_ready_tick, active_max_age, active_stable_time);
    return;
  }

  if (!cali_dependency_recent(RM_IMU_TOE, now, active_max_age))
  {
    gimbal_ready_tick = 0;
    dbg_gimbal_cali_gate_state = DBG_GIMBAL_CALI_GATE_WAIT_IMU_FEEDBACK;
    update_gimbal_cali_debug_snapshot(now, gimbal_ready_tick, active_max_age, active_stable_time);
    return;
  }

  if (gimbal_ready_tick == 0)
  {
    gimbal_ready_tick = now;
    dbg_gimbal_cali_gate_state = DBG_GIMBAL_CALI_GATE_WAIT_STABLE_WINDOW;
    update_gimbal_cali_debug_snapshot(now, gimbal_ready_tick, active_max_age, active_stable_time);
    return;
  }

  if ((now - gimbal_ready_tick) < active_stable_time)
  {
    dbg_gimbal_cali_gate_state = DBG_GIMBAL_CALI_GATE_WAIT_STABLE_WINDOW;
    update_gimbal_cali_debug_snapshot(now, gimbal_ready_tick, active_max_age, active_stable_time);
    return;
  }

  cali_sensor[CALI_GIMBAL].cali_cmd = 1;
  gimbal_auto_cali_pending = 0;
  gimbal_host_cali_pending = 0;
  gimbal_ready_tick = 0;
  dbg_gimbal_cali_gate_state = DBG_GIMBAL_CALI_GATE_RUNNING;
  update_gimbal_cali_debug_snapshot(now, gimbal_ready_tick, active_max_age, active_stable_time);
}

static uint32_t cali_dependency_age_ms(uint8_t toe, uint32_t now)
{
  const error_t *error_list_point = get_error_list_point();

  if (error_list_point == NULL || toe >= ERROR_LIST_LENGHT)
  {
    return 0xFFFFFFFFU;
  }

  if (error_list_point[toe].enable == 0)
  {
    return 0xFFFFFFFEU;
  }

  if (!detect_has_real_update(toe))
  {
    return 0xFFFFFFFFU;
  }

  return now - error_list_point[toe].new_time;
}

static void update_gimbal_cali_debug_snapshot(uint32_t now, uint32_t ready_tick, uint32_t active_max_age, uint32_t active_stable_time)
{
  dbg_gimbal_cali_now_tick = now;
  dbg_gimbal_cali_ready_tick = ready_tick;
  dbg_gimbal_cali_ready_elapsed_ms = (ready_tick == 0U) ? 0U : (now - ready_tick);
  dbg_gimbal_cali_dependency_max_age_ms = active_max_age;
  dbg_gimbal_cali_stable_time_ms = active_stable_time;

  dbg_gimbal_cali_host_pending = (gimbal_host_cali_pending != 0U) ? 1U : 0U;
  dbg_gimbal_cali_auto_pending = (gimbal_auto_cali_pending != 0U) ? 1U : 0U;
  dbg_gimbal_cali_pending = ((gimbal_host_cali_pending != 0U) || (gimbal_auto_cali_pending != 0U)) ? 1U : 0U;
  dbg_gimbal_cali_cmd = (cali_sensor[CALI_GIMBAL].cali_cmd != 0U) ? 1U : 0U;
  dbg_gimbal_cali_running = dbg_gimbal_cali_cmd;
  dbg_gimbal_cali_valid = (cali_sensor[CALI_GIMBAL].cali_done == CALIED_FLAG) ? 1U : 0U;
  dbg_gimbal_cali_block_nav_fresh = ros_nav_cmd_fresh() ? 1U : 0U;

  dbg_gimbal_cali_yaw_age_ms = cali_dependency_age_ms(YAW_GIMBAL_MOTOR_TOE, now);
  dbg_gimbal_cali_pitch_age_ms = cali_dependency_age_ms(PITCH_GIMBAL_MOTOR_TOE, now);
  dbg_gimbal_cali_imu_age_ms = cali_dependency_age_ms(RM_IMU_TOE, now);

  dbg_gimbal_cali_yaw_recent = (dbg_gimbal_cali_yaw_age_ms <= active_max_age) ? 1U : 0U;
  dbg_gimbal_cali_pitch_recent = (dbg_gimbal_cali_pitch_age_ms <= active_max_age) ? 1U : 0U;
  dbg_gimbal_cali_imu_recent = (dbg_gimbal_cali_imu_age_ms <= active_max_age) ? 1U : 0U;
}

static bool_t cali_dependency_recent(uint8_t toe, uint32_t now, uint32_t max_age)
{
  const error_t *error_list_point = get_error_list_point();

  if (error_list_point == NULL || toe >= ERROR_LIST_LENGHT)
  {
    return 0;
  }

  if (error_list_point[toe].enable == 0)
  {
    return 0;
  }

  if (!detect_has_real_update(toe))
  {
    return 0;
  }

  return ((now - error_list_point[toe].new_time) <= max_age) ? 1 : 0;
}

static bool_t gimbal_flash_cali_data_valid(const gimbal_cali_t *cali)
{
  if (cali == NULL)
  {
    return 0;
  }

  if ((cali->yaw_max_angle - cali->yaw_min_angle) < 0.6f)
  {
    return 0;
  }

  if ((cali->pitch_max_angle - cali->pitch_min_angle) < 0.25f)
  {
    return 0;
  }

  return 1;
}


/**
  * @brief          write the data to flash
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ��flashд��У׼����
  * @param[in]      none
  * @retval         none
  */
static bool_t cali_data_verify(void)
{
  memset(flash_read_verify_buf, 0, sizeof(flash_read_verify_buf));
  cali_flash_read(FLASH_USER_ADDR, (uint32_t *)flash_read_verify_buf, (FLASH_WRITE_BUF_LENGHT + 3) / 4);

  if (memcmp(flash_write_buf, flash_read_verify_buf, FLASH_WRITE_BUF_LENGHT) == 0)
  {
    return 1;
  }

  return 0;
}

static bool_t cali_data_write(void)
{
    uint8_t i = 0;
  uint8_t write_retry = 0;
    uint16_t offset = 0;

  memset(flash_write_buf, 0, sizeof(flash_write_buf));

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        //copy the data of device calibration data
        memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].flash_buf, cali_sensor[i].flash_len * 4);
        offset += cali_sensor[i].flash_len * 4;

        //copy the name and "CALI_FLAG" of device
        memcpy((void *)(flash_write_buf + offset), (void *)cali_sensor[i].name, CALI_SENSOR_HEAD_LEGHT * 4);
        offset += CALI_SENSOR_HEAD_LEGHT * 4;
    }

    for (write_retry = 0; write_retry < 3; write_retry++)
    {
      //erase the page
      cali_flash_erase(FLASH_USER_ADDR,1);
      //write data and verify by readback
      if (cali_flash_write(FLASH_USER_ADDR, (uint32_t *)flash_write_buf, (FLASH_WRITE_BUF_LENGHT + 3) / 4) == 0)
      {
        if (cali_data_verify())
        {
          return 1;
        }
      }
    }

    return 0;
}


/**
  * @brief          "head" sensor cali function
  * @param[in][out] cali:the point to head data. when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          "head"�豸У׼
  * @param[in][out] cali:ָ��ָ��head����,��cmdΪCALI_FUNC_CMD_INIT, ����������,CALI_FUNC_CMD_ON,���������
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: ������У׼���ݳ�ʼ��ԭʼ����
                    CALI_FUNC_CMD_ON: ������ҪУ׼
  * @retval         0:У׼����û����
                    1:У׼�����Ѿ����
  */
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd)
{
    head_cali_t *local_cali_t = (head_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
//        memcpy(&head_cali, local_cali_t, sizeof(head_cali_t));

        return 1;
    }
    // self id
    local_cali_t->self_id = SELF_ID;
    //imu control temperature
    local_cali_t->temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
    //head_cali.temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
    if (local_cali_t->temperature > (int8_t)(GYRO_CONST_MAX_TEMP))
    {
        local_cali_t->temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
    }
    
    local_cali_t->firmware_version = FIRMWARE_VERSION;
    //shenzhen latitude 
    local_cali_t->latitude = 22.0f;

    return 1;
}

/**
  * @brief          gyro cali function
  * @param[in][out] cali:the point to gyro data, when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          �������豸У׼
  * @param[in][out] cali:ָ��ָ������������,��cmdΪCALI_FUNC_CMD_INIT, ����������,CALI_FUNC_CMD_ON,���������
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: ������У׼���ݳ�ʼ��ԭʼ����
                    CALI_FUNC_CMD_ON: ������ҪУ׼
  * @retval         0:У׼����û����
                    1:У׼�����Ѿ����
  */
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd)
{
    imu_cali_t *local_cali_t = (imu_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        gyro_set_cali(local_cali_t->scale, local_cali_t->offset);
        
        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        static uint16_t count_time = 0;
        gyro_cali_fun(local_cali_t->scale, local_cali_t->offset, &count_time);
        if (count_time > GYRO_CALIBRATE_TIME)
        {
            count_time = 0;
            cali_buzzer_off();
            gyro_cali_enable_control();
            return 1;
        }
        else
        {
            gyro_cali_disable_control(); //disable the remote control to make robot no move
            imu_start_buzzer();
            
            return 0;
        }
    }

    return 0;
}

/**
  * @brief          gimbal cali function
  * @param[in][out] cali:the point to gimbal data, when cmd == CALI_FUNC_CMD_INIT, param is [in],cmd == CALI_FUNC_CMD_ON, param is [out]
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: means to use cali data to initialize original data
                    CALI_FUNC_CMD_ON: means need to calibrate
  * @retval         0:means cali task has not been done
                    1:means cali task has been done
  */
/**
  * @brief          ��̨�豸У׼
  * @param[in][out] cali:ָ��ָ����̨����,��cmdΪCALI_FUNC_CMD_INIT, ����������,CALI_FUNC_CMD_ON,���������
  * @param[in]      cmd: 
                    CALI_FUNC_CMD_INIT: ������У׼���ݳ�ʼ��ԭʼ����
                    CALI_FUNC_CMD_ON: ������ҪУ׼
  * @retval         0:У׼����û����
                    1:У׼�����Ѿ����
  */
static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd)
{

    gimbal_cali_t *local_cali_t = (gimbal_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        set_cali_gimbal_hook(local_cali_t->yaw_offset, local_cali_t->pitch_offset,
                             local_cali_t->yaw_max_angle, local_cali_t->yaw_min_angle,
                             local_cali_t->pitch_max_angle, local_cali_t->pitch_min_angle);
        
        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        if (cmd_cali_gimbal_hook(&local_cali_t->yaw_offset, &local_cali_t->pitch_offset,
                                 &local_cali_t->yaw_max_angle, &local_cali_t->yaw_min_angle,
                                 &local_cali_t->pitch_max_angle, &local_cali_t->pitch_min_angle))
        {
            cali_buzzer_off();
          gimbal_host_cali_pending = 0;
            
            return 1;
        }
        else
        {
            gimbal_start_buzzer();
            
            return 0;
        }
    }
    
    return 0;
}

