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
  *             �������С��ԽǶ�.��̨У׼����Ҫ������Ư.���ٶȼƺʹ�����У׼��û��ʵ��
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


#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H

#include "struct_typedef.h"

//when imu is calibrating ,buzzer set frequency and strength. ��imu��У׼,������������Ƶ�ʺ�ǿ��
#define imu_start_buzzer()          buzzer_on(95, 10000)    
//when gimbal is calibrating ,buzzer set frequency and strength.����̨��У׼,������������Ƶ�ʺ�ǿ��
#define gimbal_start_buzzer()       buzzer_on(31, 19999)    
#define cali_buzzer_off()           buzzer_off()            //buzzer off���رշ�����


//get stm32 chip temperature, to calc imu control temperature.��ȡstm32Ƭ���¶ȣ�����imu�Ŀ����¶�
#define cali_get_mcu_temperature()  get_temprate()      



#define cali_flash_read(address, buf, len)  flash_read((address), (buf), (len))                     //flash read function, flash ��ȡ����
#define cali_flash_write(address, buf, len) flash_write_single_address((address), (buf), (len))     //flash write function,flash д�뺯��
#define cali_flash_erase(address, page_num) flash_erase_address((address), (page_num))              //flash erase function,flash��������


#define get_remote_ctrl_point_cali()        (NULL)
#define gyro_cali_disable_control()         ((void)0)
#define gyro_cali_enable_control()          ((void)0)

// calc the zero drift function of gyro, ������������Ư
#define gyro_cali_fun(cali_scale, cali_offset, time_count)  INS_cali_gyro((cali_scale), (cali_offset), (time_count))
//set the zero drift to the INS task, ������INS task�ڵ���������Ư
#define gyro_set_cali(cali_scale, cali_offset)              INS_set_cali_gyro((cali_scale), (cali_offset))



#define FLASH_USER_ADDR         ADDR_FLASH_SECTOR_9 //write flash page 9,�����flashҳ��ַ

#define GYRO_CONST_MAX_TEMP     45.0f               //max control temperature of gyro,��������ǿ����¶�

#define CALI_FUNC_CMD_ON        1                   //need calibrate,����У׼
#define CALI_FUNC_CMD_INIT      0                   //has been calibrated, set value to init.�Ѿ�У׼��������У׼ֵ

#define CALIBRATE_CONTROL_TIME  1                   //osDelay time,  means 1ms.1ms ϵͳ��ʱ

#define CALI_SENSOR_HEAD_LEGHT  1

#define SELF_ID                 0                   //ID 
#define FIRMWARE_VERSION        12345               //handware version.
#define CALIED_FLAG             0x55                // means it has been calibrated
//you have 20 seconds to calibrate by remote control. ��20s������ң��������У׼
#define CALIBRATE_END_TIME          20000
//when 10 second, buzzer frequency change to high frequency of gimbal calibration.��10s��ʱ��,�������гɸ�Ƶ����
#define RC_CALI_BUZZER_MIDDLE_TIME  10000
//in the beginning, buzzer frequency change to low frequency of imu calibration.����ʼУ׼��ʱ��,�������гɵ�Ƶ����
#define RC_CALI_BUZZER_START_TIME   0


#define rc_cali_buzzer_middle_on()  gimbal_start_buzzer()
#define rc_cali_buzzer_start_on()   imu_start_buzzer()
#define RC_CMD_LONG_TIME            2000    

#define RCCALI_BUZZER_CYCLE_TIME    400        
#define RC_CALI_BUZZER_PAUSE_TIME   200       
#define RC_CALI_VALUE_HOLE          600     //remote control threshold, the max value of remote control channel is 660. 


#define GYRO_CALIBRATE_TIME         20000   //gyro calibrate time,������У׼ʱ��

// automatic gimbal calibration on boot: only used when gimbal flash data is invalid.
// startup is still gated by motor/IMU online checks and extra stable time.
#define AUTO_GIMBAL_CALI_ENABLE             1
#define AUTO_GIMBAL_CALI_START_DELAY        0
#define AUTO_GIMBAL_CALI_READY_STABLE_TIME  100
#define AUTO_GIMBAL_CALI_DEPENDENCY_MAX_AGE 20

// host-triggered re-calibration uses a looser gate than boot auto-start.
// reason: host requests are usually sent after the system is already stable,
// and a slightly wider recent window avoids getting stuck in pending due to
// brief feedback jitter or detect_task scheduling skew.
#define HOST_GIMBAL_CALI_READY_STABLE_TIME  20
#define HOST_GIMBAL_CALI_DEPENDENCY_MAX_AGE 100

// local watch/JScope gate-state values for gimbal calibration startup.
// these values are also mirrored into the UART status report for host-side debugging.
#define DBG_GIMBAL_CALI_GATE_IDLE                 0U
#define DBG_GIMBAL_CALI_GATE_WAIT_START_DELAY     1U
#define DBG_GIMBAL_CALI_GATE_WAIT_NAV_CLEAR       2U
#define DBG_GIMBAL_CALI_GATE_WAIT_YAW_FEEDBACK    3U
#define DBG_GIMBAL_CALI_GATE_WAIT_PITCH_FEEDBACK  4U
#define DBG_GIMBAL_CALI_GATE_WAIT_IMU_FEEDBACK    5U
#define DBG_GIMBAL_CALI_GATE_WAIT_STABLE_WINDOW   6U
#define DBG_GIMBAL_CALI_GATE_RUNNING              7U
#define DBG_GIMBAL_CALI_GATE_VALID_READY          8U

//cali device name
typedef enum
{
    CALI_HEAD = 0,
    CALI_GIMBAL = 1,
    CALI_GYRO = 2,
    CALI_ACC = 3,
    CALI_MAG = 4,
    //add more...
    CALI_LIST_LENGHT,
} cali_id_e;


typedef __packed struct
{
    uint8_t name[3];                                    //device name
    uint8_t cali_done;                                  //0x55 means has been calibrated
    uint8_t flash_len : 7;                              //buf lenght
    uint8_t cali_cmd : 1;                               //1 means to run cali hook function,
    uint32_t *flash_buf;                                //link to device calibration data
    bool_t (*cali_hook)(uint32_t *point, bool_t cmd);   //cali function
} cali_sensor_t;

//header device
typedef __packed struct
{
    uint8_t self_id;            // the "SELF_ID"
    uint16_t firmware_version;  // set to the "FIRMWARE_VERSION"
    //'temperature' and 'latitude' should not be in the head_cali, because don't want to create a new sensor
    //'temperature' and 'latitude'��Ӧ����head_cali,��Ϊ���봴��һ���µ��豸�ͷ�����
    int8_t temperature;         // imu control temperature
    fp32 latitude;              // latitude
} head_cali_t;
//gimbal device
typedef struct
{
    uint16_t yaw_offset;
    uint16_t pitch_offset;
    fp32 yaw_max_angle;
    fp32 yaw_min_angle;
    fp32 pitch_max_angle;
    fp32 pitch_min_angle;
	
//	uint16_t yaw_dual_offset;
//	uint16_t pitch_dual_offset;
//	fp32 yaw_dual_max_angle;
//    fp32 yaw_dual_min_angle;
//    fp32 pitch_dual_max_angle;
//    fp32 pitch_dual_min_angle;
	
} gimbal_cali_t;
//gyro, accel, mag device
typedef struct
{
    fp32 offset[3]; //x,y,z
    fp32 scale[3];  //x,y,z
} imu_cali_t;


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
extern void cali_param_init(void);
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
extern int8_t get_control_temperature(void);

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
extern void get_flash_latitude(float *latitude);

/**
  * @brief          request gimbal-only calibration.
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          请求启动仅云台校准。
  * @param[in]      无
  * @retval         无
  */
extern void request_gimbal_calibration(void);

/**
  * @brief          get current gimbal calibration validity.
  * @param[in]      none
  * @retval         1: valid 0: invalid
  */
/**
  * @brief          获取当前云台校准是否有效。
  * @param[in]      无
  * @retval         1:有效 0:无效
  */
extern uint8_t gimbal_calibration_is_valid(void);

/**
  * @brief          get whether gimbal calibration is running.
  * @param[in]      none
  * @retval         1: running 0: idle
  */
/**
  * @brief          获取当前云台校准是否正在运行。
  * @param[in]      无
  * @retval         1:运行中 0:空闲
  */
extern uint8_t gimbal_calibration_is_running(void);

/**
  * @brief          get whether gimbal calibration is pending.
  * @param[in]      none
  * @retval         1: pending 0: idle
  */
/**
  * @brief          获取当前云台校准是否处于等待状态。
  * @param[in]      无
  * @retval         1:等待中 0:空闲
  */
extern uint8_t gimbal_calibration_is_pending(void);

// local calibration gate debug variables.
// they can be observed in debugger/live watch and are mirrored into CMD_STATUS_REPORT.
extern volatile uint32_t dbg_gimbal_cali_request_count;
extern volatile uint32_t dbg_gimbal_cali_last_req_tick;
extern volatile uint32_t dbg_gimbal_cali_now_tick;
extern volatile uint32_t dbg_gimbal_cali_ready_tick;
extern volatile uint32_t dbg_gimbal_cali_ready_elapsed_ms;
extern volatile uint32_t dbg_gimbal_cali_yaw_age_ms;
extern volatile uint32_t dbg_gimbal_cali_pitch_age_ms;
extern volatile uint32_t dbg_gimbal_cali_imu_age_ms;
extern volatile uint32_t dbg_gimbal_cali_dependency_max_age_ms;
extern volatile uint32_t dbg_gimbal_cali_stable_time_ms;
extern volatile uint8_t dbg_gimbal_cali_gate_state;
extern volatile uint8_t dbg_gimbal_cali_host_pending;
extern volatile uint8_t dbg_gimbal_cali_auto_pending;
extern volatile uint8_t dbg_gimbal_cali_pending;
extern volatile uint8_t dbg_gimbal_cali_running;
extern volatile uint8_t dbg_gimbal_cali_valid;
extern volatile uint8_t dbg_gimbal_cali_cmd;
extern volatile uint8_t dbg_gimbal_cali_block_nav_fresh;
extern volatile uint8_t dbg_gimbal_cali_yaw_recent;
extern volatile uint8_t dbg_gimbal_cali_pitch_recent;
extern volatile uint8_t dbg_gimbal_cali_imu_recent;

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
extern void calibrate_task(void const *pvParameters);


#endif
