/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculate by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ����������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================
    add a gimbal behaviour mode
    1. in gimbal_behaviour.h , add a new behaviour name in gimbal_behaviour_e
    erum
    {  
        ...
        ...
        GIMBAL_XXX_XXX, // new add
    }gimbal_behaviour_e,
    2. implement new function. gimbal_xxx_xxx_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
        "yaw, pitch" param is gimbal movement contorl input. 
        first param: 'yaw' usually means  yaw axis move,usaully means increment angle.
            positive value means counterclockwise move, negative value means clockwise move.
        second param: 'pitch' usually means pitch axis move,usaully means increment angle.
            positive value means counterclockwise move, negative value means clockwise move.

        in this new function, you can assign set-point to "yaw" and "pitch",as your wish
    3.  in "gimbal_behavour_set" function, add new logical judgement to assign GIMBAL_XXX_XXX to  "gimbal_behaviour" variable,
        and in the last of the "gimbal_behaviour_mode_set" function, add "else if(gimbal_behaviour == GIMBAL_XXX_XXX)" 
        choose a gimbal control mode.
        four mode:
        GIMBAL_MOTOR_RAW : will use 'yaw' and 'pitch' as motor current set,  derectly sent to can bus.
        GIMBAL_MOTOR_ENCONDE : 'yaw' and 'pitch' are angle increment,  control enconde relative angle.
        GIMBAL_MOTOR_GYRO : 'yaw' and 'pitch' are angle increment,  control gyro absolute angle.
    4. in the last of "gimbal_behaviour_control_set" function, add
        else if(gimbal_behaviour == GIMBAL_XXX_XXX)
        {
            gimbal_xxx_xxx_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
        }

        
    ���Ҫ����һ���µ���Ϊģʽ
    1.���ȣ���gimbal_behaviour.h�ļ��У� ����һ������Ϊ������ gimbal_behaviour_e
    erum
    {  
        ...
        ...
        GIMBAL_XXX_XXX, // �����ӵ�
    }gimbal_behaviour_e,

    2. ʵ��һ���µĺ��� gimbal_xxx_xxx_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
        "yaw, pitch" ��������̨�˶�����������
        ��һ������: 'yaw' ͨ������yaw���ƶ�,ͨ���ǽǶ�����,��ֵ����ʱ���˶�,��ֵ��˳ʱ��
        �ڶ�������: 'pitch' ͨ������pitch���ƶ�,ͨ���ǽǶ�����,��ֵ����ʱ���˶�,��ֵ��˳ʱ��
        ������µĺ���, ���ܸ� "yaw"��"pitch"��ֵ��Ҫ�Ĳ���
    3.  ��"gimbal_behavour_set"��������У������µ��߼��жϣ���gimbal_behaviour��ֵ��GIMBAL_XXX_XXX
        ��gimbal_behaviour_mode_set�����������"else if(gimbal_behaviour == GIMBAL_XXX_XXX)" ,Ȼ��ѡ��һ����̨����ģʽ
        3��:
        GIMBAL_MOTOR_RAW : ʹ��'yaw' and 'pitch' ��Ϊ��������趨ֵ,ֱ�ӷ��͵�CAN������.
        GIMBAL_MOTOR_ENCONDE : 'yaw' and 'pitch' �ǽǶ�����,  ���Ʊ�����ԽǶ�.
        GIMBAL_MOTOR_GYRO : 'yaw' and 'pitch' �ǽǶ�����,  ���������Ǿ��ԽǶ�.
    4.  ��"gimbal_behaviour_control_set" �������������
        else if(gimbal_behaviour == GIMBAL_XXX_XXX)
        {
            gimbal_xxx_xxx_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
#include "detect_task.h"

#include "user_lib.h"

//when gimbal is in calibrating, set buzzer frequency and strenght
//����̨��У׼, ���÷�����Ƶ�ʺ�ǿ��
#define gimbal_warn_buzzer_on() buzzer_on(31, 20000)
#define gimbal_warn_buzzer_off() buzzer_off()

/* ROS coordinate sign compensation.
 * Set to -1.0f on the axis whose positive direction is opposite to the
 * STM32 INS convention.  Change one axis at a time and verify with the
 * dbg_* variables in gimbal_task.c before touching the other axis.
 * Interface layer only — do NOT change YAW_TURN / PITCH_TURN at the same time.
 */
#define ROS_YAW_SIGN    1.0f
#define ROS_PITCH_SIGN  1.0f


/**
  * @brief          judge if gimbal reaches the limit by gyro
  * @param          gyro: rotation speed unit rad/s
  * @param          timing time, input "GIMBAL_CALI_STEP_TIME"
  * @param          record angle, unit rad
  * @param          feedback angle, unit rad
  * @param          record ecd, unit raw
  * @param          feedback ecd, unit raw
  * @param          cali step, +1 by one step
  */
/**
  * @brief          ͨ���жϽ��ٶ����ж���̨�Ƿ񵽴Ｋ��λ��
  * @param          ��Ӧ��Ľ��ٶȣ���λrad/s
  * @param          ��ʱʱ�䣬����GIMBAL_CALI_STEP_TIME��ʱ������
  * @param          ��¼�ĽǶ� rad
  * @param          �����ĽǶ� rad
  * @param          ��¼�ı���ֵ raw
  * @param          �����ı���ֵ raw
  * @param          У׼�Ĳ��� ���һ�� ��һ
  */
#define gimbal_cali_gyro_judge(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }

/**
  * @brief          gimbal behave mode set.
  * @param[in]      gimbal_mode_set: gimbal data
  * @retval         none
  */
/**
  * @brief          ��̨��Ϊ״̬������.
  * @param[in]      gimbal_mode_set: ��̨����ָ��
  * @retval         none
  */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_ZERO_FORCE, the function is called
  *                 and gimbal control mode is raw. The raw mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all zero.
  * @param[out]     yaw: yaw motor current set, it will be sent to CAN bus derectly.
  * @param[out]     pitch: pitch motor current set, it will be sent to CAN bus derectly.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          ����̨��Ϊģʽ��GIMBAL_ZERO_FORCE, ��������ᱻ����,��̨����ģʽ��rawģʽ.ԭʼģʽ��ζ��
  *                 �趨ֵ��ֱ�ӷ��͵�CAN������,�������������������Ϊ0.
  * @param[in]      yaw:����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      pitch:����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      gimbal_control_set: ��̨����ָ��
  * @retval         none
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_INIT, the function is called
  *                 and gimbal control mode is gyro mode. gimbal will lift the pitch axis
  *                 and rotate yaw axis.
  * @param[out]     yaw: yaw motor relative angle increment, unit rad.
  * @param[out]     pitch: pitch motor absolute angle increment, unit rad.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          ��̨��ʼ�����ƣ�����������ǽǶȿ��ƣ���̨��̧��pitch�ᣬ����תyaw��
  * @param[out]     yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[out]     pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_CALI, the function is called
  *                 and gimbal control mode is raw mode. gimbal will lift the pitch axis, 
  *                 and then put down the pitch axis, and rotate yaw axis counterclockwise,
  *                 and rotate yaw axis clockwise.
  * @param[out]     yaw: yaw motor current set, will be sent to CAN bus decretly
  * @param[out]     pitch: pitch motor current set, will be sent to CAN bus decretly
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          ��̨У׼���ƣ������raw���ƣ���̨��̧��pitch������pitch������תyaw�����תyaw����¼��ʱ�ĽǶȺͱ���ֵ
  * @author         RM
  * @param[out]     yaw:����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[out]     pitch:����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      gimbal_control_set:��̨����ָ��
  * @retval         none
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_ABSOLUTE_ANGLE, the function is called
  *                 and gimbal control mode is gyro mode. 
  * @param[out]     yaw: yaw axia absolute angle increment, unit rad
  * @param[out]     pitch: pitch axia absolute angle increment,unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          ��̨�����ǿ��ƣ�����������ǽǶȿ��ƣ�
  * @param[out]     yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[out]     pitch:pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      gimbal_control_set:��̨����ָ��
  * @retval         none
  */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_RELATIVE_ANGLE, the function is called
  *                 and gimbal control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment, unit rad
  * @param[out]     pitch: pitch axia relative angle increment,unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          ��̨����ֵ���ƣ��������ԽǶȿ��ƣ�
  * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      gimbal_control_set: ��̨����ָ��
  * @retval         none
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);






static void gimbal_ros_absolute_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behaviour mode is GIMBAL_MOTIONLESS, the function is called
  *                 and gimbal control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment,  unit rad
  * @param[out]     pitch: pitch axia relative angle increment, unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          ��̨����ң������������ƣ��������ԽǶȿ��ƣ�
  * @author         RM
  * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      gimbal_control_set:��̨����ָ��
  * @retval         none
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

//��̨��Ϊ״̬��
static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

/**
  * @brief          the function is called by gimbal_set_mode function in gimbal_task.c
  *                 the function set gimbal_behaviour variable, and set motor mode.
  * @param[in]      gimbal_mode_set: gimbal data
  * @retval         none
  */
/**
  * @brief          ��gimbal_set_mode����������gimbal_task.c,��̨��Ϊ״̬���Լ����״̬������
  * @param[out]     gimbal_mode_set: ��̨����ָ��
  * @retval         none
  */

void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //set gimbal_behaviour variable
    //��̨��Ϊ״̬������
    gimbal_behavour_set(gimbal_mode_set);

    //accoring to gimbal_behaviour, set motor control mode
    //������̨��Ϊ״̬�����õ��״̬��
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
		//˫��̨���
//		gimbal_mode_set->gimbal_dual_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
//        gimbal_mode_set->gimbal_dual_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
		//˫��̨���
//		gimbal_mode_set->gimbal_dual_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
//        gimbal_mode_set->gimbal_dual_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_CALI)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
		//˫��̨���
//		gimbal_mode_set->gimbal_dual_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
//        gimbal_mode_set->gimbal_dual_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
		//˫��̨���
//		gimbal_mode_set->gimbal_dual_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
//        gimbal_mode_set->gimbal_dual_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
		//˫��̨���
//		gimbal_mode_set->gimbal_dual_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
//        gimbal_mode_set->gimbal_dual_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
		
    }
    else if (gimbal_behaviour == GIMBAL_ROS_ABSOLUTE)
    {
      gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
      gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
		//˫��̨���
//		gimbal_mode_set->gimbal_dual_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
//        gimbal_mode_set->gimbal_dual_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
}

/**
  * @brief          the function is called by gimbal_set_contorl function in gimbal_task.c
  *                 accoring to the gimbal_behaviour variable, call the corresponding function
  * @param[out]     add_yaw:yaw axis increment angle, unit rad
  * @param[out]     add_pitch:pitch axis increment angle,unit rad
  * @param[in]      gimbal_mode_set: gimbal data
  * @retval         none
  */
/**
  * @brief          ��̨��Ϊ���ƣ����ݲ�ͬ��Ϊ���ò�ͬ���ƺ���
  * @param[out]     add_yaw:���õ�yaw�Ƕ�����ֵ����λ rad
  * @param[out]     add_pitch:���õ�pitch�Ƕ�����ֵ����λ rad
  * @param[in]      gimbal_mode_set:��̨����ָ��
  * @retval         none
  */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }


    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_init_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_CALI)
    {
        gimbal_cali_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_ROS_ABSOLUTE)
    {
        gimbal_ros_absolute_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
    }

}

/**
  * @brief          in some gimbal mode, need chassis keep no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
/**
  * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���̲���
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_MOTIONLESS || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          in some gimbal mode, need shoot keep no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
/**
  * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���ֹͣ
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
  * @brief          gimbal behave mode set.
  * @param[in]      gimbal_mode_set: gimbal data
  * @retval         none
  */
/**
  * @brief          ��̨��Ϊ״̬������.
  * @param[in]      gimbal_mode_set: ��̨����ָ��
  * @retval         none
  */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //in cali mode, return
    if (gimbal_behaviour == GIMBAL_CALI && gimbal_mode_set->gimbal_cali.step != GIMBAL_CALI_END_STEP)
    {
        return;
    }
    if (gimbal_mode_set->gimbal_cali.step == GIMBAL_CALI_START_STEP)
    {
        gimbal_behaviour = GIMBAL_CALI;
        return;
    }

    //init mode, judge if gimbal is in middle place
    if (gimbal_behaviour == GIMBAL_INIT)
    {
        static uint16_t init_time = 0;
        static uint16_t init_stop_time = 0;
        init_time++;

        if ((fabs(gimbal_mode_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
             fabs(gimbal_mode_set->gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR))
        {
            if (init_stop_time < GIMBAL_INIT_STOP_TIME)
            {
                init_stop_time++;
            }
        }
        else
        {
            if (init_time < GIMBAL_INIT_TIME)
            {
                init_time++;
            }
        }

        /* exit init when: timeout OR settled at center */
        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME)
        {
            return;
        }
        else
        {
            init_stop_time = 0;
            init_time = 0;
        }
    }

    /* ===== ROS autonomous mode selection (no recognition dependency) ===== */
    if (gimbal_mode_set->ros_nav_cmd != NULL &&
             (gimbal_mode_set->ros_nav_cmd->nav_ctrl_flags & NAV_FLAG_GIMBAL_ABS_VALID))
    {
        gimbal_behaviour = GIMBAL_ROS_ABSOLUTE;
    }
    /* no command: hold current position */
    else
    {
        gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
    }

    // ROS-only control: never force INIT sweep automatically.
}

/**
  * @brief          when gimbal behaviour mode is GIMBAL_ZERO_FORCE, the function is called
  *                 and gimbal control mode is raw. The raw mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all zero.
  * @param[out]     yaw: yaw motor current set, it will be sent to CAN bus derectly.
  * @param[out]     pitch: pitch motor current set, it will be sent to CAN bus derectly.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          ����̨��Ϊģʽ��GIMBAL_ZERO_FORCE, ��������ᱻ����,��̨����ģʽ��rawģʽ.ԭʼģʽ��ζ��
  *                 �趨ֵ��ֱ�ӷ��͵�CAN������,�������������������Ϊ0.
  * @param[in]      yaw:����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      pitch:����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      gimbal_control_set: ��̨����ָ��
  * @retval         none
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL )
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}
/**
  * @brief          when gimbal behaviour mode is GIMBAL_INIT, the function is called
  *                 and gimbal control mode is gyro mode. gimbal will lift the pitch axis
  *                 and rotate yaw axis.
  * @param[out]     yaw: yaw motor relative angle increment, unit rad.
  * @param[out]     pitch: pitch motor absolute angle increment, unit rad.
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          ��̨��ʼ�����ƣ�����������ǽǶȿ��ƣ���̨��̧��pitch�ᣬ����תyaw��
  * @author         RM
  * @param[out]     yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[out]     pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      ��̨����ָ��
  * @retval         ���ؿ�
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    //��ʼ��״̬����������
    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    }
}

/**
  * @brief          when gimbal behaviour mode is GIMBAL_CALI, the function is called
  *                 and gimbal control mode is raw mode. gimbal will lift the pitch axis, 
  *                 and then put down the pitch axis, and rotate yaw axis counterclockwise,
  *                 and rotate yaw axis clockwise.
  * @param[out]     yaw: yaw motor current set, will be sent to CAN bus decretly
  * @param[out]     pitch: pitch motor current set, will be sent to CAN bus decretly
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          ��̨У׼���ƣ������raw���ƣ���̨��̧��pitch������pitch������תyaw�����תyaw����¼��ʱ�ĽǶȺͱ���ֵ
  * @author         RM
  * @param[out]     yaw:����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[out]     pitch:����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
  * @param[in]      gimbal_control_set:��̨����ָ��
  * @retval         none
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static uint16_t cali_time = 0;

    if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MAX_STEP)
    {

        *pitch = GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        //�ж����������ݣ� ����¼�����С�Ƕ�����
        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MIN_STEP)
    {
        *pitch = -GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MAX_STEP)
    {
        *pitch = 0;
        *yaw = GIMBAL_CALI_MOTOR_SET;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }

    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MIN_STEP)
    {
        *pitch = 0;
        *yaw = -GIMBAL_CALI_MOTOR_SET;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        cali_time = 0;
    }
}


/**
  * @brief          when gimbal behaviour mode is GIMBAL_ABSOLUTE_ANGLE, the function is called
  *                 and gimbal control mode is gyro mode. 
  * @param[out]     yaw: yaw axia absolute angle increment, unit rad
  * @param[out]     pitch: pitch axia absolute angle increment,unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          ��̨�����ǿ��ƣ�����������ǽǶȿ��ƣ�
  * @param[out]     yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[out]     pitch:pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      gimbal_control_set:��̨����ָ��
  * @retval         none
  */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    /* ROS autonomous: no RC/keyboard input */
    *yaw = 0.0f;
    *pitch = 0.0f;
}


/**
  * @brief          when gimbal behaviour mode is GIMBAL_RELATIVE_ANGLE, the function is called
  *                 and gimbal control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment, unit rad
  * @param[out]     pitch: pitch axia relative angle increment,unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          ��̨����ֵ���ƣ��������ԽǶȿ��ƣ�
  * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      gimbal_control_set: ��̨����ָ��
  * @retval         none
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    /* ROS autonomous: hold current position, no RC input */
    *yaw = 0.0f;
    *pitch = 0.0f;
}



/**
  * @brief          ��̨����ң������������ƣ��������ԽǶȿ��ƣ�
  * @author         RM
  * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      gimbal_control_set:��̨����ָ��
  * @retval         none
  */
/**
  * @brief          when gimbal behaviour mode is GIMBAL_MOTIONLESS, the function is called
  *                 and gimbal control mode is encode mode. 
  * @param[out]     yaw: yaw axia relative angle increment,  unit rad
  * @param[out]     pitch: pitch axia relative angle increment, unit rad
  * @param[in]      gimbal_control_set: gimbal data
  * @retval         none
  */
/**
  * @brief          ��̨����ң������������ƣ��������ԽǶȿ��ƣ�
  * @author         RM
  * @param[in]      yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      pitch: pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      gimbal_control_set:��̨����ָ��
  * @retval         none
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
  * @brief          ROS absolute angle control — compute incremental yaw/pitch
  *                 from the difference between ROS target and current encoder angle
  */
static void gimbal_ros_absolute_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    const ros_nav_cmd_t *nav = gimbal_control_set->ros_nav_cmd;
    if (nav == NULL || !(nav->nav_ctrl_flags & NAV_FLAG_GIMBAL_ABS_VALID))
    {
        *yaw = 0.0f;
        *pitch = 0.0f;
        return;
    }

    /*
     * ROS gives absolute yaw/pitch targets.
     * In GIMBAL_MOTOR_GYRO mode, add_yaw/add_pitch are angle increments
     * applied to absolute_angle_set every control cycle.
     *
     * Therefore we directly drive setpoint to target using the delta between
     * target and current absolute setpoint (not feedback), so each new frame
     * can refresh target deterministically instead of relying on incremental
     * drift in ENCODE mode.
     */
    const fp32 target_yaw   = rad_format(ROS_YAW_SIGN   * nav->yaw_abs);
    const fp32 target_pitch = rad_format(ROS_PITCH_SIGN * nav->pitch_abs);
    *yaw   = rad_format(target_yaw   - gimbal_control_set->gimbal_yaw_motor.absolute_angle_set);
    *pitch = rad_format(target_pitch - gimbal_control_set->gimbal_pitch_motor.absolute_angle_set);
}
