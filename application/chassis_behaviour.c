  /**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      according to remote control, change the chassis behaviour.
  *             ����ң������ֵ������������Ϊ��
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================
    add a chassis behaviour mode
    1. in chassis_behaviour.h , add a new behaviour name in chassis_behaviour
    erum
    {  
        ...
        ...
        CHASSIS_XXX_XXX, // new add
    }chassis_behaviour_e,
    2. implement new function. chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
        "vx, vy, wz" param is chassis movement contorl input. 
        first param: 'vx' usually means  vertical speed,
            positive value means forward speed, negative value means backward speed.
        second param: 'vy' usually means horizotal speed,
            positive value means letf speed, negative value means right speed
        third param: 'wz' can be rotation speed set or angle set, 

        in this new function, you can assign speed to "vx","vy",and "wz",as your wish
    3.  in "chassis_behaviour_mode_set" function, add new logical judgement to assign CHASSIS_XXX_XXX to  "chassis_behaviour_mode" variable,
        and in the last of the function, add "else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" 
        choose a chassis control mode.
        four mode:
        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy' are speed control, 'wz' is angle set to control relative angle
            between chassis and gimbal. you can name third param to 'xxx_angle_set' other than 'wz'
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy' are speed control, 'wz' is angle set to control absolute angle calculated by gyro
            you can name third param to 'xxx_angle_set.
        CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy' are speed control, 'wz' is rotation speed control.
        CHASSIS_VECTOR_RAW : will use 'vx' 'vy' and 'wz'  to linearly calculate four wheel current set, 
            current set will be derectly sent to can bus.
    4. in the last of "chassis_behaviour_control_set" function, add
        else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
        {
            chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
        }

        
    ���Ҫ����һ���µ���Ϊģʽ
    1.���ȣ���chassis_behaviour.h�ļ��У� ����һ������Ϊ������ chassis_behaviour_e
    erum
    {  
        ...
        ...
        CHASSIS_XXX_XXX, // �����ӵ�
    }chassis_behaviour_e,

    2. ʵ��һ���µĺ��� chassis_xxx_xxx_control(fp32 *vx, fp32 *vy, fp32 *wz, chassis_move_t * chassis )
        "vx,vy,wz" �����ǵ����˶�����������
        ��һ������: 'vx' ͨ�����������ƶ�,��ֵ ǰ���� ��ֵ ����
        �ڶ�������: 'vy' ͨ�����ƺ����ƶ�,��ֵ ����, ��ֵ ����
        ����������: 'wz' �����ǽǶȿ��ƻ�����ת�ٶȿ���
        ������µĺ���, ���ܸ� "vx","vy",and "wz" ��ֵ��Ҫ���ٶȲ���
    3.  ��"chassis_behaviour_mode_set"��������У������µ��߼��жϣ���chassis_behaviour_mode��ֵ��CHASSIS_XXX_XXX
        �ں����������"else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" ,Ȼ��ѡ��һ�ֵ��̿���ģʽ
        4��:
        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'�ǽǶȿ��� ��̨�͵��̵���ԽǶ�
        �����������"xxx_angle_set"������'wz'
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'�ǽǶȿ��� ���̵������Ǽ�����ľ��ԽǶ�
        �����������"xxx_angle_set"
        CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'����ת�ٶȿ���
        CHASSIS_VECTOR_RAW : ʹ��'vx' 'vy' and 'wz'ֱ�����Լ�������ֵĵ���ֵ������ֵ��ֱ�ӷ��͵�can ������
    4.  ��"chassis_behaviour_control_set" �������������
        else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
        {
            chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"

#include "gimbal_behaviour.h"

#define ROS_HOST_CHASSIS_DEBUG 1

/**
  * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
  *                 and chassis control mode is raw. The raw chassis control mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all speed zero.
  * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
  * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
  * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vy_set���ҵ��ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      wz_set��ת���ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);


/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_MOVE, chassis control mode is speed control mode.
  *                 chassis does not follow gimbal, and the function will set all speed zero to make chassis no move
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: wz rotate speed value, positive value means counterclockwise , negative value means clockwise.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ���̲��ƶ�����Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set��ת���ٶȣ���ת�ٶ��ǿ��Ƶ��̵ĵ��̽��ٶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
  *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle difference between chassis and gimbal
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ���̸�����̨����Ϊ״̬���£�����ģʽ�Ǹ�����̨�Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      angle_set��������̨���Ƶ�����ԽǶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, chassis control mode is speed control mode.
  *                 chassis will follow chassis yaw, chassis rotation speed is calculated from the angle difference between set angle and chassis yaw.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle[-PI, PI]
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ���̸������yaw����Ϊ״̬���£�����ģʽ�Ǹ�����̽Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      angle_set�������õ�yaw����Χ -PI��PI
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
  *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ���̲�����Ƕȵ���Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�������ת�ٶ��ɲ���ֱ���趨
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set�������õ���ת�ٶ�,��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);

static void chassis_no_follow_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);


/**
  * @brief          when chassis behaviour mode is CHASSIS_OPEN, chassis control mode is raw control mode.
  *                 set value will be sent to can bus.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ���̿�������Ϊ״̬���£�����ģʽ��rawԭ��״̬���ʶ��趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶȣ���ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set ��ת�ٶȣ� ��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         none
  */

static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);






//highlight, the variable chassis behaviour mode 
//���⣬���������Ϊģʽ����
chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;


/**
  * @brief          logical judgement to assign "chassis_behaviour_mode" variable to which mode
  * @param[in]      chassis_move_mode: chassis data
  * @retval         none
  */
/**
  * @brief          ͨ���߼��жϣ���ֵ"chassis_behaviour_mode"������ģʽ
  * @param[in]      chassis_move_mode: ��������
  * @retval         none
  */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }


    //remote control  set chassis behaviour mode
    //ң��������ģʽ
    if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        //can change to CHASSIS_ZERO_FORCE,CHASSIS_NO_MOVE,CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,
        //CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW,CHASSIS_NO_FOLLOW_YAW,CHASSIS_OPEN
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
    }
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }
    else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
//		chassis_behaviour_mode = CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW;
    }

    //when gimbal in some mode, such as init mode, chassis must's move
    //����̨��ĳЩģʽ�£����ʼ���� ���̲���
  #if ROS_HOST_CHASSIS_DEBUG
    chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
  #else
    if (gimbal_cmd_to_chassis_stop())
    {
      chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }
  #endif


    //add your own logic to enter the new mode
    //�����Լ����߼��жϽ�����ģʽ


    //accord to beheviour mode, choose chassis control mode
    //������Ϊģʽѡ��һ�����̿���ģʽ
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; 
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
    }
}


/**
  * @brief          set control set-point. three movement param, according to difference control mode,
  *                 will control corresponding movement.in the function, usually call different control function.
  * @param[out]     vx_set, usually controls vertical speed.
  * @param[out]     vy_set, usually controls horizotal speed.
  * @param[out]     wz_set, usually controls rotation speed.
  * @param[in]      chassis_move_rc_to_vector,  has all data of chassis
  * @retval         none
  */
/**
  * @brief          ���ÿ�����.���ݲ�ͬ���̿���ģʽ��������������Ʋ�ͬ�˶�.������������棬����ò�ͬ�Ŀ��ƺ���.
  * @param[out]     vx_set, ͨ�����������ƶ�.
  * @param[out]     vy_set, ͨ�����ƺ����ƶ�.
  * @param[out]     wz_set, ͨ��������ת�˶�.
  * @param[in]      chassis_move_rc_to_vector,  ��������������Ϣ.
  * @retval         none
  */

void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
//        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
		chassis_no_follow_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_engineer_follow_chassis_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_open_set_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_ZERO_FORCE, the function is called
  *                 and chassis control mode is raw. The raw chassis chontrol mode means set value
  *                 will be sent to CAN bus derectly, and the function will set all speed zero.
  * @param[out]     vx_can_set: vx speed value, it will be sent to CAN bus derectly.
  * @param[out]     vy_can_set: vy speed value, it will be sent to CAN bus derectly.
  * @param[out]     wz_can_set: wz rotate speed value, it will be sent to CAN bus derectly.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vy_set���ҵ��ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      wz_set��ת���ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_MOVE, chassis control mode is speed control mode.
  *                 chassis does not follow gimbal, and the function will set all speed zero to make chassis no move
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: wz rotate speed value, positive value means counterclockwise , negative value means clockwise.
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ���̲��ƶ�����Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set��ת���ٶȣ���ת�ٶ��ǿ��Ƶ��̵ĵ��̽��ٶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW, chassis control mode is speed control mode.
  *                 chassis will follow gimbal, chassis rotation speed is calculated from the angle difference.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle difference between chassis and gimbal
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ���̸�����̨����Ϊ״̬���£�����ģʽ�Ǹ�����̨�Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      angle_set��������̨���Ƶ�����ԽǶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    //channel value and keyboard value change to speed set-point, in general
    //ң������ͨ��ֵ�Լ����̰��� �ó� һ������µ��ٶ��趨ֵ
    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    //swing angle is generated by sin function, swing_time is the input time of sin
    //ҡ�ڽǶ�������sin�������ɣ�swing_time ��sin����������ֵ
    static fp32 swing_time = 0.0f;
    
    static fp32 swing_angle = 0.0f;
    //max_angle is the max angle that chassis will ratate
    //max_angle ��sin�����ķ�ֵ
    static fp32 max_angle = SWING_NO_MOVE_ANGLE;
    //swing_time  plus the add_time in one control cycle
    //swing_time ��һ�����������ڣ����� add_time
    static fp32 const add_time = PI * 0.5f * configTICK_RATE_HZ / CHASSIS_CONTROL_TIME_MS;
    
    static uint8_t swing_flag = 0;

    //judge if swing
    //�ж��Ƿ�Ҫҡ��
    if (chassis_move_rc_to_vector->chassis_RC->key.v & SWING_KEY)
    {
        if (swing_flag == 0)
        {
            swing_flag = 1;
            swing_time = 0.0f;
        }
    }
    else
    {
        swing_flag = 0;
    }

    //judge if keyboard is controlling the chassis, if yes, reduce the max_angle
    //�жϼ��������ǲ����ڿ��Ƶ����˶����������˶���Сҡ�ڽǶ�
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY ||
        chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        max_angle = SWING_MOVE_ANGLE;
    }
    else
    {
        max_angle = SWING_NO_MOVE_ANGLE;
    }
    
    if (swing_flag)
    {
        swing_angle = max_angle * arm_sin_f32(swing_time);
        swing_time += add_time;
    }
    else
    {
        swing_angle = 0.0f;
    }
    //swing_time  range [0, 2*PI]
    //sin����������2pi
    if (swing_time > 2 * PI)
    {
        swing_time -= 2 * PI;
    }


    *angle_set = swing_angle;
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, chassis control mode is speed control mode.
  *                 chassis will follow chassis yaw, chassis rotation speed is calculated from the angle difference between set angle and chassis yaw.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     angle_set: control angle[-PI, PI]
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ���̸������yaw����Ϊ״̬���£�����ģʽ�Ǹ�����̽Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      angle_set�������õ�yaw����Χ -PI��PI
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - CHASSIS_ANGLE_Z_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL]);
}

/**
  * @brief          when chassis behaviour mode is CHASSIS_NO_FOLLOW_YAW, chassis control mode is speed control mode.
  *                 chassis will no follow angle, chassis rotation speed is set by wz_set.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ���̲�����Ƕȵ���Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�������ת�ٶ��ɲ���ֱ���趨
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set�������õ���ת�ٶ�,��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}

static void chassis_no_follow_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
  *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
  //	*wz_set = 5.0f;
	
}


/**
  * @brief          when chassis behaviour mode is CHASSIS_OPEN, chassis control mode is raw control mode.
  *                 set value will be sent to can bus.
  * @param[out]     vx_set: vx speed value, positive value means forward speed, negative value means backward speed,
  * @param[out]     vy_set: vy speed value, positive value means left speed, negative value means right speed.
  * @param[out]     wz_set: rotation speed,positive value means counterclockwise , negative value means clockwise
  * @param[in]      chassis_move_rc_to_vector: chassis data
  * @retval         none
  */
/**
  * @brief          ���̿�������Ϊ״̬���£�����ģʽ��rawԭ��״̬���ʶ��趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
  * @param[in]      vy_set���ҵ��ٶȣ���ֵ �����ٶȣ� ��ֵ �����ٶ�
  * @param[in]      wz_set ��ת�ٶȣ� ��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         none
  */

static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *wz_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    return;
}
