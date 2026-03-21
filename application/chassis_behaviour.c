#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "arm_math.h"

#include "gimbal_behaviour.h"
#include "vision_rx_task.h"
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

static void chassis_ros_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);


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

    /* ROS autonomous mode: always NO_FOLLOW_YAW, no RC dependency */
    chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;

    //accord to beheviour mode, choose chassis control mode
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
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
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

    chassis_ros_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    *angle_set = 0.0f;
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

    chassis_ros_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    *angle_set = chassis_move_rc_to_vector->chassis_yaw_set;
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

    chassis_ros_control(vx_set, vy_set, wz_set, chassis_move_rc_to_vector);
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

    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
    return;
}

/**
  * @brief          ROS autonomous chassis control — read vx/vy/vz from ROS nav command
  */
static void chassis_ros_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    const ros_nav_cmd_t *nav = chassis_move_rc_to_vector->ros_nav_cmd;
    if (nav != NULL && (nav->nav_ctrl_flags & NAV_FLAG_CHASSIS_VALID))
    {
        *vx_set = nav->vx;
        *vy_set = nav->vy;
        *wz_set = nav->vz;
    }
    else
    {
        *vx_set = 0.0f;
        *vy_set = 0.0f;
        *wz_set = 0.0f;
    }
}
