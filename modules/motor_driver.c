#include "motor_driver.h"

#include "bsp_motor_pwm.h"
#include "chassis_config.h"

/* 目标值（上层要的）与输出值（真正给到硬件的）分开保存，
 * 两者之间的差距由斜率限制逐步弥合。 */
static int16_t s_left_target = 0;
static int16_t s_right_target = 0;
static int16_t s_left_output = 0;
static int16_t s_right_output = 0;

/* 把目标值夹到配置允许的范围 */
static int16_t Motor_Clamp(int16_t speed_per_mille)
{
    if (speed_per_mille > (int16_t)CHASSIS_MOTOR_DUTY_MAX) {
        return (int16_t)CHASSIS_MOTOR_DUTY_MAX;
    }
    if (speed_per_mille < -(int16_t)CHASSIS_MOTOR_DUTY_MAX) {
        return -(int16_t)CHASSIS_MOTOR_DUTY_MAX;
    }

    return speed_per_mille;
}

/* 朝目标靠近一步，步长 = CHASSIS_MOTOR_RAMP_PER_TICK */
static int16_t Motor_RampToward(int16_t current, int16_t target)
{
    int16_t step = (int16_t)CHASSIS_MOTOR_RAMP_PER_TICK;

    /* 步长为 0 时直接跟随，避免下面加减无效果 */
    if (step <= 0) {
        return target;
    }

    if (target > current) {
        current = (int16_t)(current + step);
        if (current > target) {
            current = target;
        }
    } else if (target < current) {
        current = (int16_t)(current - step);
        if (current < target) {
            current = target;
        }
    }

    return current;
}

void Motor_Init(void)
{
    s_left_target = 0;
    s_right_target = 0;
    s_left_output = 0;
    s_right_output = 0;

    /* BSP 初始化末尾会 Stop()，保证上电滑行 */
    BSP_Motor_Init();
}

void Motor_SetSpeed(int16_t left_per_mille, int16_t right_per_mille)
{
    s_left_target = Motor_Clamp(left_per_mille);
    s_right_target = Motor_Clamp(right_per_mille);

    /* 斜率限制：每调用一次只走一步 */
    s_left_output = Motor_RampToward(s_left_output, s_left_target);
    s_right_output = Motor_RampToward(s_right_output, s_right_target);

    BSP_Motor_SetLeft(s_left_output);
    BSP_Motor_SetRight(s_right_output);
}

void Motor_Stop(void)
{
    /* 安全路径不走斜率：直接清零并关断 H 桥 */
    s_left_target = 0;
    s_right_target = 0;
    s_left_output = 0;
    s_right_output = 0;

    BSP_Motor_Stop();
}

int16_t Motor_GetLeftOutput(void)
{
    return s_left_output;
}

int16_t Motor_GetRightOutput(void)
{
    return s_right_output;
}
