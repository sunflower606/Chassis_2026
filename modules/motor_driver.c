#include "motor_driver.h"

#include "bsp_motor_pwm.h"
#include "chassis_config.h"
#include "speed_loop.h"

/* 目标值（上层要的）与输出值（真正给到硬件的）分开保存，
 * 两者之间的差距由斜率限制逐步弥合。
 *
 * P1 之后多了一层：上层的「目标」是**速度**（‰），
 * 中间经过 SpeedLoop 的 PI 变成**占空比**，再经斜率限制送到 H 桥。 */
static int16_t s_left_target = 0;   /* 目标速度 ‰（正 = 后退） */
static int16_t s_right_target = 0;
static int16_t s_left_output = 0;   /* 实际占空比 ‰ */
static int16_t s_right_output = 0;

/* 把目标值夹到物理允许的范围 */
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

    SpeedLoop_Init();

    /* BSP 初始化末尾会 Stop()，保证上电滑行 */
    BSP_Motor_Init();
}

void Motor_SetSpeed(int16_t left_per_mille, int16_t right_per_mille)
{
#if (CHASSIS_SPEED_LOOP_ENABLE != 0)
    int16_t duty_left = 0;
    int16_t duty_right = 0;

    /* 速度 -> 占空比：编码器反馈 + PI，内部已按 CHASSIS_MOTOR_DUTY_MAX 限幅。
     * 注意这里传进去的是**速度目标**，不是占空比。 */
    SpeedLoop_Update(left_per_mille, right_per_mille, &duty_left, &duty_right);

    s_left_target = Motor_Clamp(duty_left);
    s_right_target = Motor_Clamp(duty_right);
#else
    /* 开环模式：参数就是占空比，按老路子走 */
    s_left_target = Motor_Clamp(left_per_mille);
    s_right_target = Motor_Clamp(right_per_mille);
#endif

    /* 斜率限制：每调用一次只走一步（对占空比生效，防电流冲击） */
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

    /* 清积分并丢弃这段时间攒下的编码器增量：
     * 否则下次起步第一拍会读到一个巨大的假速度，PI 会先反向踢一下。 */
    SpeedLoop_Reset();

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

int16_t Motor_GetLeftSpeed(void)
{
    return SpeedLoop_GetLeftPermille();
}

int16_t Motor_GetRightSpeed(void)
{
    return SpeedLoop_GetRightPermille();
}
