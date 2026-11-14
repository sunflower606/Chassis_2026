#ifndef SPEED_LOOP_H
#define SPEED_LOOP_H

/* ============================================================================
 * Module：轮速闭环（编码器反馈 + PI）
 *
 * 解决的问题
 * ----------
 * 改之前 `Motor_SetSpeed(x)` 的 x 是**占空比**：同样 350‰，
 * 电池满电跑得飞快、电池亏电就爬不动；地上有地毯更是两回事。
 * 所以「调参数」永远调不准 —— 因为被控对象一直在变。
 *
 * 加了本模块之后 `Motor_SetSpeed(x)` 的 x 变成**目标速度**：
 *   - 前馈：按标定把目标速度折算成占空比，直接给一个大致正确的输出；
 *   - 反馈：用编码器实测速度，PI 修正剩余误差。
 * 结果：同一份参数下，满电/亏电、地砖/地毯的车速基本一致。
 *
 * 单位约定（与 chassis_task.c 一致，务必别搞反）
 * ---------------------------------------------
 *   **正 = 后退、负 = 前进**。本模块沿用这个约定：
 *   输入的 target_per_mille 正数=要让轮子往后转。
 *
 * 采样与更新频率
 * --------------
 * `SpeedLoop_Update()` 每个控制拍（5 ms）都会被调用一次，但**只有每
 * CHASSIS_SPEED_LOOP_DIV 拍（默认 20 ms）才真正算一次 PI**，其余拍输出保持。
 * 原因：MG513 编码器 5 ms 只走十几个计数，1 个计数的量化误差就相当于
 * 几十‰ 的速度跳动，PI 会跟着噪声抖。攒到 20 ms 再算，分辨率提高 4 倍。
 *
 * 静止与堵转
 * ----------
 *   - 目标为 0：输出直接清零、积分清零（不做刹车，靠惯性滑行）。
 *   - 有目标但算出来占空比很小：补到 CHASSIS_SPEED_MIN_DUTY，
 *     否则电机克服不了静摩擦，PI 会一直以为「还没动」而把积分顶到上限。
 *   - 堵转（顶墙/卡住）：积分被 CHASSIS_SPEED_INTEG_MAX 夹住，
 *     并把注入方向的积分撤掉（抗饱和），松开车立刻恢复。
 * ========================================================================== */

#include <stdint.h>

/** @brief 清空积分与上次输出（上电、停车、急停后调用）。 */
void SpeedLoop_Init(void);

/** @brief 同 SpeedLoop_Init：丢弃编码器增量并清积分，用于 Motor_Stop()。 */
void SpeedLoop_Reset(void);

/**
 * @brief 一个控制拍调用一次。
 * @param target_left_per_mille  左轮目标速度（‰，正=后退）
 * @param target_right_per_mille 右轮目标速度（‰，正=后退）
 * @param duty_left_per_mille    输出：左轮占空比（‰，已限幅到 ±CHASSIS_MOTOR_DUTY_MAX）
 * @param duty_right_per_mille   输出：右轮占空比（同上）
 */
void SpeedLoop_Update(int16_t target_left_per_mille, int16_t target_right_per_mille,
                      int16_t *duty_left_per_mille, int16_t *duty_right_per_mille);

/** @return 左轮实测速度（‰，正=后退）。调试显示用。 */
int16_t SpeedLoop_GetLeftPermille(void);

/** @return 右轮实测速度（‰，正=后退）。调试显示用。 */
int16_t SpeedLoop_GetRightPermille(void);

#endif /* SPEED_LOOP_H */
