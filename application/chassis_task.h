#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

/* ============================================================================
 * Application：循迹主控板任务（模式状态机）
 *
 * 对应需求《实验室学习项目》的两种工作模式：
 *
 *   模式 1【循迹 + 云台人体跟踪】
 *       小车沿黑线自主行驶；云台由另一块板独立盯着人。
 *       本板只负责循迹，不参与云台。
 *
 *   模式 2【纯人体跟随】
 *       关闭黑线循迹；根据人体在画面的左右偏差转向、
 *       根据目标远近决定前进/后退，保持安全距离；
 *       目标丢失则减速停止并等待。
 *
 * ⚠️ 当前状态：**模式切换与显示可用，两种模式的驱动逻辑留空**。
 *    留空的部分全部标注了 TODO，且 Motor_SetSpeed 目前不真正驱动电机，
 *    所以现在烧进去车不会动（安全）。
 * ========================================================================== */

#include <stdint.h>

/** @brief 工作模式。 */
typedef enum {
    CHASSIS_MODE_LINE_TRACK = 0,    /* 模式 1：循迹 + 云台跟踪 */
    CHASSIS_MODE_FOLLOW,            /* 模式 2：纯人体跟随 */
    CHASSIS_MODE_COUNT
} ChassisMode_t;

/**
 * @brief 初始化：电机 PWM（滑行）、循迹输入、按键、OLED、双板链路。
 * @note 上电默认进入 CHASSIS_MODE_LINE_TRACK，但电机保持滑行不动作。
 */
void ChassisTask_Init(void);

/**
 * @brief 任务单次执行，调用周期必须等于 CHASSIS_CONTROL_PERIOD_MS（5 ms）。
 *
 * 内部完成：链路轮询 -> 循迹采样 -> 按键消抖与模式切换 -> 按模式分派
 *          -> 节流刷新 OLED 与串口回传。
 */
void ChassisTask_Loop(void);

/** @return 当前工作模式。 */
ChassisMode_t ChassisTask_GetMode(void);

#endif /* CHASSIS_TASK_H */
