#ifndef PARAMS_H
#define PARAMS_H

/* ============================================================================
 * Module：运行期参数表（在线调参）
 *
 * 目的：
 *   把「调参要重新编译烧录」变成「发一条命令就生效」。
 *   控制律原本直接引用 config 目录里的 #define，编译期就固定；
 *   本模块把其中需要现场整定的那些搬进一张运行期表，
 *   chassis_task.c 顶部用一组 #undef/#define 把宏名重定向到
 *   Params_Get()，因此控制律代码一个字都不用改。
 *
 * 回滚：
 *   把 config/motor_pid_config.h 的 PARAM_LIVE 改成 0，
 *   所有宏立刻恢复成编译期常量，行为与加功能之前完全一致。
 *
 * 单位：
 *   全部用 int16 传递（链路帧只有 2 字节给值），
 *   所以浮点参数一律按倍数放大成整数（例如 Kp=0.12 存成 12）。
 *   本表里前 13 个参数都是整数单位，放大倍数是 1。
 *
 * 参数 ID：0~63 归底盘（本板），64~127 归云台板。
 *   必须与 Gimbal_2026/modules/param_table.c 以及
 *   docs/双板分工与协议.md 三处保持一致。
 * ========================================================================== */

#include <stdint.h>

/* ---- 参数 ID ---- */
#define PARAM_ID_SG          0U    /* 转向增益    ‰ / 度        */
#define PARAM_ID_SDB         1U    /* 转向死区    0.1 度        */
#define PARAM_ID_SMAX        2U    /* 转向上限    ‰             */
#define PARAM_ID_DG          3U    /* 距离增益    ‰ / cm        */
#define PARAM_ID_VMAX        4U    /* 前后速度上限 ‰            */
#define PARAM_ID_BASE        5U    /* 距离无效直行速度 ‰        */
#define PARAM_ID_DDB         6U    /* 距离死区    cm            */
#define PARAM_ID_KD          7U    /* 转向阻尼    ‰ /(度/秒)    */
#define PARAM_ID_TO          8U    /* 只转不走门限 0.1 度       */
#define PARAM_ID_LG          9U    /* 循迹增益                  */
#define PARAM_ID_LV         10U    /* 循迹速度    ‰             */
#define PARAM_ID_BD         11U    /* 盲转门限    0.1 度        */
#define PARAM_ID_BS         12U    /* 盲转速度    ‰             */
#define PARAM_ID_A2         13U    /* A2 开关：是否允许参数用 Flash 保存/加载 */
#define PARAM_ID_LS         14U    /* 循迹弯道降速  %（满偏转时减速多少） */
#define PARAM_ID_LH         15U    /* 循迹脱线保持  ms          */
#define PARAM_ID_LKD        16U    /* 循迹 D 项     ‰/单位      */
#define PARAM_ID_CRS        17U    /* 十字路口停车门限 ms（0=只长鸣不停车） */
#define PARAM_ID_WAVE       18U    /* VOFA+ 波形输出开关（0=关 1=开） */

#define PARAM_COUNT         19U

/* ---- 返回码：与链路帧 0x04 的错误码一一对应 ---- */
#define PARAM_OK             0U
#define PARAM_ERR_ID         1U    /* 参数号不存在 */
#define PARAM_ERR_RANGE      2U    /* 超出允许范围 */
#define PARAM_ERR_READONLY   3U    /* 只读参数（本表没有，留给将来） */
#define PARAM_ERR_NOT_READY  4U    /* 参数表未初始化 */

/** @brief 一个可调参数。 */
typedef struct {
    uint16_t id;          /* 参数编号，见 PARAM_ID_* */
    const char *name;     /* 短名，供 dump 显示（云台板用同一套名字） */
    const char *unit;     /* 单位，供 dump 显示 */
    int16_t min_value;    /* 允许最小值（含） */
    int16_t max_value;    /* 允许最大值（含） */
    int16_t value;        /* 当前生效值 */
} Param_t;

/** @brief 初始化：把所有参数恢复成编译期默认值。 */
void Params_Init(void);

/**
 * @brief 取参数当前值。
 * @param id 参数编号
 * @return 当前值；编号非法时返回 0（调用方不应传非法编号）
 */
int16_t Params_Get(uint16_t id);

/**
 * @brief 设置参数（带范围校验）。
 * @param id    参数编号
 * @param value 新值
 * @return PARAM_OK / PARAM_ERR_ID / PARAM_ERR_RANGE
 *
 * @note 超范围时**拒绝**并保留原值，不做自动夹紧 ——
 *       悄悄夹到边界会让操作者以为改成功了，反而更难排障。
 */
uint8_t Params_Set(uint16_t id, int16_t value);

/** @return 1 = 编号存在；0 = 不存在。 */
uint8_t Params_Exists(uint16_t id);

/** @brief 全部恢复默认值。 */
void Params_Reset(void);

/** @return 参数个数（= PARAM_COUNT）。 */
uint8_t Params_Count(void);

/** @brief 按下标取参数项，越界返回 NULL。 */
const Param_t *Params_At(uint8_t index);

/* ---------------------------------------------------------------- A2 存储 --

 * A2 = 「参数存 Flash」。用一个字符（蓝牙发 f）随时开关：
 *   关（默认）：完全不碰 Flash，行为与没有这个功能时一模一样；
 *   开：允许 save 写盘，且上电时自动加载上次存的值。
 *
 * 开关状态本身也存在同一页里，所以「关掉之后断电」是真的关掉了。
 * ---------------------------------------------------------------------- */

/**
 * @brief 从 Flash 读参数（上电时由 Params_Init 调用）。
 * @return 1 = 读到有效数据并已加载；0 = 没有（用默认值）
 */
uint8_t Params_LoadFromFlash(void);

/**
 * @brief 把当前参数写进 Flash。
 * @return PARAM_OK / PARAM_ERR_NOT_READY（A2 未开启）/ PARAM_ERR_RANGE（写失败）
 */
uint8_t Params_Save(void);

/**
 * @brief 切换 A2 开关，并把新状态立即写盘（这样"关"才真的能保持到下次上电）。
 * @return 切换后的状态：1 = 开，0 = 关
 */
uint8_t Params_ToggleA2(void);

/** @return 1 = A2 已开启。 */
uint8_t Params_A2Enabled(void);

#endif /* PARAMS_H */
