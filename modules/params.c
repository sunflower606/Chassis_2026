#include "params.h"

#include <stddef.h>

#include "chassis_config.h"
#include "bsp_flash.h"

/* 参数表。
 *
 * 初值直接取 config 里的宏 —— 这样「参数默认值」只有一处真源：
 * 改宏 = 改出厂默认值，不用在两个文件里同步两遍。
 *
 * min/max 不是随便写的：
 *   - 上限都设成「再大就会明显危险或明显无意义」的值，
 *     例如 VMAX 最大 700，等于电机硬上限，再大也没用；
 *   - 下限防止「调成 0 之后看起来像坏了」，
 *     例如 SG 最小 1，调 0 会让车永远不转向。
 */
static Param_t s_params[PARAM_COUNT] = {
    { PARAM_ID_SG,   "sg",   "permille/deg", 1,   20,
      (int16_t)CHASSIS_FOLLOW_STEER_GAIN_Q },
    { PARAM_ID_SDB,  "sdb",  "0.1deg",       0,   300,
      (int16_t)CHASSIS_FOLLOW_STEER_DEADBAND_TENTH_DEG },
    { PARAM_ID_SMAX, "smax", "permille",     0,   700,
      (int16_t)CHASSIS_FOLLOW_STEER_MAX },
    { PARAM_ID_DG,   "dg",   "permille/cm",  1,   20,
      (int16_t)CHASSIS_FOLLOW_DISTANCE_GAIN_Q },
    { PARAM_ID_VMAX, "vmax", "permille",     50,  700,
      (int16_t)CHASSIS_FOLLOW_SPEED_MAX },
    { PARAM_ID_BASE, "base", "permille",     50,  700,
      (int16_t)CHASSIS_FOLLOW_BASE_SPEED },
    { PARAM_ID_DDB,  "ddb",  "cm",           0,   50,
      (int16_t)CHASSIS_FOLLOW_DISTANCE_DEADBAND_CM },
    { PARAM_ID_KD,   "kd",   "permille/(deg/s)", 0, 20,
      (int16_t)CHASSIS_FOLLOW_STEER_KD },
    { PARAM_ID_TO,   "to",   "0.1deg",       100, 1800,
      (int16_t)CHASSIS_FOLLOW_TURN_ONLY_TENTH_DEG },
    { PARAM_ID_LG,   "lg",   "coef",         0,   800,
      (int16_t)CHASSIS_LINE_STEER_GAIN },
    { PARAM_ID_LV,   "lv",   "permille",     50,  700,
      (int16_t)CHASSIS_LINE_BASE_SPEED },
    { PARAM_ID_BD,   "bd",   "0.1deg",       100, 1800,
      (int16_t)CHASSIS_FOLLOW_BLIND_DEV_TENTH_DEG },
    { PARAM_ID_BS,   "bs",   "permille",     0,   700,
      (int16_t)CHASSIS_FOLLOW_BLIND_STEER },
    /* A2 不是控制量，是"是否允许参数存 Flash"的开关：0 = 关（默认），1 = 开 */
    { PARAM_ID_A2,   "a2",   "0=off 1=on",   0,   1,
      0 },
    { PARAM_ID_LS,   "ls",   "percent",      0,  90,
      (int16_t)CHASSIS_LINE_SLOW_GAIN },
    { PARAM_ID_LH,   "lh",   "ms",           0, 800,
      (int16_t)CHASSIS_LINE_LOST_HOLD_MS },
    { PARAM_ID_LKD,  "lkd",  "permille",  -200, 200,
      (int16_t)CHASSIS_LINE_STEER_KD },
    { PARAM_ID_CRS,  "crs",  "ms",           0, 1000,
      (int16_t)CHASSIS_LINE_CROSS_MS },
    /* 波形输出开关：只影响串口多不多发一行数字，不影响任何控制量 */
    { PARAM_ID_WAVE, "wave", "0/1",          0, 1,    0 }
};

/* 表里每一项的出厂默认值（编译期常量，用于 Reset） */
static const int16_t s_defaults[PARAM_COUNT] = {
    (int16_t)CHASSIS_FOLLOW_STEER_GAIN_Q,
    (int16_t)CHASSIS_FOLLOW_STEER_DEADBAND_TENTH_DEG,
    (int16_t)CHASSIS_FOLLOW_STEER_MAX,
    (int16_t)CHASSIS_FOLLOW_DISTANCE_GAIN_Q,
    (int16_t)CHASSIS_FOLLOW_SPEED_MAX,
    (int16_t)CHASSIS_FOLLOW_BASE_SPEED,
    (int16_t)CHASSIS_FOLLOW_DISTANCE_DEADBAND_CM,
    (int16_t)CHASSIS_FOLLOW_STEER_KD,
    (int16_t)CHASSIS_FOLLOW_TURN_ONLY_TENTH_DEG,
    (int16_t)CHASSIS_LINE_STEER_GAIN,
    (int16_t)CHASSIS_LINE_BASE_SPEED,
    (int16_t)CHASSIS_FOLLOW_BLIND_DEV_TENTH_DEG,
    (int16_t)CHASSIS_FOLLOW_BLIND_STEER,
    0,                           /* A2 出厂默认关闭 */
    (int16_t)CHASSIS_LINE_SLOW_GAIN,
    (int16_t)CHASSIS_LINE_LOST_HOLD_MS,
    (int16_t)CHASSIS_LINE_STEER_KD,
    (int16_t)CHASSIS_LINE_CROSS_MS
};

static uint8_t s_ready = 0U;

static Param_t *Params_Find(uint16_t id)
{
    uint8_t i;

    for (i = 0U; i < PARAM_COUNT; i++) {
        if (s_params[i].id == id) {
            return &s_params[i];
        }
    }
    return NULL;
}

void Params_Init(void)
{
    uint8_t i;

    for (i = 0U; i < PARAM_COUNT; i++) {
        s_params[i].value = s_defaults[i];
    }
    s_ready = 1U;

    /* 上次存过盘就直接用存的值（A2 关着时这个函数会返回 0，一切照旧） */
    (void)Params_LoadFromFlash();
}

int16_t Params_Get(uint16_t id)
{
    Param_t *param;

    param = Params_Find(id);
    return (param != NULL) ? param->value : 0;
}

uint8_t Params_Set(uint16_t id, int16_t value)
{
    Param_t *param;

    if (s_ready == 0U) {
        return PARAM_ERR_NOT_READY;
    }

    param = Params_Find(id);
    if (param == NULL) {
        return PARAM_ERR_ID;
    }

    /* 超范围直接拒绝，不夹紧 —— 见 params.h 里的说明 */
    if (value < param->min_value || value > param->max_value) {
        return PARAM_ERR_RANGE;
    }

    param->value = value;
    return PARAM_OK;
}

uint8_t Params_Exists(uint16_t id)
{
    return (Params_Find(id) != NULL) ? 1U : 0U;
}

void Params_Reset(void)
{
    Params_Init();
}

uint8_t Params_Count(void)
{
    return PARAM_COUNT;
}

const Param_t *Params_At(uint8_t index)
{
    if (index >= PARAM_COUNT) {
        return NULL;
    }
    return &s_params[index];
}

/* ============================================================ A2：存 Flash ==
 * 一页 1 KB 只放 19 个半字（38 字节），剩余空间留白 ——
 * 将来要加参数不用改布局，只要 count 对得上就行。
 *
 * 布局（半字下标）：
 *   0-1  magic（uint32，小端两半）
 *   2    version
 *   3    count（= PARAM_COUNT）
 *   4..  14 个参数值
 *   18   crc（前 18 个半字逐字异或）
 * ========================================================================== */

#define PARAM_FLASH_MAGIC      0x50413231UL   /* 'P','A','2','1' */
#define PARAM_FLASH_VERSION    1U
#define PARAM_FLASH_HW_COUNT   (4U + PARAM_COUNT + 1U)

/* 只在需要写盘时才插进去，读的时候不需要这个 */
static uint16_t s_hw[PARAM_FLASH_HW_COUNT];

static uint16_t Params_Crc(const uint16_t *hw, uint16_t count)
{
    uint16_t crc = 0U;
    uint16_t i;

    for (i = 0U; i < count; i++) {
        crc ^= hw[i];
    }
    return crc;
}

static void Params_Pack(void)
{
    uint8_t i;

    s_hw[0] = (uint16_t)(PARAM_FLASH_MAGIC & 0xFFFFUL);
    s_hw[1] = (uint16_t)((PARAM_FLASH_MAGIC >> 16) & 0xFFFFUL);
    s_hw[2] = (uint16_t)PARAM_FLASH_VERSION;
    s_hw[3] = (uint16_t)PARAM_COUNT;

    for (i = 0U; i < PARAM_COUNT; i++) {
        s_hw[4U + i] = (uint16_t)s_params[i].value;
    }

    s_hw[PARAM_FLASH_HW_COUNT - 1U] = Params_Crc(s_hw, (uint16_t)(PARAM_FLASH_HW_COUNT - 1U));
}

/* 真正写盘：不看 A2 开关（开关自己也要写进去） */
static uint8_t Params_WriteFlash(void)
{
    if (BSP_Flash_ErasePage(BSP_FLASH_PARAM_PAGE_ADDR) == 0U) {
        return 0U;
    }

    Params_Pack();

    if (BSP_Flash_ProgramHalfWords(BSP_FLASH_PARAM_PAGE_ADDR, s_hw,
                                   (uint16_t)PARAM_FLASH_HW_COUNT) == 0U) {
        return 0U;
    }

    /* 回读校验：魔数对得上才算真的写进去了 */
    return (BSP_Flash_ReadHalfWord(BSP_FLASH_PARAM_PAGE_ADDR) ==
            (uint16_t)(PARAM_FLASH_MAGIC & 0xFFFFUL)) ? 1U : 0U;
}

uint8_t Params_LoadFromFlash(void)
{
    uint16_t hw[PARAM_FLASH_HW_COUNT];
    uint16_t magic;
    uint8_t i;

    for (i = 0U; i < (uint8_t)PARAM_FLASH_HW_COUNT; i++) {
        hw[i] = BSP_Flash_ReadHalfWord(BSP_FLASH_PARAM_PAGE_ADDR + ((uint32_t)i * 2UL));
    }

    magic = (uint16_t)(hw[0] | ((uint16_t)0U << 16));

    if ((magic != (uint16_t)(PARAM_FLASH_MAGIC & 0xFFFFUL)) ||
        (hw[1] != (uint16_t)((PARAM_FLASH_MAGIC >> 16) & 0xFFFFUL)) ||
        (hw[2] != (uint16_t)PARAM_FLASH_VERSION) ||
        (hw[3] != (uint16_t)PARAM_COUNT)) {
        return 0U;
    }

    if (hw[PARAM_FLASH_HW_COUNT - 1U] !=
        Params_Crc(hw, (uint16_t)(PARAM_FLASH_HW_COUNT - 1U))) {
        return 0U;
    }

    /* 存盘数据也要过一遍范围校验：万一有人改过布局，宁可回默认值 */
    for (i = 0U; i < PARAM_COUNT; i++) {
        int16_t v = (int16_t)hw[4U + i];

        if ((v < s_params[i].min_value) || (v > s_params[i].max_value)) {
            return 0U;
        }
    }

    /* 开关本身存的是 0 时，说明用户主动关掉了 A2，那就别加载参数 */
    if ((int16_t)hw[4U + PARAM_ID_A2] == 0) {
        return 0U;
    }

    for (i = 0U; i < PARAM_COUNT; i++) {
        s_params[i].value = (int16_t)hw[4U + i];
    }

    return 1U;
}

uint8_t Params_A2Enabled(void)
{
    return (Params_Get(PARAM_ID_A2) != 0) ? 1U : 0U;
}

uint8_t Params_Save(void)
{
    if (s_ready == 0U) {
        return PARAM_ERR_NOT_READY;
    }

    if (Params_A2Enabled() == 0U) {
        return PARAM_ERR_NOT_READY;   /* A2 没开 = 不允许写盘 */
    }

    return (Params_WriteFlash() != 0U) ? PARAM_OK : PARAM_ERR_RANGE;
}

uint8_t Params_ToggleA2(void)
{
    uint8_t on = (Params_A2Enabled() != 0U) ? 0U : 1U;

    (void)Params_Set(PARAM_ID_A2, (int16_t)on);

    /* 立即写盘：这样"关掉"才真的能保持到下次上电，
     * 否则重新上电又会从 Flash 里读回 on。失败也不改返回值。 */
    (void)Params_WriteFlash();

    return on;
}
