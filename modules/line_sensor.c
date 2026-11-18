#include "line_sensor.h"

#include "bsp_line_gpio.h"
#include "chassis_config.h"

/* 每路的权重：CH1..CH7 = -3 -2 -1 0 +1 +2 +3
 * 装车后各路间距若不相等，必须按实际间距重算权重（不能直接用 -3..+3）。 */
static const int8_t s_weight[LINE_SENSOR_CHANNEL_COUNT] =
{
    -3, -2, -1, 0, 1, 2, 3
};

/* 循迹传感器快照 */
static LineSensor_t s_line;

/* 连续「全都没压线」的拍数，用于脱线判定 */
static uint16_t s_lost_count;

void LineSensor_Init(void)
{
    s_line.raw_levels = 0U;
    s_line.active_mask = 0U;
    s_line.position = 0;
    s_line.lost = 1U;           /* 上电还没采过，按「不知道」处理 */
    s_lost_count = 0U;

    BSP_Line_Init();
}

void LineSensor_Update(void)
{
    uint8_t active = 0U;
    int32_t sum = 0;
    uint8_t hit = 0U;
    uint8_t i;

    s_line.raw_levels = BSP_Line_ReadRaw();

    /* 1) 极性转换：把电平转成「这一路是否压着黑线」 */
    for (i = 0U; i < LINE_SENSOR_CHANNEL_COUNT; i++) {
        uint8_t level = (uint8_t)((s_line.raw_levels >> i) & 0x01U);
        uint8_t pressed;

#if (CHASSIS_LINE_ACTIVE_LOW != 0U)
        pressed = (uint8_t)(level == 0U);   /* 压线 = 低电平 */
#else
        pressed = level;                    /* 压线 = 高电平 */
#endif

        if (pressed != 0U) {
            active |= (uint8_t)(1U << i);
            sum += (int32_t)s_weight[i];
            hit++;
        }
    }
    s_line.active_mask = active;

    if (hit != 0U) {
        /* 2) 加权平均 -> -100（最左）~ +100（最右）
         * 先放大 10 倍做整数运算，避免整除丢精度；权重最大 3，故除以 (hit*3)。
         * 多路同时压线时取平均，线卡在两路之间时会给出中间值，而不是来回跳。 */
        int32_t scaled = (sum * 1000) / ((int32_t)hit * 3);

        if (scaled > 1000) {
            scaled = 1000;
        } else if (scaled < -1000) {
            scaled = -1000;
        }
        s_line.position = (int16_t)(scaled / 10);

        s_lost_count = 0U;
        s_line.lost = 0U;
    } else {
        /* 3) 脱线判定：全部没压线，连续 CHASSIS_LINE_LOST_COUNT 拍才算真脱线 */
        s_line.position = 0;

        if (s_lost_count < 0xFFFFU) {
            s_lost_count++;
        }
        s_line.lost = (s_lost_count >= CHASSIS_LINE_LOST_COUNT) ? 1U : 0U;
    }
}

LineSensor_t LineSensor_Get(void)
{
    return s_line;
}
