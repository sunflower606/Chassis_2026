#include "line_sensor.h"

#include "bsp_line_gpio.h"
#include "chassis_config.h"

/* 循迹传感器快照 */
static LineSensor_t s_line;

void LineSensor_Init(void)
{
    s_line.raw_levels = 0U;
    s_line.position = 0;

    /* 尚未实现判线，明确置 1 表示「不知道」，不要让人误以为线还在 */
    s_line.lost = 1U;

    BSP_Line_Init();
}

void LineSensor_Update(void)
{
    /* 只取原始电平。极性转换、加权、脱线判定都还没实现 —— 见头文件 TODO。 */
    s_line.raw_levels = BSP_Line_ReadRaw();

    /* TODO(循迹实现时):
     *   1) 用 CHASSIS_LINE_ACTIVE_LOW 把 raw_levels 转成 active_mask
     *      （压黑线的通道位置 1，共 7 位）；
     *   2) 对 active_mask 做加权平均 -> position（-100 ~ +100），
     *      权重 CH1..CH7 = -3 -2 -1 0 +1 +2 +3，多路同时压线时取平均；
     *   3) active_mask == 0 连续 CHASSIS_LINE_LOST_COUNT 拍 -> lost = 1。
     */
    s_line.position = 0;
    s_line.lost = 1U;
}

LineSensor_t LineSensor_Get(void)
{
    return s_line;
}
