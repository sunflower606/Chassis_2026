/* ============================================================================
 * 模块：编码器里程计（航向角 + 行驶距离）
 *
 * 详见 odometry.h 的说明。这里只说实现上的三个取舍：
 *
 *   1) 为什么用 float 累加而不是定点整数
 *      角度是小量高频累加（每拍增量典型 0.01~0.5°），定点要额外维护标度，
 *      而本工程 RAM 够（20 KB，当前用了约 9 KB），float 更不容易写错。
 *
 *   2) 为什么每个控制拍都读
 *      计数是 16 位、会回绕。两次读取间隔越短，单次增量越小，
 *      越不可能超过 32767（超过就分不清是正转还是倒转）。
 *      5 ms 一拍，即使 3000 计数/秒也只有 15 个计数，安全余量极大。
 *
 *   3) 为什么这里不做“打滑校正”
 *      打滑需要外部参照（视觉/地图）。本阶段只做观测，把误差暴露给串口，
 *      等第 3、4 步接入控制时再用视觉定期对齐。
 * ========================================================================== */

#include "odometry.h"

#include "bsp_encoder.h"
#include "chassis_config.h"

/* 圆周率：不引 <math.h>，避免链入浮点数学库（体积 + 链接选项） */
#define ODOMETRY_PI 3.14159265f

/* 每 1 个计数对应的轮面位移（米）：
 *   轮周长 = π × D，D 由 CHASSIS_WHEEL_DIAMETER_MM 给出（毫米 → 米） */
#define ODOMETRY_M_PER_COUNT                                                  \
    (ODOMETRY_PI * ((float)CHASSIS_WHEEL_DIAMETER_MM / 1000.0f) /             \
     (float)CHASSIS_ENCODER_COUNTS_PER_REV)

/* 两轮中心距（米） */
#define ODOMETRY_TRACK_M ((float)CHASSIS_WHEEL_TRACK_MM / 1000.0f)

static int16_t s_last_left;    /* 上一拍读到的左轮原始计数 */
static int16_t s_last_right;   /* 上一拍读到的右轮原始计数 */
static float s_heading_deg;    /* 航向角，已归一化到 [-180, 180) */
static float s_distance_m;     /* 累计里程（米） */

/* 把角度归一化到 [-180, 180)：连续转几圈也不会让数值无限增大 */
static float Odometry_WrapDeg(float deg)
{
    while (deg >= 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

void Odometry_Init(void)
{
    /* 以当前计数为零点：上电时车没动，这就是基准 */
    s_last_left = BSP_Encoder_ReadLeftCount();
    s_last_right = BSP_Encoder_ReadRightCount();

    s_heading_deg = 0.0f;
    s_distance_m = 0.0f;
}

void Odometry_Reset(void)
{
    s_heading_deg = 0.0f;
    s_distance_m = 0.0f;
}

void Odometry_Update(void)
{
    int16_t now_left;
    int16_t now_right;
    int16_t delta_left;
    int16_t delta_right;
    float ds_left;
    float ds_right;

    now_left = BSP_Encoder_ReadLeftCount();
    now_right = BSP_Encoder_ReadRightCount();

    /* int16 相减天然处理 16 位回绕：只要两次读取之间没走过半圈以上就行 */
    delta_left = (int16_t)(now_left - s_last_left);
    delta_right = (int16_t)(now_right - s_last_right);

    s_last_left = now_left;
    s_last_right = now_right;

    /* 本车实测：左轮前进计数为正、右轮前进计数为负（镜像安装，见问题记录 C-04）。
     * 这里把两轮的“前进”统一成正的位移，后面的公式才有意义。 */
    ds_left = (float)delta_left * ODOMETRY_M_PER_COUNT *
              (float)CHASSIS_ODOMETRY_LEFT_SIGN;
    ds_right = (float)delta_right * ODOMETRY_M_PER_COUNT *
               (float)CHASSIS_ODOMETRY_RIGHT_SIGN;

    /* 距离：两轮的平均位移 */
    s_distance_m += (ds_left + ds_right) * 0.5f;

    /* 航向：右轮多走 → 车体左转 → 正角度 */
    s_heading_deg += (ds_right - ds_left) / ODOMETRY_TRACK_M *
                     (float)CHASSIS_ODOMETRY_HEADING_SIGN;

    s_heading_deg = Odometry_WrapDeg(s_heading_deg);
}

float Odometry_GetHeadingDeg(void)
{
    return s_heading_deg;
}

int16_t Odometry_GetHeadingTenthDeg(void)
{
    /* 四舍五入到 0.1°，范围 ±1800 正好落在 int16 内 */
    return (int16_t)(s_heading_deg * 10.0f +
                     ((s_heading_deg >= 0.0f) ? 0.5f : -0.5f));
}

float Odometry_GetDistanceM(void)
{
    return s_distance_m;
}

uint16_t Odometry_GetDistanceCm(void)
{
    float cm = s_distance_m * 100.0f;

    if (cm <= 0.0f) {
        return 0U;
    }
    if (cm >= 65535.0f) {
        return 65535U;
    }

    return (uint16_t)(cm + 0.5f);
}
