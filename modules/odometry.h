#ifndef ODOMETRY_H
#define ODOMETRY_H

/* ============================================================================
 * 模块：编码器里程计（航向角 + 行驶距离）
 *
 * 作用：用两个驱动轮的**编码器计数差**推算车体转过的角度，
 *       用两轮**计数和**推算走过的距离。
 *
 *   Δs_L = ΔN_L × k        （k = π·D / 每圈计数，单位 米/计数）
 *   Δs_R = ΔN_R × k
 *   θ   += (Δs_R − Δs_L) / b          b = 两轮中心距（米）
 *   里程 += (Δs_L + Δs_R) / 2
 *
 * 符号约定（重要）：
 *   θ > 0  = 车体**向左**转（从上往下看逆时针）
 *   θ < 0  = 车体向右转
 *   里程   恒为正（前进/后退都累加，取绝对值意义）
 *   若实测“向左转 θ 反而变小”，只需把 CHASSIS_ODOMETRY_HEADING_SIGN 改成 -1，
 *   不要改动差速合成或转向符号。
 *
 * 与速度环的关系：速度环每个控制拍都用 BSP_Encoder_GetDelta() 取走增量
 * （见 modules/speed_loop.c），那套接口是**消费型**的，谁先读谁拿到。
 * 本模块因此改用 BSP_Encoder_ReadXCount()（读原始计数、不改状态），
 * 自己保存上一次的快照做差 —— 两套互不干扰，可以同时使用。
 *
 * 精度说明：
 *   1) 打滑时编码器仍计数 → 航向会漂，典型每米 1~3%；
 *   2) 轮距 b 必须实测（两轮接地点中心距），量错会等比缩放所有角度；
 *   3) 只在 P0 观测阶段使用，**尚未参与任何控制律**。
 * ========================================================================== */

#include <stdint.h>

/** @brief 初始化（把当前计数当作零点）。上电或重新起跑时调用。 */
void Odometry_Init(void);

/** @brief 清零航向与里程（不重新读计数，用于“重新起跑”）。 */
void Odometry_Reset(void);

/** @brief 每个控制拍调用一次（5 ms），内部自己算增量。 */
void Odometry_Update(void);

/** @brief 航向角，单位 度，范围 [-180.0, +180.0)，正 = 左转。 */
float Odometry_GetHeadingDeg(void);

/** @brief 航向角，单位 0.1 度（串口/OLED 用）。 */
int16_t Odometry_GetHeadingTenthDeg(void);

/** @brief 累计里程，单位 米（只增不减）。 */
float Odometry_GetDistanceM(void);

/** @brief 累计里程，单位 厘米（串口/OLED 用，饱和到 65535）。 */
uint16_t Odometry_GetDistanceCm(void);

#endif /* ODOMETRY_H */
