#ifndef VISION_LINK_H
#define VISION_LINK_H

/* ============================================================================
 * Module：与云台板的数据链路（接收「云台 yaw 偏角」）
 *
 * 为什么用 yaw 偏角而不是画面偏差：
 *   云台板一直在跟踪人，人始终被保持在画面中心，所以画面偏差恒接近 0，
 *   **不携带方向信息**，拿它转向车会一直不动。
 *   而「云台转了多少度才对准人」正好编码了人相对车头的方位：
 *   车朝人的方向转，人相对车头就回正，云台也就转回中点 ——
 *   这是一个天然收敛的负反馈。
 *
 * 协议（13 字节，type = 0x02）：
 *   0      head0               0xAA
 *   1      head1               0x55
 *   2      type                0x02
 *   3      length              0x08
 *   4-5    yaw_dev_tenth_deg   int16，云台 yaw 相对中点偏角，单位 0.1 度
 *   6-7    offset_x            int16，画面偏差 -100~+100（诊断用）
 *   8-9    distance_cm         uint16，0 = 未知（预留）
 *   10     flags               uint8，bit0 = 检测到人
 *   11     reserved            uint8
 *   12     checksum            uint8，前 12 字节异或
 *
 * 解析策略与 K230 解析器相同：滑窗找帧头、失配只前进 1 字节、
 * 校验失败只丢 1 字节重同步。
 * ========================================================================== */

#include <stdint.h>

/** @brief 链路数据快照。 */
typedef struct {
    int16_t yaw_dev_tenth_deg;  /* 云台 yaw 相对中点偏角，单位 0.1 度 */
    int16_t offset_x;           /* 画面水平偏差，-100 ~ +100（诊断用） */
    uint16_t distance_cm;       /* 距离，单位 cm；0 = 未知 */
    uint8_t detected;           /* 1 = 对端报告检测到人 */
    uint8_t fresh;              /* 1 = 有人且数据未超时（可直接用于控制） */
    uint32_t frame_count;       /* 校验通过的帧数 */
    uint32_t checksum_error_count;
    uint32_t last_ms;           /* 最近一次合法帧时刻，ms */
} VisionLink_t;

/* 初始化串口与链路状态 */
void VisionLink_Init(void);

/* 每控制周期调用：把缓冲里所有完整帧解析出来，只保留最新一帧 */
void VisionLink_Poll(void);

/** @brief 取最近一次快照（值拷贝）。 */
VisionLink_t VisionLink_Get(void);

/** @return 云台 yaw 偏角，单位 0.1 度；无数据时为 0。 */
int16_t VisionLink_GetYawDevTenthDeg(void);

/** @return 画面水平偏差（诊断用），-100 ~ +100。 */
int16_t VisionLink_GetOffsetX(void);

/** @return 距离，单位 cm；0 表示未知。 */
uint16_t VisionLink_GetDistanceCm(void);

/** @return 1 = 有人且数据新鲜，可用于控制；0 = 不可用。 */
uint8_t VisionLink_IsFresh(void);

/** @brief 最近 CHASSIS_LINK_TIMEOUT_MS 内是否收到过有效帧（不管有没有检测到人）。
 *
 *  与 VisionLink_IsFresh() 的区别：
 *    IsFresh()     = 有人 && 未超时   -> 可以据此跟随
 *    IsReceiving() = 未超时（不管 flag）-> 用来区分「断链」和「在线但丢人」
 *
 *  @return 1 = 链路在线；0 = 从未收到帧或已超时。
 */
uint8_t VisionLink_IsReceiving(void);

#endif /* VISION_LINK_H */
