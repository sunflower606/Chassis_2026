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
 *   11     command             uint8，0 = 无命令 / 1 = mode1 / 2 = mode2 / 3 = stop
 *                               （手机经蓝牙发到云台板的文本命令，云台板解析后
 *                                 借这个字节下发；原来是保留位，长度不变）
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
    uint8_t command;            /* 0 = 无命令；1/2/3 = mode1/mode2/stop */
    uint8_t fresh;              /* 1 = 有人且数据未超时（可直接用于控制） */
    uint32_t frame_count;       /* 校验通过的帧数 */
    uint32_t checksum_error_count;
    uint32_t last_ms;           /* 最近一次合法帧时刻，ms */
} VisionLink_t;

/* 初始化串口与链路状态 */
void VisionLink_Init(void);

/* 每控制周期调用：把缓冲里所有完整帧解析出来，只保留最新一帧 */
void VisionLink_Poll(void);

/* 0x05 提示音帧的音型号 —— 必须与 Gimbal_2026/modules/buzzer.h 的枚举一一对应 */
#define LINK_BEEP_START          0U   /* 起跑：两声上行 */
#define LINK_BEEP_CROSS          1U   /* 过十字：一声短 */
#define LINK_BEEP_LOST           2U   /* 丢线/丢人：三声急促 */
#define LINK_BEEP_STOP_LINE      3U   /* 到停车线：长鸣 1 s */
#define LINK_BEEP_LINK_LOST      4U   /* 断链：长鸣 0.5 s */
#define LINK_BEEP_LOW_BAT        5U   /* 低电压：一短声 */

/* 模式上报（0x06，底盘 -> 云台，第 8 字节 = 模式码）。
 * 云台用它决定"丢人后要不要扫视"：只有模式 1 才扫。 */
#define LINK_MODE_LINE           0U   /* 模式 1：循迹 */
#define LINK_MODE_FOLLOW         1U   /* 模式 2：人体跟随 */
#define LINK_MODE_STOP           2U   /* 整车停（蓝牙 stop 锁存） */

/** @brief 让云台板蜂鸣器响一下（0x05 帧，底盘 -> 云台）。
 *  @param pattern 音型号：0 起跑 / 1 过十字 / 2 丢线 / 3 停车线 / 4 断链 / 5 低电压。
 *
 *  蜂鸣器在云台板上，而触发这些事件的信息只有底盘有，所以要跨板发。
 *  内部走 TX 环缓，不阻塞控制环；链路断开时调用无副作用（帧发出去没人收）。 */
void VisionLink_SendBeep(uint8_t pattern);

/**
 * @brief 向云台板上报当前工作模式（LINK_MODE_*）。
 *        模式变化时立刻发一次，另每 1 s 补发一次（防丢帧）。
 */
void VisionLink_SendMode(uint8_t mode);

/** @brief 取最近一次快照（值拷贝）。 */
VisionLink_t VisionLink_Get(void);

/** @return 云台 yaw 偏角，单位 0.1 度；无数据时为 0。 */
int16_t VisionLink_GetYawDevTenthDeg(void);

/** @return 画面水平偏差（诊断用），-100 ~ +100。 */
int16_t VisionLink_GetOffsetX(void);

/** @return 距离，单位 cm；0 表示未知。 */
uint16_t VisionLink_GetDistanceCm(void);

/** @return 云台板转发来的蓝牙命令码：0 = 无命令，1 = mode1，2 = mode2，3 = stop。 */
uint8_t VisionLink_GetCommand(void);

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
