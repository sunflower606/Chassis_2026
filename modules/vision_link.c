#include "vision_link.h"

#include <stdbool.h>

#include "bsp_uart.h"
#include "blackbox.h"
#include "chassis_config.h"
#include "params.h"
#include "Delay.h"

/* 帧结构与常量：必须与云台板 modules/link_protocol.c 以及
 * docs/双板分工与协议.md 三处保持一致。 */
#define LINK_FRAME_LEN          13U
#define LINK_HEAD_LEN           4U
#define LINK_HEAD_0             0xAAU
#define LINK_HEAD_1             0x55U
#define LINK_TYPE_STATUS        0x02U   /* 状态帧：云台 -> 底盘 */
#define LINK_TYPE_PARAM_SET     0x03U   /* 调参设置：云台 -> 底盘 */
#define LINK_TYPE_PARAM_ACK     0x04U   /* 调参应答：底盘 -> 云台 */
#define LINK_TYPE_BEEP          0x05U   /* 提示音：底盘 -> 云台（第 8 字节 = 音型号） */
#define LINK_TYPE_MODE          0x06U   /* 模式上报：底盘 -> 云台（第 8 字节 = 模式码） */

/* 0x03 帧第 8 字节：操作码 */
#define LINK_PARAM_OP_SET       0U      /* 改值 */
#define LINK_PARAM_OP_GET       1U      /* 只读回当前值 */
#define LINK_PARAM_OP_RESET     2U      /* 全部恢复默认值（id 忽略） */
#define LINK_PARAM_OP_TOGGLE    3U      /* 切换 A2（参数存 Flash）开关，结果在应答值里 */
#define LINK_PARAM_OP_SAVE      4U      /* 把当前参数写进 Flash（需 A2 已开） */
#define LINK_PARAM_OP_DUMP      5U      /* 导出黑匣子（逐行 ASCII 文本回传） */

/* 应答里用来表示"不是某个具体参数"的特殊 id */
#define LINK_ACK_ID_RESET       0xFFFFU /* 恢复默认成功了 */
#define LINK_ACK_ID_SAVE        0xFFFEU /* 存盘结果：value = 1 成功 / 0 失败 */
#define LINK_ACK_ID_DUMP        0xFFFDU /* 黑匣子导出已开始，value = 条数 */
#define LINK_LENGTH             0x08U
#define LINK_CHECKSUM_INDEX     12U

#define LINK_FLAG_DETECTED      0x01U

static VisionLink_t s_link;

/* 大端读取 */
static int16_t VisionLink_ReadInt16Be(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

static uint16_t VisionLink_ReadUint16Be(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

/* offset 处是不是完整的 4 字节帧头 */
static uint8_t VisionLink_IsHeaderAt(uint16_t offset)
{
    uint8_t byte = 0U;

    if (BSP_UART_PeekAt(offset, &byte) == 0U || byte != LINK_HEAD_0) {
        return 0U;
    }
    if (BSP_UART_PeekAt((uint16_t)(offset + 1U), &byte) == 0U || byte != LINK_HEAD_1) {
        return 0U;
    }
    if (BSP_UART_PeekAt((uint16_t)(offset + 2U), &byte) == 0U) {
        return 0U;
    }
    /* 状态帧与调参帧共用同一个 13 字节骨架，只有 type 不同 */
    if ((byte != LINK_TYPE_STATUS) && (byte != LINK_TYPE_PARAM_SET)) {
        return 0U;
    }
    if (BSP_UART_PeekAt((uint16_t)(offset + 3U), &byte) == 0U || byte != LINK_LENGTH) {
        return 0U;
    }

    return 1U;
}

/* ============================================================================
 * 调参帧（type = 0x03 / 0x04）
 *
 * 为什么要这两种帧：
 *   原来的 0x02 是单向的（云台 -> 底盘），只装"状态"。
 *   调参是一次来回对话：上位机问「把 3 号参数改成 6」，
 *   底盘必须能回答「改好了，现在是 6」或者「超范围，没改」，
 *   所以需要一对请求/应答帧。
 *
 * 0x03 设置帧字段： 4-5 参数号，6-7 新值，8-11 保留
 * 0x04 应答帧字段： 4-5 参数号，6-7 当前值，8 保留，9 错误码，10-11 保留
 * ========================================================================== */

/* 发一帧 0x04 应答。13 字节走的是 TX 环缓（不阻塞），不会打乱控制周期。 */
static void VisionLink_SendParamAck(uint16_t id, int16_t value, uint8_t error_code)
{
    uint8_t frame[LINK_FRAME_LEN];
    uint8_t checksum = 0U;
    uint16_t raw_value = (uint16_t)value;
    uint8_t i;

    frame[0] = LINK_HEAD_0;
    frame[1] = LINK_HEAD_1;
    frame[2] = LINK_TYPE_PARAM_ACK;
    frame[3] = LINK_LENGTH;
    frame[4] = (uint8_t)(id >> 8);
    frame[5] = (uint8_t)(id & 0xFFU);
    frame[6] = (uint8_t)(raw_value >> 8);
    frame[7] = (uint8_t)(raw_value & 0xFFU);
    frame[8] = 0U;
    frame[9] = error_code;
    frame[10] = 0U;
    frame[11] = 0U;

    for (i = 0U; i < LINK_CHECKSUM_INDEX; i++) {
        checksum ^= frame[i];
    }
    frame[LINK_CHECKSUM_INDEX] = checksum;

    for (i = 0U; i < LINK_FRAME_LEN; i++) {
        BSP_UART_SendByte(frame[i]);
    }
}

/* 发一帧 0x05 提示音指令（底盘 -> 云台）。
 *
 * 为什么需要跨板：蜂鸣器焊在云台板的 PB4 上，而"过十字 / 到停车线 / 丢线"
 * 这些事件只有底盘知道（它读循迹传感器），所以必须借链路把"响"送过去。
 *
 * 第 8 字节 = 音型号，与 Gimbal_2026/modules/buzzer.h 的枚举一一对应：
 *   0 起跑  1 过十字  2 丢线/丢人  3 到停车线  4 断链  5 低电压
 *
 * 同样走 TX 环缓，13 字节约 1.1 ms，不阻塞 5 ms 控制环。 */
/* 模式上报：13 字节，第 8 字节 = 模式码。
 * 云台收到后只在"模式 1（循迹）"时才允许丢人扫视。 */
void VisionLink_SendMode(uint8_t mode)
{
    uint8_t frame[LINK_FRAME_LEN];
    uint8_t checksum = 0U;
    uint8_t i;

    frame[0] = LINK_HEAD_0;
    frame[1] = LINK_HEAD_1;
    frame[2] = LINK_TYPE_MODE;
    frame[3] = LINK_LENGTH;
    frame[4] = 0U;
    frame[5] = 0U;
    frame[6] = 0U;
    frame[7] = 0U;
    frame[8] = mode;
    frame[9] = 0U;
    frame[10] = 0U;
    frame[11] = 0U;

    for (i = 0U; i < LINK_CHECKSUM_INDEX; i++) {
        checksum ^= frame[i];
    }
    frame[LINK_CHECKSUM_INDEX] = checksum;

    for (i = 0U; i < LINK_FRAME_LEN; i++) {
        BSP_UART_SendByte(frame[i]);
    }
}

void VisionLink_SendBeep(uint8_t pattern)
{
    uint8_t frame[LINK_FRAME_LEN];
    uint8_t checksum = 0U;
    uint8_t i;

    frame[0] = LINK_HEAD_0;
    frame[1] = LINK_HEAD_1;
    frame[2] = LINK_TYPE_BEEP;
    frame[3] = LINK_LENGTH;
    frame[4] = 0U;
    frame[5] = 0U;
    frame[6] = 0U;
    frame[7] = 0U;
    frame[8] = pattern;
    frame[9] = 0U;
    frame[10] = 0U;
    frame[11] = 0U;

    for (i = 0U; i < LINK_CHECKSUM_INDEX; i++) {
        checksum ^= frame[i];
    }
    frame[LINK_CHECKSUM_INDEX] = checksum;

    for (i = 0U; i < LINK_FRAME_LEN; i++) {
        BSP_UART_SendByte(frame[i]);
    }
}

/* 处理一帧 0x03：改参数并回执。
 * 失败也回执 —— 上位机必须知道"没改成"，否则会以为调好了。 */
static void VisionLink_HandleParamSet(const uint8_t *raw)
{
    uint16_t id;
    int16_t value;
    uint8_t result;

    id = VisionLink_ReadUint16Be(&raw[4]);
    value = VisionLink_ReadInt16Be(&raw[6]);

    /* 操作码：0 设置 / 1 读取 / 2 全部恢复默认 */
    switch (raw[8]) {
    case LINK_PARAM_OP_GET:
        result = (Params_Exists(id) != 0U) ? PARAM_OK : PARAM_ERR_ID;
        VisionLink_SendParamAck(id, Params_Get(id), result);
        break;

    case LINK_PARAM_OP_RESET:
        Params_Reset();
        VisionLink_SendParamAck(LINK_ACK_ID_RESET, 0, PARAM_OK);
        break;

    case LINK_PARAM_OP_TOGGLE:
        /* 值字段回的是"切换之后的状态"：1 = A2 已开，0 = 已关 */
        VisionLink_SendParamAck(PARAM_ID_A2, (int16_t)Params_ToggleA2(), PARAM_OK);
        break;

    case LINK_PARAM_OP_SAVE:
        result = Params_Save();
        VisionLink_SendParamAck(LINK_ACK_ID_SAVE,
                                (result == PARAM_OK) ? 1 : 0,
                                result);
        break;

    case LINK_PARAM_OP_DUMP:
        /* 导出是分拍做的：这里只负责"开始"，数据由 Blackbox_DumpStep() 慢慢发，
         * 所以立即回执 —— 上位机看到 ACK 就可以开始存文本了。 */
        if (Blackbox_HasData() != 0U) {
            Blackbox_StartDump();
            VisionLink_SendParamAck(LINK_ACK_ID_DUMP,
                                    (int16_t)Blackbox_Count(), PARAM_OK);
        } else {
            VisionLink_SendParamAck(LINK_ACK_ID_DUMP, 0, PARAM_ERR_NOT_READY);
        }
        break;

    case LINK_PARAM_OP_SET:
    default:
        result = Params_Set(id, value);
        /* 回的是"当前真实生效值"：成功时就是新值，失败时是没变的老值 */
        VisionLink_SendParamAck(id, Params_Get(id), result);
        break;
    }
}

void VisionLink_Init(void)
{
    s_link.yaw_dev_tenth_deg = 0;
    s_link.offset_x = 0;
    s_link.distance_cm = 0U;
    s_link.detected = 0U;
    s_link.command = 0U;
    s_link.fresh = 0U;
    s_link.frame_count = 0U;
    s_link.checksum_error_count = 0U;
    s_link.last_ms = 0U;

    BSP_UART_Init();
}

void VisionLink_Poll(void)
{
    uint16_t available;
    uint16_t position;
    uint8_t raw[LINK_FRAME_LEN];

    for (;;) {
        available = BSP_UART_GetRxCount();

        if (available < LINK_FRAME_LEN) {
            break;
        }

        /* 滑窗找帧头 */
        position = 0U;
        while ((uint16_t)(position + LINK_FRAME_LEN) <= available) {
            if (VisionLink_IsHeaderAt(position) != 0U) {
                break;
            }
            position++;
        }

        /* 没找到：丢掉前面的垃圾，保留末尾 3 字节（帧头可能跨边界） */
        if ((uint16_t)(position + LINK_FRAME_LEN) > available) {
            uint16_t drop = available;

            if (drop > (LINK_HEAD_LEN - 1U)) {
                drop = (uint16_t)(drop - (LINK_HEAD_LEN - 1U));
            }

            BSP_UART_Skip(drop);
            break;
        }

        if (position > 0U) {
            BSP_UART_Skip(position);
        }

        {
            uint16_t i;

            for (i = 0U; i < LINK_FRAME_LEN; i++) {
                if (BSP_UART_PeekAt(i, &raw[i]) == 0U) {
                    return;             /* 数据不够，等下次 */
                }
            }
        }

        {
            uint8_t checksum = 0U;
            uint8_t i;

            for (i = 0U; i < LINK_CHECKSUM_INDEX; i++) {
                checksum ^= raw[i];
            }

            if (checksum != raw[LINK_CHECKSUM_INDEX]) {
                s_link.checksum_error_count++;
                BSP_UART_Skip(1U);
                continue;
            }
        }

        BSP_UART_Skip(LINK_FRAME_LEN);

        /* 调参帧：只改参数并回执，不参与控制快照
         * （否则一帧调参数据会覆盖掉刚收到的偏角） */
        if (raw[2] == LINK_TYPE_PARAM_SET) {
            VisionLink_HandleParamSet(raw);
            continue;
        }

        s_link.yaw_dev_tenth_deg = VisionLink_ReadInt16Be(&raw[4]);
        s_link.offset_x = VisionLink_ReadInt16Be(&raw[6]);
        s_link.distance_cm = VisionLink_ReadUint16Be(&raw[8]);
        s_link.detected = ((raw[10] & LINK_FLAG_DETECTED) != 0U) ? 1U : 0U;
        /* 蓝牙命令码：云台板原样透传，这里只存不判，具体动作交给 chassis_task
         * （那里做边沿检测，同一个命令连发多帧也只执行一次）。 */
        s_link.command = raw[11];
        s_link.frame_count++;
        s_link.last_ms = Delay_GetMs();
    }

    /* 刷新「可用」标志：有人 + 未超时。
     * 断线时必须判为不可用 —— 否则会拿着最后一次的偏角一直转向。 */
    if (s_link.detected == 0U || s_link.frame_count == 0U) {
        s_link.fresh = 0U;
    } else if ((uint32_t)(Delay_GetMs() - s_link.last_ms) > CHASSIS_LINK_TIMEOUT_MS) {
        s_link.fresh = 0U;
        /* 审计 B3：超时后把命令码清 0。
         * 否则断链前最后一条命令（比如 3=stop）会一直留在 command 里，
         * 重连时底盘看到的「上一条命令」还是 3，边沿检测就把这次重发当成「没变化」吞掉。 */
        s_link.command = 0U;
    } else {
        s_link.fresh = 1U;
    }
}

VisionLink_t VisionLink_Get(void)
{
    return s_link;
}

int16_t VisionLink_GetYawDevTenthDeg(void)
{
    return s_link.yaw_dev_tenth_deg;
}

uint8_t VisionLink_GetCommand(void)
{
    return s_link.command;
}

int16_t VisionLink_GetOffsetX(void)
{
    return s_link.offset_x;
}

uint16_t VisionLink_GetDistanceCm(void)
{
    return s_link.distance_cm;
}

uint8_t VisionLink_IsFresh(void)
{
    return s_link.fresh;
}

uint8_t VisionLink_IsReceiving(void)
{
    /* 只看「有没有帧、帧新不新」，不看 detected。
     * 云台丢失目标后仍会每 30 ms 继续发 flags=0 的帧，
     * 所以这里为 1 而 IsFresh() 为 0 -> 在线但丢人。 */
    if (s_link.frame_count == 0U) {
        return 0U;
    }
    if ((uint32_t)(Delay_GetMs() - s_link.last_ms) > CHASSIS_LINK_TIMEOUT_MS) {
        return 0U;
    }
    return 1U;
}
