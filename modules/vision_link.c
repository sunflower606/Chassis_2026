#include "vision_link.h"

#include <stdbool.h>

#include "bsp_uart.h"
#include "chassis_config.h"
#include "Delay.h"

/* 帧结构与常量：必须与云台板 modules/link_protocol.c 以及
 * docs/双板分工与协议.md 三处保持一致。 */
#define LINK_FRAME_LEN          13U
#define LINK_HEAD_LEN           4U
#define LINK_HEAD_0             0xAAU
#define LINK_HEAD_1             0x55U
#define LINK_TYPE               0x02U
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
    if (BSP_UART_PeekAt((uint16_t)(offset + 2U), &byte) == 0U || byte != LINK_TYPE) {
        return 0U;
    }
    if (BSP_UART_PeekAt((uint16_t)(offset + 3U), &byte) == 0U || byte != LINK_LENGTH) {
        return 0U;
    }

    return 1U;
}

void VisionLink_Init(void)
{
    s_link.yaw_dev_tenth_deg = 0;
    s_link.offset_x = 0;
    s_link.distance_cm = 0U;
    s_link.detected = 0U;
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

        s_link.yaw_dev_tenth_deg = VisionLink_ReadInt16Be(&raw[4]);
        s_link.offset_x = VisionLink_ReadInt16Be(&raw[6]);
        s_link.distance_cm = VisionLink_ReadUint16Be(&raw[8]);
        s_link.detected = ((raw[10] & LINK_FLAG_DETECTED) != 0U) ? 1U : 0U;
        s_link.frame_count++;
        s_link.last_ms = Delay_GetMs();
    }

    /* 刷新「可用」标志：有人 + 未超时。
     * 断线时必须判为不可用 —— 否则会拿着最后一次的偏角一直转向。 */
    if (s_link.detected == 0U || s_link.frame_count == 0U) {
        s_link.fresh = 0U;
    } else if ((uint32_t)(Delay_GetMs() - s_link.last_ms) > CHASSIS_LINK_TIMEOUT_MS) {
        s_link.fresh = 0U;
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
