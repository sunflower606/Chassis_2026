/* ============================================================================
 * 黑匣子实现（方案 B：RAM 环形缓冲 + 停车时整块落盘）
 *
 * 三个缓冲区各管一件事：
 *   s_ram[]    —— 运行时的 300 条环形缓冲，只写 RAM，零停顿；
 *   Flash 区   —— 停车时把 s_ram 顺序摊开写进去，最多 284 条；
 *   s_dump_*   —— 导出时的游标，每拍发几行，不阻塞控制环。
 *
 * 序号 seq 的策略：从 1 开始单调递增，永不回绕（uint16 到 65535 后从 1 重来，
 * 那时旧数据早已被覆盖，不会混淆）。导出端按 seq 排序即可看出有没有丢条。
 * ========================================================================== */

#include "blackbox.h"

#include "bsp_uart.h"
#include "bsp_flash.h"

#include <stdio.h>

/* ---- 布局常量 ---------------------------------------------------------- */

/** RAM 环形缓冲条数：300 条 × 100 ms = 30 秒。 */
#define BB_RAM_SLOTS        300U

/** Flash 区起始地址（页 58），共 5 页 = 5120 字节。 */
#define BB_FLASH_BASE       0x0800E800UL

/** Flash 区页数（每页 1 KB）。 */
#define BB_FLASH_PAGES      5U

/** 一条记录占多少字节（9 个半字）。 */
#define BB_REC_BYTES        18U

/** Flash 区能放多少条（5120 / 18 = 284）。 */
#define BB_FLASH_SLOTS      ((BB_FLASH_PAGES * 1024U) / BB_REC_BYTES)

/** 导出时每拍最多发几行（115200 波特下约 5 ms 能发 4 行 45 字节）。 */
#define BB_DUMP_LINES_PER_TICK  4U

/* ---- 运行时状态 -------------------------------------------------------- */

static BlackboxSample_t s_ram[BB_RAM_SLOTS];
static uint16_t s_ram_head;    /* 下一条要写的位置 */
static uint16_t s_ram_count;   /* 已写条数（≤ BB_RAM_SLOTS） */
static uint16_t s_seq;         /* 下一条的序号 */

static uint8_t  s_dumping;
static uint16_t s_dump_slot;   /* 当前读到 Flash 的第几个槽 */

/* ---- 半字拆包/组包 ----------------------------------------------------- */

/** @brief 把一条记录压成 9 个半字（与 Flash 里的排布一致）。 */
static void Blackbox_Pack(const BlackboxSample_t *r, uint16_t *w)
{
    w[0] = r->seq;
    w[1] = r->t_ms;
    w[2] = (uint16_t)r->dev;
    w[3] = (uint16_t)r->out_l;
    w[4] = (uint16_t)r->out_r;
    w[5] = (uint16_t)r->sp_l;
    w[6] = (uint16_t)r->sp_r;
    w[7] = r->dist_cm;
    w[8] = (uint16_t)(((uint16_t)(uint8_t)r->pos << 8) | (uint16_t)r->st);
}

/** @brief 从 Flash 读一条记录并还原成结构体。 */
static void Blackbox_Unpack(uint32_t slot_addr, BlackboxSample_t *r)
{
    uint16_t w[9];
    uint8_t i;

    for (i = 0U; i < 9U; i++)
    {
        w[i] = BSP_Flash_ReadHalfWord(slot_addr + ((uint32_t)i * 2U));
    }

    r->seq     = w[0];
    r->t_ms    = w[1];
    r->dev     = (int16_t)w[2];
    r->out_l   = (int16_t)w[3];
    r->out_r   = (int16_t)w[4];
    r->sp_l    = (int16_t)w[5];
    r->sp_r    = (int16_t)w[6];
    r->dist_cm = w[7];
    r->pos     = (int8_t)(uint8_t)(w[8] >> 8);
    r->st      = (uint8_t)(w[8] & 0xFFU);
}

/* ---- 对外接口 ---------------------------------------------------------- */

void Blackbox_Init(void)
{
    s_ram_head = 0U;
    s_ram_count = 0U;
    s_seq = 1U;
    s_dumping = 0U;
    s_dump_slot = 0U;
}

void Blackbox_Record(const BlackboxSample_t *sample)
{
    if (sample == 0)
    {
        return;
    }

    s_ram[s_ram_head] = *sample;
    s_ram[s_ram_head].seq = s_seq;

    s_seq++;
    if (s_seq == 0U)
    {
        s_seq = 1U;   /* 0 与 Flash 擦除态 0xFFFF 区分开，从 1 重来 */
    }

    s_ram_head++;
    if (s_ram_head >= BB_RAM_SLOTS)
    {
        s_ram_head = 0U;
    }

    if (s_ram_count < BB_RAM_SLOTS)
    {
        s_ram_count++;
    }
}

uint8_t Blackbox_Flush(void)
{
    uint16_t w[9];
    uint16_t count;
    uint16_t i;
    uint16_t slot;
    uint16_t page;
    uint16_t pages_needed;
    uint16_t oldest;
    uint32_t addr;

    if (s_ram_count == 0U)
    {
        return 0U;
    }

    count = s_ram_count;
    if (count > BB_FLASH_SLOTS)
    {
        count = BB_FLASH_SLOTS;   /* Flash 比 RAM 小，只留最近的 */
    }

    pages_needed = (uint16_t)(((uint32_t)count * BB_REC_BYTES + 1023UL) / 1024UL);
    if (pages_needed > BB_FLASH_PAGES)
    {
        pages_needed = BB_FLASH_PAGES;
    }

    /* 1) 只擦真正要用到的页（写 1 页就别擦 5 页，省 120 ms） */
    for (page = 0U; page < pages_needed; page++)
    {
        if (BSP_Flash_ErasePage(BB_FLASH_BASE + ((uint32_t)page * 1024UL)) == 0U)
        {
            return 0U;
        }
    }

    /* 2) 按时间顺序摊开写：最老的一条放在槽 0 */
    oldest = (uint16_t)((s_ram_head + BB_RAM_SLOTS - count) % BB_RAM_SLOTS);

    for (i = 0U; i < count; i++)
    {
        slot = (uint16_t)((oldest + i) % BB_RAM_SLOTS);
        Blackbox_Pack(&s_ram[slot], w);

        addr = BB_FLASH_BASE + ((uint32_t)i * BB_REC_BYTES);
        if (BSP_Flash_ProgramHalfWords(addr, w, 9U) == 0U)
        {
            return 0U;
        }
    }

    return 1U;
}

uint8_t Blackbox_HasData(void)
{
    BlackboxSample_t r;

    Blackbox_Unpack(BB_FLASH_BASE, &r);
    return (r.seq != 0xFFFFU) ? 1U : 0U;
}

uint16_t Blackbox_Count(void)
{
    uint16_t i;
    BlackboxSample_t r;

    for (i = 0U; i < BB_FLASH_SLOTS; i++)
    {
        Blackbox_Unpack(BB_FLASH_BASE + ((uint32_t)i * BB_REC_BYTES), &r);
        if (r.seq == 0xFFFFU)
        {
            break;
        }
    }

    return i;
}

void Blackbox_StartDump(void)
{
    s_dump_slot = 0U;
    s_dumping = 1U;
    BSP_UART_SendString("BB:BEGIN\r\n");
}

uint8_t Blackbox_IsDumping(void)
{
    return s_dumping;
}

void Blackbox_DumpStep(void)
{
    char line[96];
    BlackboxSample_t r;
    uint8_t n;

    if (s_dumping == 0U)
    {
        return;
    }

    /* 每拍只发几行：115200 波特下 4 行 × 45 字节 ≈ 1.6 ms，
       远小于 5 ms 控制周期，不会把环路拖慢。 */
    for (n = 0U; n < BB_DUMP_LINES_PER_TICK; n++)
    {
        if (s_dump_slot >= BB_FLASH_SLOTS)
        {
            s_dumping = 0U;
            BSP_UART_SendString("BB:END\r\n");
            return;
        }

        Blackbox_Unpack(BB_FLASH_BASE + ((uint32_t)s_dump_slot * BB_REC_BYTES), &r);
        s_dump_slot++;

        if (r.seq == 0xFFFFU)
        {
            s_dumping = 0U;
            BSP_UART_SendString("BB:END\r\n");
            return;
        }

        (void)snprintf(line, sizeof(line),
                       "BB,%u,%u,%d,%d,%d,%d,%d,%u,%d,%u\r\n",
                       (unsigned)r.seq,
                       (unsigned)r.t_ms,
                       (int)r.dev,
                       (int)r.out_l,
                       (int)r.out_r,
                       (int)r.sp_l,
                       (int)r.sp_r,
                       (unsigned)r.dist_cm,
                       (int)r.pos,
                       (unsigned)r.st);
        BSP_UART_SendString(line);
    }
}
