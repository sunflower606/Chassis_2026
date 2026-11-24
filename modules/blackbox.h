#ifndef BLACKBOX_H
#define BLACKBOX_H

/* ============================================================================
 * 黑匣子：把最近约 30 秒的运行数据留在板子上，事后能导出到电脑分析
 *
 * 为什么是「RAM 缓冲 + 停车时落盘」而不是「边跑边写 Flash」：
 *   1) STM32F103 编程单位是半字，约 52 us/半字，擦一页 20~40 ms；
 *      4.8 KB 写 + 擦 5 页约 280 ms —— 放在控制环里必然卡顿。
 *   2) 没有掉电检测硬件（ADC 脚已被编码器/电机/循迹占满），
 *      直接拔电池时来不及写完，所以不做「掉电抢救」。
 *   3) 于是把代价挪到「已经停车」的时刻：车本来就不动，280 ms 停顿无害。
 *
 * 语义因此变成「**最后一次停车前的 30 秒**」，而不是「关机前 30 秒」。
 *
 * 存哪儿（64 KB Flash，页 1 KB，最后 1 KB 是参数区）：
 *   0x0800E800 ~ 0x0800FBFF（页 58~62，共 5 KB）= 284 条 × 18 字节。
 *   参数区 0x0800FC00（页 63）完全不受影响，互不覆盖。
 *
 * 记录格式（18 字节 = 9 个半字，手工拆包，避免结构体对齐差异）：
 *   seq(u16) t_ms(u16) dev(i16) outL(i16) outR(i16) spL(i16) spR(i16)
 *   dist(u16) pos(i8) st(u8)
 *   —— pos 合并进最后一个半字的高字节。
 *
 * 导出（不拔线、不接调试器）：
 *   蓝牙发 `bb` → 云台板组 0x03 帧（op=5，id=0xFFFD）→ 循迹板把 Flash 里的
 *   记录逐行转成 ASCII 文本发回 PA9 → 云台板把非 0xAA 开头的内容原样转蓝牙
 *   → 手机/电脑上的串口助手保存文本 → 用 tools/decode_blackbox.py 转 CSV。
 * ========================================================================== */

#include <stdint.h>

/** @brief 一条记录（RAM 里也用它，字段含义与 Flash 里 9 个半字一一对应）。 */
typedef struct
{
    uint16_t seq;       /**< 递增序号，导出后按它排序/看丢帧 */
    uint16_t t_ms;      /**< 采样时刻，毫秒（65535 会回绕，只看差值） */
    int16_t  dev;       /**< 云台偏角，0.1 度 */
    int16_t  out_l;     /**< 左轮占空比指令，‰（正 = 后退） */
    int16_t  out_r;     /**< 右轮占空比指令，‰ */
    int16_t  sp_l;      /**< 左轮实测速度，‰ */
    int16_t  sp_r;      /**< 右轮实测速度，‰ */
    uint16_t dist_cm;   /**< 超声距离，cm（0 表示无效） */
    int8_t   pos;       /**< 循迹位置，-100 ~ +100（跟随模式恒 0） */
    uint8_t  st;        /**< 分支码（与串口行里的 st= 同义） */
} BlackboxSample_t;

/** @brief 初始化：清 RAM 缓冲，序号从头开始。 */
void Blackbox_Init(void);

/**
 * @brief 记一条到 RAM 环形缓冲（不碰 Flash，随时可调，10 us 级开销）。
 * @note  每 100 ms 调一次即可装满约 30 秒。
 */
void Blackbox_Record(const BlackboxSample_t *sample);

/**
 * @brief 把 RAM 缓冲整块刷进 Flash（会阻塞 100~300 ms）。
 * @return 1 = 成功，0 = 失败（Flash 写保护或仍在上一次擦写中）
 * @note  只能在**整车已停**的时候调（蓝牙 stop / 十字路口停 / 按键停车）。
 */
uint8_t Blackbox_Flush(void);

/** @brief Flash 里是否有可导出的记录。 */
uint8_t Blackbox_HasData(void);

/** @brief Flash 里有效记录的条数。 */
uint16_t Blackbox_Count(void);

/** @brief 开始导出（后续每拍调 DumpStep 慢慢发，不阻塞控制环）。 */
void Blackbox_StartDump(void);

/** @brief 是否正在导出。 */
uint8_t Blackbox_IsDumping(void);

/** @brief 导出一步：每拍最多发几行；不在导出状态时立即返回。 */
void Blackbox_DumpStep(void);

#endif /* BLACKBOX_H */
