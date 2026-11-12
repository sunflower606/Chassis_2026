#include "stm32f10x.h"
#include "Delay.h"

/* ============================================================================
 * 毫秒时基
 *
 * 【为什么放在这里】
 *
 * 原来的做法是：主循环每轮读一次 SysTick->VAL，用先后读数之差累加毫秒。
 * 但 Delay_us() 每次都会重装 SysTick 并清零计数器：
 *
 *      SysTick->LOAD = 72 * xus;
 *      SysTick->VAL  = 0x00;      <-- 清零
 *      ...
 *      SysTick->CTRL = 0x00000004;  <-- 关闭（计数停在重装值附近）
 *
 * 所以每一轮结束时读到的 VAL 几乎一样（都在 ~72000 附近），
 * 差值恒为 0 -> 毫秒时钟只在上电后跳一次，之后永远不动。
 *
 * 结果就是 Vision_IsTargetFresh() 的 300ms 超时判断恒成立：
 * K230 断线/死机时云台不会判"丢失"，会继续用最后的坐标。
 *
 * 【现在的做法】
 *
 * 既然 Delay_us 自己知道"这次延了多久"，就在它内部直接记账，
 * 不再去猜 SysTick 的当前值。延时行为本身完全没变，只在末尾多了几行累加。
 *
 * 【精度】
 *
 * 只统计 Delay_* 花掉的时间。主循环里其他阻塞（每 100ms 刷一次 OLED、
 * 每 500ms 打一次串口）不计入，实测平均偏低约 6%。
 * 对"300ms 没收到数据就判丢失"这种用途完全够用。
 * ========================================================================== */

/* 已累计的毫秒数 */
static volatile uint32_t s_tick_ms = 0;

/* 不足 1ms 的零头（微秒），攒够 1000 就进位 */
static uint32_t s_us_pending = 0;


/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
	SysTick->LOAD = 72 * xus;				//设置定时器重装值
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
	SysTick->CTRL = 0x00000004;				//关闭定时器

	/* ---- 记账：把这次延掉的微秒累加进毫秒时基 ----
	 * 这几行放在延时之后，不影响延时长度本身
	 * （10ms 的延时总共 720000 个周期，这几条指令约几十个周期，可忽略）。 */
	s_us_pending += xus;
	if (s_us_pending >= 1000u)
	{
		s_tick_ms   += s_us_pending / 1000u;
		s_us_pending %= 1000u;
	}
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 

/**
  * @brief  取上电以来经过的毫秒数
  * @retval 毫秒数（uint32_t，约 49.7 天回绕）
  * @note   由 Delay_us() 内部累加，不依赖 SysTick->VAL 的差值
  */
uint32_t Delay_GetMs(void)
{
	return s_tick_ms;
}
