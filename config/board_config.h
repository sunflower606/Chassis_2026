#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* ============================================================================
 * 循迹板（Chassis_2026）片上外设配置
 *
 * 主控：STM32F103C8T6（64 KB Flash / 20 KB RAM）
 *
 * ★ 引脚定义已全部移到 config/pin_config.h —— 改接线只改那个文件。
 *   本文件只负责「外设怎么配」：串口实例、定时器时基、通道号、超时门限。
 *
 * 硬件事实（已确认）：
 *   - 电机：MG513 系列 **霍尔编码器款**（MG513X / MG513XL 接口相同）
 *     6 线：1=电机线一 2=编码器电源 3=编码器A相 4=编码器B相 5=编码器地 6=电机线+
 *   - 底盘：**后双轮差速驱动**（后轴两个驱动轮，前部万向轮）
 *
 * 层次约定：只有 bsp/ 层的实现文件可以引用这些宏去操作寄存器；
 *          application/ 与 modules/ 不允许直接包含本文件里的外设句柄。
 * ========================================================================== */

#include "stm32f10x.h"
#include "pin_config.h"

/* ------------------------------------------------------------- 主机通信串口
 * 与云台板交换「云台 yaw 偏角」，模式 2 转向用；同时兼调试打印。
 * 引脚（PA9/PA10）见 pin_config.h；换串口组要同时改下面的实例。
 * ------------------------------------------------------------------------- */
#define BOARD_LINK_UART                     USART1
#define BOARD_LINK_UART_CLK                 RCC_APB2Periph_USART1
#define BOARD_LINK_UART_IRQn                USART1_IRQn
#define BOARD_LINK_UART_IRQHandler          USART1_IRQHandler
#define BOARD_LINK_BAUDRATE                 115200U

/* 接收环形缓冲大小（2 的幂便于取模；实际可用 255 字节） */
#define BOARD_LINK_RX_BUF_SIZE              256U

/* ------------------------------------------------------------- 电机 PWM
 * TIM3：72 MHz / 72 = 1 MHz 计数，ARR = 1000 -> 1 kHz 载波
 * 占空比用千分比表示：0 ~ 1000 对应 0 ~ 100%。
 *
 * ⚠️ 通道号必须与引脚成对（CH1=PA6，CH2=PA7）；编码器模式占用了
 *    TIM2/TIM4 的 CH1/CH2，所以电机只能用 TIM3。
 * ------------------------------------------------------------------------- */
#define BOARD_MOTOR_TIM                     TIM3
#define BOARD_MOTOR_TIM_CLK                 RCC_APB1Periph_TIM3

#define BOARD_MOTOR_LEFT_CHANNEL            1U      /* TIM3_CH1 -> PA6 */
#define BOARD_MOTOR_RIGHT_CHANNEL           2U      /* TIM3_CH2 -> PA7 */

#define BOARD_MOTOR_TIM_PRESCALER           (72U - 1U)
#define BOARD_MOTOR_TIM_PERIOD              (1000U - 1U)
#define BOARD_MOTOR_DUTY_MAX                1000U   /* 千分比满量程 */

/* ------------------------------------------------------------- 编码器定时器
 * 霍尔编码器 A/B 相，接定时器编码器模式（4 倍频）。
 * 分辨率（每转脉冲数）与减速比有关，**必须实测标定**，
 * 结果写进 config/chassis_config.h 的 CHASSIS_ENCODER_COUNTS_PER_REV。
 *
 * 左轮 TIM2（**默认映射** PA0/PA1，不做部分重映射）；右轮 TIM4（PB6/PB7）。
 * 引脚见 pin_config.h。
 * ------------------------------------------------------------------------- */
#define BOARD_ENC_LEFT_TIM                  TIM2
#define BOARD_ENC_LEFT_TIM_CLK              RCC_APB1Periph_TIM2

#define BOARD_ENC_RIGHT_TIM                 TIM4
#define BOARD_ENC_RIGHT_TIM_CLK             RCC_APB1Periph_TIM4

/* ------------------------------------------------------------- 超声波测距
 * 按 **HC-SR04**（4 线：VCC / Trig / Echo / GND）设计。
 * 若你的模块不是 HC-SR04（比如 US-100、AJ-SR04M），告诉我，接口要改。
 *
 * 引脚：Trig = PB10，Echo = PB11（详见 pin_config.h 里的选型理由）。
 *
 * Echo 为什么用 EXTI 而不是硬件捕获：
 *   本板没有空闲的定时器捕获通道可用（TIM2/TIM4 被编码器占用，TIM3 出 PWM，
 *   TIM1 的捕获脚 PA8~PA11 在 PCB 上已被循迹占用），
 *   所以用 **EXTI 双边沿中断 + TIM1 自由运行 1 MHz 时基**。
 *   TIM1 只当计时器用，不接任何引脚。
 *   Echo 接 PB11 -> EXTI 线 11 -> 落在 EXTI15_10 中断组（下面中断号与之对应）。
 * ------------------------------------------------------------------------- */
#define BOARD_ULTRASONIC_ECHO_IRQn          EXTI15_10_IRQn
#define BOARD_ULTRASONIC_ECHO_IRQHandler    EXTI15_10_IRQHandler

/* TIM1 只做 1 MHz 自由运行时基，不接任何引脚 */
#define BOARD_ULTRASONIC_TIM                TIM1
#define BOARD_ULTRASONIC_TIM_CLK            RCC_APB2Periph_TIM1
#define BOARD_ULTRASONIC_TIM_IRQn           TIM1_UP_IRQn
#define BOARD_ULTRASONIC_TIM_IRQHandler     TIM1_UP_IRQHandler

/* 1 MHz 计数 -> 1 个计数 = 1 us；16 位满量程 65536 us ≈ 65.5 ms */
#define BOARD_ULTRASONIC_TIM_PRESCALER      (72U - 1U)
#define BOARD_ULTRASONIC_TIM_PERIOD         0xFFFFU

/* Trig 触发脉冲宽度：HC-SR04 要求 >= 10 us */
#define BOARD_ULTRASONIC_TRIG_PULSE_US      10U

/* ------------------------------------------------------------- 红外循迹通道数
 * 硬件是 **7 路**红外对管。改成其它路数时，必须同时：
 *   1) 在 pin_config.h 里给出对应的 BOARD_LINE_CHn_PIN；
 *   2) 改这里的数目（bsp_line_gpio.c 会用 sizeof 断言两者一致，
 *      对不上直接编译报错，不会静默错位）。 */
#define BOARD_LINE_CHANNEL_COUNT            7U

#endif /* BOARD_CONFIG_H */
