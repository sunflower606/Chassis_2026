#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

/* ============================================================================
 * 循迹板（Chassis_2026）引脚定义 —— **改引脚只改这个文件**
 *
 * 主控：STM32F103C8T6（LQFP48）
 *
 * 为什么单独拆出这个文件：
 *   引脚是「板子怎么焊」的事，定时器/波特率/中断号是「程序怎么配」的事。
 *   混在一起时改一根线要在一堆外设配置里翻半天。现在分开：
 *       pin_config.h    只放「哪根线接哪个脚」—— 你平时要改的就是这个
 *       board_config.h  只放「外设怎么配」（USART/TIM 实例、波特率、预分频）
 *
 * 改引脚前的四条硬约束：
 *
 *  1)【外设绑定 —— 不能自由换】
 *      电机 PWM   必须是 TIM3_CH1/CH2：CH1 = PA6，CH2 = PA7
 *      左轮编码器 必须是 TIM2_CH1/CH2：CH1 = PA0，CH2 = PA1（**默认映射，不重映射**）
 *      右轮编码器 必须是 TIM4_CH1/CH2：CH1 = PB6，CH2 = PB7
 *      编码器模式只认 CH1/CH2，换到别的脚就得改定时器配置。
 *
 *  2)【5V 容忍 —— 会烧芯片】
 *      TTa（**不**容忍 5V）：PA0~PA7、PB0、PB1
 *      FT （容忍 5V）     ：PA8~PA15、PB2~PB15、PC13~PC15
 *      外部器件输出 5V 的信号（超声波 Echo、5V 供电的编码器 A/B）必须接 FT 脚；
 *      只需 MCU 输出的信号（Trig、电机方向、STBY）不挑，TTa 也能用。
 *
 *      ✅ 左轮编码器已确认由 3.3V 供电，且该编码器兼容 3.3V 供电 ——
 *         A/B 相输出 3.3V，接 PA0/PA1 安全。
 *         ⚠ 以后换编码器时注意：**不要换成只能 5V 供电的型号**，
 *           5V 输出进 PA0/PA1 会超过绝对最大额定值。
 *         真要用 5V 编码器，就把循迹 CH6/CH7 从 PA15/PB3 挪走、编码器改到
 *         PA15/PB3（FT），并在 bsp_encoder.c 里恢复 GPIO_PartialRemap1_TIM2。
 *
 *  3)【JTAG 脚】
 *      PA15(JTDI)、PB3(JTDO)、PB4(NJTRST) 复位后归 JTAG。
 *      本工程用它们当普通 IO（循迹 CH6 / CH7、按键 STOP），靠 bsp_encoder.c 里的
 *      GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE) 释放（保留 SWD）。
 *      不关 JTAG 时这几个脚读不到外部电平，而且**不报错**。
 *
 *  4)【串口绑定】
 *      USART1 = PA9(TX)/PA10(RX)，USART2 = PA2/PA3，USART3 = PB10/PB11。
 *      换组等于换串口实例，board_config.h 的 BOARD_LINK_UART 要跟着改。
 *
 * 改完引脚后：
 *   - 编译一次（pin_config.h 会被 board_config.h 自动包含）；
 *   - 同步更新 docs/引脚分配.md；
 *   - 跑 tests/check_pins.ps1 确认没有两个功能抢同一个脚。
 *
 * 引脚总表见 docs/引脚分配.md
 * ========================================================================== */

#include "stm32f10x.h"

/* ---------------------------------------------------------------- 辅助宏 --- */

/** @brief 由 GPIO_Pin_x 取出引脚号（0~15）。GPIO_Pin_12 -> 12 */
#define PIN_INDEX(pin)              (__builtin_ctz((unsigned int)(pin)))

/** @brief 由 GPIOx 取出 EXTI 的端口编号：GPIOA=0，GPIOB=1，GPIOC=2，未知=15 */
#define PIN_PORT_SRC(gpio)          (((gpio) == GPIOA) ? 0U : \
                                     ((gpio) == GPIOB) ? 1U : \
                                     ((gpio) == GPIOC) ? 2U : 15U)

/* ===================================================== TB6612 电机驱动 ===
 * 模块丝印：PWMA AIN1 AIN2 STBY BIN1 BIN2 PWMB | E1A E1B E2A E2B | ADC 5V GND
 *
 * 本工程的通道约定：
 *   A 通道 = 左轮：PWMA + AIN1/AIN2
 *   B 通道 = 右轮：PWMB + BIN1/BIN2
 *   （哪台电机接 A、哪台接 B 由实机决定；接反了**不用改代码**，
 *     把 AIN1/AIN2 两根线对调，或把电机两根线对调即可。）
 *
 * TB6612 单通道真值表：
 *   AIN1 AIN2 PWMA    结果
 *    1    0   有占空比  正转
 *    0    1   有占空比  反转
 *    0    0   任意      滑行（H 桥关断，自由停车）
 *    1    1   任意      刹车（绕组短接，急停）
 *
 * ⚠ STBY（待机）：低电平时整片芯片休眠，**电机一律不转**。
 *   本工程把 STBY 接到 PB5，上电即拉高（bsp_motor_pwm.c 的 Init 里做）。
 *
 * 写法：每个信号都是「_GPIO + _PIN」成对两条，改哪个信号就改哪两行。
 *   例如把 PWMA 从 PA6 挪到 PA2：只改下面 BOARD_TB6612_PWMA_GPIO / _PIN 两条，
 *   再把 board_config.h 里的定时器通道对应关系一起确认（PWMA 绑定 TIM3_CH1）。
 *
 * 5V / GND：5V 是逻辑供电，GND 必须与主控共地（不在这里定义）。
 * ADC：部分模块的电流检测输出，**本工程不用** —— 没有采样代码，也没有对应宏。
 *
 * 引脚约束：PWMA/PWMB 必须是 TIM3_CH1/CH2 -> PA6 / PA7
 *           （与 board_config.h 的 BOARD_MOTOR_TIM、通道宏成对）。
 * ------------------------------------------------------------------------- */
#define BOARD_TB6612_PWMA_GPIO              GPIOA
#define BOARD_TB6612_PWMA_PIN               GPIO_Pin_6      /* PA6 = TIM3_CH1，A 通道调速 */

#define BOARD_TB6612_PWMB_GPIO              GPIOA
#define BOARD_TB6612_PWMB_PIN               GPIO_Pin_7      /* PA7 = TIM3_CH2，B 通道调速 */

#define BOARD_TB6612_AIN1_GPIO              GPIOA
#define BOARD_TB6612_AIN1_PIN               GPIO_Pin_5      /* PA5，A 通道方向 1（输出） */

#define BOARD_TB6612_AIN2_GPIO              GPIOB
#define BOARD_TB6612_AIN2_PIN               GPIO_Pin_0      /* PB0，A 通道方向 2（输出） */

#define BOARD_TB6612_BIN1_GPIO              GPIOB
#define BOARD_TB6612_BIN1_PIN               GPIO_Pin_1      /* PB1，B 通道方向 1（输出） */

#define BOARD_TB6612_BIN2_GPIO              GPIOB
#define BOARD_TB6612_BIN2_PIN               GPIO_Pin_14     /* PB14，B 通道方向 2（输出） */

#define BOARD_TB6612_STBY_GPIO              GPIOB
#define BOARD_TB6612_STBY_PIN               GPIO_Pin_5      /* PB5，待机控制，必须输出高 */

/* 端口时钟：与具体信号无关，只跟「用了哪几个端口」有关 */
#define BOARD_TB6612_GPIOA_CLK              RCC_APB2Periph_GPIOA
#define BOARD_TB6612_GPIOB_CLK              RCC_APB2Periph_GPIOB

/* ============================================ 编码器 E1A / E1B / E2A / E2B ===
 * TB6612 模块把这四根线原样引到编码器接口（无电平转换）。
 *
 * E1A/E1B -> 左轮：TIM2_CH1 = PA0，TIM2_CH2 = PA1（**默认映射，不重映射**）
 * E2A/E2B -> 右轮：TIM4_CH1 = PB6，TIM4_CH2 = PB7
 *
 * ⚠ PA0/PA1 是 TTa：左轮编码器必须 3.3V 供电，A/B 相不能是 5V。
 *   若编码器只能 5V 供电，把左轮改到 PA15/PB3（TIM2 部分重映射，FT），
 *   并把循迹 CH6/CH7 从 PA15/PB3 挪走、恢复 bsp_encoder.c 里的重映射。
 *
 * 编码器模式只认 CH1/CH2，这两对脚不能随便换。
 * A/B 相接反只会让「前进」被算成负数，把两根线对调即可，不用改代码。
 * ------------------------------------------------------------------------- */
#define BOARD_E1A_GPIO                      GPIOA
#define BOARD_E1A_GPIO_CLK                  RCC_APB2Periph_GPIOA
#define BOARD_E1A_PIN                       GPIO_Pin_0      /* PA0 = TIM2_CH1，左轮 A 相 */

#define BOARD_E1B_GPIO                      GPIOA
#define BOARD_E1B_GPIO_CLK                  RCC_APB2Periph_GPIOA
#define BOARD_E1B_PIN                       GPIO_Pin_1      /* PA1 = TIM2_CH2，左轮 B 相 */

#define BOARD_E2A_GPIO                      GPIOB
#define BOARD_E2A_GPIO_CLK                  RCC_APB2Periph_GPIOB
#define BOARD_E2A_PIN                       GPIO_Pin_6      /* PB6 = TIM4_CH1，右轮 A 相 */

#define BOARD_E2B_GPIO                      GPIOB
#define BOARD_E2B_GPIO_CLK                  RCC_APB2Periph_GPIOB
#define BOARD_E2B_PIN                       GPIO_Pin_7      /* PB7 = TIM4_CH2，右轮 B 相 */

/* ---- 语义别名：E1 = 左轮，E2 = 右轮 ---- */
#define BOARD_ENC_LEFT_A_GPIO               BOARD_E1A_GPIO
#define BOARD_ENC_LEFT_A_GPIO_CLK           BOARD_E1A_GPIO_CLK
#define BOARD_ENC_LEFT_A_PIN                BOARD_E1A_PIN

#define BOARD_ENC_LEFT_B_GPIO               BOARD_E1B_GPIO
#define BOARD_ENC_LEFT_B_GPIO_CLK           BOARD_E1B_GPIO_CLK
#define BOARD_ENC_LEFT_B_PIN                BOARD_E1B_PIN

#define BOARD_ENC_RIGHT_GPIO                BOARD_E2A_GPIO
#define BOARD_ENC_RIGHT_GPIO_CLK            BOARD_E2A_GPIO_CLK
#define BOARD_ENC_RIGHT_A_PIN               BOARD_E2A_PIN
#define BOARD_ENC_RIGHT_B_PIN               BOARD_E2B_PIN

/* --------------------------------------------------------- 红外循迹（7 路）
 * **7 路**红外对管，按小车前进方向从左到右排 CH1~CH7，CH4 是正中那一路。
 *
 *   CH1~CH6 -> GPIOA：PA2  PA3  PA8  PA11  PA12  PA15
 *   CH7     -> GPIOB：PB3
 *
 * 为什么要跨两个端口：
 *   PCB 上 GPIOA 的闲置脚不够 7 个连续位置，CH7 只能放到 GPIOB。
 *   bsp_line_gpio.c 用一张「端口+引脚」表逐个读，跨端口对上层没有影响 ——
 *   上层拿到的仍是 7 位连续位图。
 *
 * 电平：PA2/PA3/PA11 是 TTa（不容忍 5V），PA8/PA12/PA15/PB3 是 FT。
 *   **红外模块整体按 3.3V 供电**最省事，这样 TTa 脚也安全，不需要分压。
 *
 * ⚠ PA15（CH6）、PB3（CH7）是 JTAG 脚，靠关 JTAG 才能当 IO（见文件头第 3 条）。
 *
 * 极性（压黑线输出高还是低）在 chassis_config.h 的 CHASSIS_LINE_ACTIVE_LOW 里选；
 * 通道数在 board_config.h 的 BOARD_LINE_CHANNEL_COUNT（= 7）。
 * ------------------------------------------------------------------------- */
#define BOARD_LINE_GPIO                     GPIOA
#define BOARD_LINE_GPIO_CLK                 RCC_APB2Periph_GPIOA

#define BOARD_LINE_CH1_PIN                  GPIO_Pin_2      /* PA2，最左 */
#define BOARD_LINE_CH2_PIN                  GPIO_Pin_3      /* PA3 */
#define BOARD_LINE_CH3_PIN                  GPIO_Pin_8      /* PA8 */
#define BOARD_LINE_CH4_PIN                  GPIO_Pin_11     /* PA11，正中 */
#define BOARD_LINE_CH5_PIN                  GPIO_Pin_12     /* PA12 */
#define BOARD_LINE_CH6_PIN                  GPIO_Pin_15     /* PA15（JTAG 脚，已关 JTAG） */

#define BOARD_LINE_B_GPIO                   GPIOB
#define BOARD_LINE_B_GPIO_CLK               RCC_APB2Periph_GPIOB

#define BOARD_LINE_B_CH7_PIN                GPIO_Pin_3      /* PB3（JTAG 脚，已关 JTAG），最右 */

/* ============================================================ 三个按键 ===
 * PCB 上三个独立按键，上拉输入，低电平有效（按下为低）。
 * 位置按 PCB 丝印从左到右：左 = MODE、中 = START、右 = STOP。
 *
 *   MODE  = PB13（中键，已接业务：切换模式 1 / 模式 2）
 *   START = PB12（左键，预留：启动 / 暂停）
 *   STOP  = PB4 （右键，预留：停车）
 *
 * 三个名字只是语义标签，具体业务在 application/chassis_task.c 里决定；
 * 想换「哪个键干什么」，只要改下面三条宏的名字和注释，引脚不用动。
 * 消抖时间在 chassis_config.h 的 CHASSIS_KEY_DEBOUNCE_MS。
 *
 * ⚠ PB4 是 JTAG 的 NJTRST，靠关 JTAG 才能当 IO（见文件头第 3 条）。
 * 三个键都靠内部上拉，按键另一端接 GND，不需要外部上拉电阻。
 * 三个键都在 GPIOB 上，所以只开一条 GPIOB 时钟。
 * ------------------------------------------------------------------------- */
#define BOARD_KEY_MODE_GPIO                 GPIOB
#define BOARD_KEY_MODE_PIN                  GPIO_Pin_13     /* PB13，中键：切换模式 */

#define BOARD_KEY_START_GPIO                GPIOB
#define BOARD_KEY_START_PIN                 GPIO_Pin_12     /* PB12，左键：启动 / 暂停（预留） */

#define BOARD_KEY_STOP_GPIO                 GPIOB
#define BOARD_KEY_STOP_PIN                  GPIO_Pin_4      /* PB4，右键：停车（急停） */

#define BOARD_KEY_GPIO_CLK                  RCC_APB2Periph_GPIOB

/* -------------------------------------------------------------- OLED 软件 I2C
 * 0.96 寸 SSD1306，GPIO 模拟 I2C。开漏输出，靠模块板载上拉电阻拉高。
 * PB8/PB9 是 FT，模块 3.3V 供电。
 * ------------------------------------------------------------------------- */
#define BOARD_OLED_GPIO                     GPIOB
#define BOARD_OLED_GPIO_CLK                 RCC_APB2Periph_GPIOB
#define BOARD_OLED_SCL_PIN                  GPIO_Pin_8      /* PB8 = SCL */
#define BOARD_OLED_SDA_PIN                  GPIO_Pin_9      /* PB9 = SDA */

/* ------------------------------------------------------------ 双板链路串口
 * 与云台板通信：云台板 PA9(TX) -> 本板 PA10(RX)。
 * 引脚必须与 board_config.h 的 BOARD_LINK_UART（当前 USART1）匹配。
 * ------------------------------------------------------------------------- */
#define BOARD_LINK_GPIO                     GPIOA
#define BOARD_LINK_GPIO_CLK                 RCC_APB2Periph_GPIOA
#define BOARD_LINK_TX_PIN                   GPIO_Pin_9      /* PA9  = USART1_TX */
#define BOARD_LINK_RX_PIN                   GPIO_Pin_10     /* PA10 = USART1_RX */

/* ------------------------------------------------------------- 超声波测距
 * HC-SR04：VCC / Trig / Echo / GND 四线。
 *
 * 本工程已取消 MPU6050（六轴模块），原分配给它的 PB10/PB11 现在给超声波用。
 *
 * Trig（PB10）：普通推挽输出，只需 MCU 驱动即可（PB10 是 FT，输出也没问题）。
 *
 * Echo（PB11）：**模块输出 5V**，必须接 FT 脚 —— PB11 是 FT。
 *
 * Echo 的 EXTI 线号与端口编号**自动从上面两个宏推导**，不用手填：
 *   PB11 -> EXTI 线 11、端口源 GPIOB，落在 EXTI15_10 中断组（线 10~15），
 *   与 board_config.h 的 BOARD_ULTRASONIC_ECHO_IRQn（EXTI15_10_IRQn）一致。
 *   若把 Echo 改到 0~4 / 5~9 号线，必须手动把 IRQn 改成 EXTI0~4_IRQn / EXTI9_5_IRQn
 *   —— 预处理器没法对 PIN_INDEX 的结果做 #if 判断。
 * ------------------------------------------------------------------------- */
#define BOARD_ULTRASONIC_TRIG_GPIO          GPIOB
#define BOARD_ULTRASONIC_TRIG_GPIO_CLK      RCC_APB2Periph_GPIOB
#define BOARD_ULTRASONIC_TRIG_PIN           GPIO_Pin_10     /* PB10，触发输出 */

#define BOARD_ULTRASONIC_ECHO_GPIO          GPIOB
#define BOARD_ULTRASONIC_ECHO_GPIO_CLK      RCC_APB2Periph_GPIOB
#define BOARD_ULTRASONIC_ECHO_PIN           GPIO_Pin_11     /* PB11，回波输入（FT，5V 安全） */

/* 下面两个由上面推导，**不要手改** */
#define BOARD_ULTRASONIC_ECHO_EXTI_LINE     PIN_INDEX(BOARD_ULTRASONIC_ECHO_PIN)
#define BOARD_ULTRASONIC_ECHO_EXTI_PORT_SRC PIN_PORT_SRC(BOARD_ULTRASONIC_ECHO_GPIO)

/* 说明：上面两个推导值对 GPIOA/GPIOB/GPIOC 三组有效。
 * 如果把 Echo 改到别的端口组，PIN_PORT_SRC 会返回 15（AFIO 里就是非法值），
 * 现象是「超声永远测不到距离」而不是烧芯片 —— 真接错端口时改回来即可。 */

#endif /* PIN_CONFIG_H */
