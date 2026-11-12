# Chassis_2026

循迹 + 电机驱动（循迹主控板）。

> 实验室学习项目《STM32 双模式智能小车》的**循迹主控**部分：
> 红外黑线循迹、电机驱动、三个按键、OLED 状态显示。

## 当前状态（重要）

| 部分 | 状态 |
|---|---|
| 三层目录 + CMake 构建 | ✅ 已完成，`-Wall -Wextra -Werror` 0 error 0 warning |
| 板级外设（TIM3 电机 PWM、TIM2/TIM4 编码器、GPIO 循迹、按键、OLED） | ✅ 已完成 |
| 三个按键（PB12/PB13/PB4，消抖 20 ms；仅 MODE 键绑定切模式） | ✅ 已完成 |
| 超声波 HC-SR04（Trig=PB10、Echo=PB11） | ✅ 已实现（EXTI 双边沿 + TIM1 1us 时基） |
| 双板通信（USART1 + 环缓 + 协议解析） | ✅ **已完成**，37 项配对测试通过 |
| **模式 2 纯人体跟随** | ✅ **已完成**（云台偏角转向 + 超声距离控前后） |
| 超声波测距（HC-SR04，TIM1 输入捕获） | ✅ **已完成** |
| 电机输出（限幅 700‰ + 斜率限制） | ✅ **已启用** |
| 编码器计数（诊断显示） | ✅ 已接入，尚未参与速度闭环 |
| **黑线判线算法** | ❌ **留空**（`modules/line_sensor.c` TODO） |
| **模式 1 循迹控制律** | ❌ **留空**（`application/chassis_task.c` TODO） |
| 蜂鸣器 | ❌ 未实现（需求属「提高要求」） |

⚠️ **现在烧进板子，上电默认是模式 1（循迹），电机保持滑行 —— 车不会动。**
按一次模式键切到模式 2（OLED 显示 `Mode:FOLL`），才会开始跟随。

模式 2 的**转向**依据是**云台 yaw 偏角**，**前后**依据是**超声波测距**：

```
转向 <- 云台偏角        （云台往哪边转，车就往哪边转）
前后 <- 超声距离闭环    （保持目标车距：太远前进，太近后退）
左轮 = 前后 + 转向      右轮 = 前后 - 转向
```

为什么转向不能用画面偏差：云台一直把人保持在画面中心，
画面偏差恒接近 0，不携带方向信息。详见 `../docs/双板分工与协议.md`。

## 基本信息

| 项目 | 内容 |
|---|---|
| 主控 | STM32F103C8T6（64 KB Flash / 20 KB RAM） |
| 循迹 | 7 路红外对管（GPIO 上拉输入，跨 GPIOA/GPIOB） |
| 电机 | 双路直流电机 + H 桥（TIM3 双通道 PWM，1 kHz），MG513 霍尔编码器 |
| 测距 | HC-SR04 超声波（Trig PB10 / Echo PB11-EXTI 线 11 + TIM1 自由运行时基） |
| 底盘 | 后双轮差速驱动 + 前部万向轮 |
| 显示 | 0.96" OLED（SSD1306，软件 I2C） |
| 双板链路 | USART1，115200-8-N-1（协议见 `../docs/双板分工与协议.md`） |
| 构建 | CMake + Ninja + arm-none-eabi-gcc + OpenOCD |

## 目录

```text
Chassis_2026/
├─ application/     chassis_task.c（模式状态机）
├─ modules/         line_sensor / motor_driver / vision_link / oled
├─ bsp/             bsp_uart / bsp_motor_pwm / bsp_line_gpio / bsp_key / bsp_soft_i2c
├─ config/          board_config.h / chassis_config.h
├─ Core/            main.c 与中断向量实现
├─ Drivers/         CMSIS + ST 标准外设库（仅用到的子集）
├─ System/          Delay（SysTick 延时 + 毫秒时基）
├─ docs/            规范、技术交付文档、引脚分配
└─ .vscode/         VS Code 工程设置
```

## 快速开始

```powershell
cmake --preset f103
cmake --build --preset f103

# 或
make
make download    # 编译 + OpenOCD/ST-Link 烧录
```

## 上板前必须做的事

1. **核对引脚**：`config/pin_config.h`（哪根线接哪个脚，TB6612 按丝印命名）
   与 `config/board_config.h`（外设怎么配）都还是初版方案，
   必须按实物逐根核对，并把 `docs/引脚分配.md` 里「待实测确认」全部消掉；
2. **测电机方向**：先只给 20% 占空比，确认左右轮都能正反转，
   正转时车应向前；方向反了就交换 AIN1/AIN2（或 BIN1/BIN2）两根线；
   电机完全不转先查 **STBY（PB5）有没有被拉高**；
3. **验编码器**：手转轮子，看串口回传的 `enc=` 是否变化；
   若恒为 0，说明 JTAG 没关掉或 A/B 相没接上（见 `bsp/bsp_encoder.c`）；
4. **标定编码器分辨率**：手转输出轴一整圈，看计数累计量，
   填进 `CHASSIS_ENCODER_COUNTS_PER_REV`；
5. **标定循迹极性**：`CHASSIS_LINE_ACTIVE_LOW` 要按实际模块确认；
6. **验转向方向**：架起车体，把人放到画面一侧看轮子转向；
   方向反了改 `CHASSIS_FOLLOW_STEER_SIGN`。

## 开发规范（强制）

1. 分层依赖：`application -> modules -> bsp -> 寄存器`，禁止反向依赖；
2. `application/` 禁止出现 `GPIO_*` / `TIM_*` / `USART_*` 等外设库调用；
3. 物理量必须带单位（`_per_mille` / `_ms` / `_px`）；
4. 禁止魔法值，参数集中到 `config/`；
5. **所有电机输出必须限幅**（BSP + Module 双层）；
6. 涉及电机的改动必须考虑急停 / 上电默认滑行；
7. 修改公共接口、控制参数、安全逻辑必须同步更新 `docs/技术交付文档.md`。

## 与 Gimbal_2026 的分工

本板不含云台与视觉。分工与通信约定见上级目录 `../docs/双板分工与协议.md`。
