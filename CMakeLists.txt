cmake_minimum_required(VERSION 3.20)

project(Chassis_2026 C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(TARGET_NAME Chassis_2026)

# ------------------------------------------------------------------ 源文件
set(SOURCES
    startup_stm32f10x_md.s

    Core/Src/main.c
    Core/Src/stm32f10x_it.c

    Drivers/CMSIS/system_stm32f10x.c
    Drivers/STM32F10x_StdPeriph_Driver/src/misc.c
    Drivers/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c
    Drivers/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c
    Drivers/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c
    Drivers/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c

    System/Delay.c

    bsp/bsp_uart.c
    bsp/bsp_motor_pwm.c
    bsp/bsp_line_gpio.c
    bsp/bsp_key.c
    bsp/bsp_soft_i2c.c
    bsp/bsp_encoder.c
    bsp/bsp_ultrasonic.c

    modules/line_sensor.c
    modules/motor_driver.c
    modules/speed_loop.c
    modules/vision_link.c
    modules/distance_sensor.c
    modules/oled.c

    application/chassis_task.c
)

# 旧字库表是「扁平初始化列表 + 靠编译器推断每行 16 个」，会触发 -Wmissing-braces。
# 只对这一个文件关掉该告警，其余全部按 -Werror 处理。
set_source_files_properties(modules/oled.c PROPERTIES COMPILE_OPTIONS "-Wno-missing-braces")

# ST 标准外设库是第三方代码，不因它的历史告警而阻断构建（本工程自身代码仍受 -Werror 约束）
set(SPL_SOURCES
    Drivers/CMSIS/system_stm32f10x.c
    Drivers/STM32F10x_StdPeriph_Driver/src/misc.c
    Drivers/STM32F10x_StdPeriph_Driver/src/stm32f10x_gpio.c
    Drivers/STM32F10x_StdPeriph_Driver/src/stm32f10x_rcc.c
    Drivers/STM32F10x_StdPeriph_Driver/src/stm32f10x_tim.c
    Drivers/STM32F10x_StdPeriph_Driver/src/stm32f10x_usart.c
    Core/Src/stm32f10x_it.c
)
set_source_files_properties(${SPL_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wno-error")

add_executable(${TARGET_NAME} ${SOURCES})

set_target_properties(${TARGET_NAME} PROPERTIES SUFFIX ".elf")

target_link_libraries(${TARGET_NAME} PRIVATE m)

target_include_directories(${TARGET_NAME} PRIVATE
    Core/Inc
    Drivers/CMSIS
    Drivers/STM32F10x_StdPeriph_Driver/inc
    System
    bsp
    modules
    config
    application
)

target_compile_definitions(${TARGET_NAME} PRIVATE
    USE_STDPERIPH_DRIVER
    STM32F10X_MD
)

target_compile_options(${TARGET_NAME} PRIVATE
    -mcpu=cortex-m3
    -mthumb
    -O0
    -g3
    -Wall
    -Wextra
    -Werror
    -ffunction-sections
    -fdata-sections
)

target_link_options(${TARGET_NAME} PRIVATE
    -mcpu=cortex-m3
    -mthumb
    -T${CMAKE_SOURCE_DIR}/STM32F103C8_FLASH.ld
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_BINARY_DIR}/${TARGET_NAME}.map
    -Wl,--print-memory-usage
    -specs=nano.specs
    -specs=nosys.specs
)

add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET_NAME}> ${CMAKE_BINARY_DIR}/${TARGET_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET_NAME}> ${CMAKE_BINARY_DIR}/${TARGET_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET_NAME}>
)

# OpenOCD 用绝对路径：本机 openocd 不在 PATH 里，
# 若这里只写 "openocd"，cmake --build --target flash 会失败。
set(OPENOCD_EXE "D:/Microsoft VS code/stm32tools/@xpack-dev-tools/openocd/0.12.0-7.1/.content/bin/openocd.exe")

add_custom_target(flash
    COMMAND ${OPENOCD_EXE} -f ${CMAKE_SOURCE_DIR}/openocd.cfg
            -c "program $<TARGET_FILE:${TARGET_NAME}> verify reset exit"
    DEPENDS ${TARGET_NAME}
    COMMENT "Flash ${TARGET_NAME} via OpenOCD/ST-Link"
)
