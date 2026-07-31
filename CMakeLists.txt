cmake_minimum_required(VERSION 3.22)

project(BallControl_STM32F407 LANGUAGES C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(TARGET_NAME BallControl_STM32F407)

option(BALLCONTROL_SINGLE_AXIS_GYRO_TEST
    "Build the USART3 single-axis gyro OLED test application" OFF)

if(BALLCONTROL_SINGLE_AXIS_GYRO_TEST)
    set(BALLCONTROL_APP_SOURCE App/Src/single_axis_gyro_test_app.c)
else()
    set(BALLCONTROL_APP_SOURCE App/Src/ball_control_app.c)
endif()

set(CORE_SOURCES
    Core/Src/main.c
    Core/Src/stm32f4xx_hal_msp.c
    Core/Src/stm32f4xx_it.c
    Core/Src/system_stm32f4xx.c
    Core/Startup/startup_stm32f407xx.s
    USB_DEVICE/App/usb_device.c
    USB_DEVICE/App/usbd_cdc_if.c
    USB_DEVICE/Target/usbd_conf.c
    USB_DEVICE/Target/usbd_desc.c
)

set(APP_SOURCES
    ${BALLCONTROL_APP_SOURCE}
    Modules/Debug/debug_console_stm32.c
    Modules/Debug/tianmengxing_wireless_serial.c
    Modules/Control/ball_control_core.c
    Modules/Motor/emm_v5.c
    Modules/JY61P/jy61p_stm32.c
    Modules/OLED/font.c
    Modules/OLED/oled_ssd1306.c
    Modules/RaspberryPiVision/raspberry_pi_vision_protocol.c
    Modules/RaspberryPiVision/raspberry_pi_vision_stm32f407.c
    Modules/SingleAxisGyro/single_axis_gyro.c
    Modules/SingleAxisGyro/single_axis_gyro_stm32f407.c
)

set(HAL_SOURCES
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
)

add_executable(${TARGET_NAME}
    ${CORE_SOURCES}
    ${APP_SOURCES}
    ${HAL_SOURCES}
)

target_include_directories(${TARGET_NAME} PRIVATE
    Core/Inc
    App/Inc
    Modules/Control
    Modules/Debug
    Modules/Motor
    Modules/JY61P
    Modules/OLED
    Modules/RaspberryPiVision
    Modules/SingleAxisGyro
    USB_DEVICE/App
    USB_DEVICE/Target
    Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
)

target_compile_definitions(${TARGET_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32F407xx
    USE_USB_FS
    HSE_VALUE=8000000U
    $<$<BOOL:${BALLCONTROL_SINGLE_AXIS_GYRO_TEST}>:BALLCONTROL_SINGLE_AXIS_GYRO_TEST=1>
)

set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

target_compile_options(${TARGET_NAME} PRIVATE
    ${MCU_FLAGS}
    -ffunction-sections
    -fdata-sections
    -fstack-usage
    -Wall
    -Wextra
    -Wshadow
    -Wconversion
    -Wdouble-promotion
    $<$<CONFIG:Debug>:-Og>
    $<$<CONFIG:Debug>:-g3>
    $<$<CONFIG:Release>:-O2>
)

target_link_options(${TARGET_NAME} PRIVATE
    ${MCU_FLAGS}
    -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F407VETX_FLASH.ld
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.map
    -Wl,--print-memory-usage
    --specs=nano.specs
    --specs=nosys.specs
)

target_link_libraries(${TARGET_NAME} PRIVATE m)

add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET_NAME}>
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${TARGET_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${TARGET_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}.bin
)

find_program(OPENOCD_EXECUTABLE
    NAMES openocd openocd.exe
    HINTS
        "C:/Program Files/xpack-openocd-0.12.0-7/bin"
        "C:/Program Files/xpack-openocd/bin"
)
if(OPENOCD_EXECUTABLE)
    add_custom_target(flash
        COMMAND ${OPENOCD_EXECUTABLE}
                -f ${CMAKE_CURRENT_SOURCE_DIR}/openocd_daplink_stm32f407.cfg
                -c "program $<TARGET_FILE:${TARGET_NAME}> verify reset exit"
        DEPENDS ${TARGET_NAME}
        USES_TERMINAL
    )
else()
    message(STATUS
        "OpenOCD not found: firmware builds normally, but the flash target "
        "will not be generated.")
endif()
