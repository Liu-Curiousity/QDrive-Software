cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)
# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms 
	USE_HAL_DRIVER 
	STM32G431xx
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/USB_Device/App
    ${CMAKE_SOURCE_DIR}/USB_Device/Target
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Inc
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/Third_Party/FreeRTOS/Source/include
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/CMSIS/Device/ST/STM32G4xx/Include
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/CMSIS/Include
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_SOURCE_DIR}/USB_Device/App/usb_device.c
    ${CMAKE_SOURCE_DIR}/USB_Device/App/usbd_desc.c
    ${CMAKE_SOURCE_DIR}/USB_Device/App/usbd_cdc_if.c
    ${CMAKE_SOURCE_DIR}/USB_Device/Target/usbd_conf.c
    ${CMAKE_SOURCE_DIR}/Core/Src/main.c
    ${CMAKE_SOURCE_DIR}/Core/Src/gpio.c
    ${CMAKE_SOURCE_DIR}/Core/Src/app_freertos.c
    ${CMAKE_SOURCE_DIR}/Core/Src/adc.c
    ${CMAKE_SOURCE_DIR}/Core/Src/fdcan.c
    ${CMAKE_SOURCE_DIR}/Core/Src/spi.c
    ${CMAKE_SOURCE_DIR}/Core/Src/tim.c
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32g4xx_it.c
    ${CMAKE_SOURCE_DIR}/Core/Src/stm32g4xx_hal_msp.c
    ${CMAKE_SOURCE_DIR}/Core/Src/sysmem.c
    ${CMAKE_SOURCE_DIR}/Core/Src/syscalls.c
    ${CMAKE_SOURCE_DIR}/startup_stm32g431xx.s
)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CMAKE_SOURCE_DIR}/Core/Src/system_stm32g4xx.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pcd_ex.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_usb.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_rcc_ex.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ex.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_flash_ramfunc.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_gpio.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_exti.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_dma_ex.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_pwr_ex.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_cortex.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_adc_ex.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_ll_adc.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_fdcan.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_spi_ex.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Drivers/STM32G4xx_HAL_Driver/Src/stm32g4xx_hal_tim_ex.c
)

# Drivers Midllewares


set(USB_Device_Library_Src
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
)
set(FreeRTOS_Src
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/Third_Party/FreeRTOS/Source/list.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/Third_Party/FreeRTOS/Source/queue.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/Third_Party/FreeRTOS/Source/timers.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    C:/Users/26757/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.1/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
)

# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(MX_LINK_LIBS 
    STM32_Drivers
    USB_Device_Library	FreeRTOS	
)
# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)


# Create USB_Device_Library static library
add_library(USB_Device_Library OBJECT)
target_sources(USB_Device_Library PRIVATE ${USB_Device_Library_Src})
target_link_libraries(USB_Device_Library PUBLIC stm32cubemx)

# Create FreeRTOS static library
add_library(FreeRTOS OBJECT)
target_sources(FreeRTOS PRIVATE ${FreeRTOS_Src})
target_link_libraries(FreeRTOS PUBLIC stm32cubemx)

# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
