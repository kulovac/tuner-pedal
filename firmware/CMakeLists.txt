#############################################################################################################################
# file:  CMakeLists.txt
# brief: Template "CMakeLists.txt" for building of executables and static libraries.
#
# usage: Edit "VARIABLES"-section to suit project requirements.
#        For debug build:
#          cmake -DCMAKE_TOOLCHAIN_FILE=cubeide-gcc.cmake  -S ./ -B Debug -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Debug
#          make -C Debug VERBOSE=1 -j
#        For release build:
#          cmake -DCMAKE_TOOLCHAIN_FILE=cubeide-gcc.cmake  -S ./ -B Release -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Release
#          make -C Release VERBOSE=1 -j
#############################################################################################################################
cmake_minimum_required(VERSION 3.20)

###################### CONSTANTS ######################################
set (PROJECT_TYPE_EXECUTABLE          "exe")
set (PROJECT_TYPE_STATIC_LIBRARY      "static-lib")
set (MCPU_CORTEX_M0				      "-mcpu=cortex-m0")
set (MCPU_CORTEX_M0PLUS				  "-mcpu=cortex-m0plus")
set (MCPU_CORTEX_M3				      "-mcpu=cortex-m3")
set (MCPU_CORTEX_M4				      "-mcpu=cortex-m4")
set (MCPU_CORTEX_M7				      "-mcpu=cortex-m7")
set (MCPU_CORTEX_M33				  "-mcpu=cortex-m33")
set (MCPU_CORTEX_M55				  "-mcpu=cortex-m55")
set (MCPU_CORTEX_M85				  "-mcpu=cortex-m85")
set (MFPU_FPV4_SP_D16                 "-mfpu=fpv4-sp-d16")
set (MFPU_FPV4                        "-mfpu=vfpv4")
set (MFPU_FPV5_D16                    "-mfpu=fpv5-d16")
set (RUNTIME_LIBRARY_REDUCED_C        "--specs=nano.specs")
set (RUNTIME_LIBRARY_STD_C            "")
set (RUNTIME_LIBRARY_SYSCALLS_MINIMAL "--specs=nosys.specs")
set (RUNTIME_LIBRARY_SYSCALLS_NONE    "")
set (MFLOAT_ABI_SOFTWARE              "-mfloat-abi=soft")
set (MFLOAT_ABI_HARDWARE              "-mfloat-abi=hard")
set (MFLOAT_ABI_MIX                   "-mfloat-abi=softfp")
#######################################################################

###################### VARIABLES ######################################
set (PROJECT_NAME             "firmware2")
set (PROJECT_TYPE             "exe")
set (LINKER_SCRIPT            "../STM32F411CEUX_FLASH.ld")
set (MCPU                     "-mcpu=cortex-m4") # Fixed capitalization just in case
set (MFPU                 "-mfpu=fpv4-sp-d16")
set (MFLOAT_ABI               "-mfloat-abi=hard") # Set to hard since F411 has an FPU
set (RUNTIME_LIBRARY          "--specs=nano.specs")
set (RUNTIME_LIBRARY_SYSCALLS "--specs=nosys.specs")

project(${PROJECT_NAME} C ASM)

set (PROJECT_SOURCES
	# LIST SOURCE FILES HERE
	Startup/startup_stm32f411ceux.s
	src/main.c
	src/syscalls.c
	src/sysmem.c


	)

set (PROJECT_DEFINES
	# LIST COMPILER DEFINITIONS HERE
	STM32F411xE
	USE_FULL_LL_DRIVER
    )

set (PROJECT_INCLUDES
	# LIST INCLUDE DIRECTORIES HERE
	src
	inc
    )

set(CMAKE_EXPORT_COMPILE_COMMANDS)

############ MODIFY ACCORDING TO REQUIREMENTS) ########################

include(FetchContent)

FetchContent_Declare(
    cmsis_core
    GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis_core.git
    GIT_TAG        v5.6.0
)

FetchContent_Declare(
    cmsis_device_f4
    GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis_device_f4.git
    GIT_TAG        v2.6.3
)

FetchContent_Declare(
    stm32f4_drivers
    GIT_REPOSITORY https://github.com/STMicroelectronics/stm32f4xx-hal-driver.git
    GIT_TAG        v1.8.2
)

FetchContent_MakeAvailable(cmsis_core cmsis_device_f4 stm32f4_drivers)

set(LL_SOURCES 
	"${stm32f4_drivers_SOURCE_DIR}/Src/stm32f4xx_ll_adc.c"
	"${stm32f4_drivers_SOURCE_DIR}/Src/stm32f4xx_ll_dma.c"
	"${stm32f4_drivers_SOURCE_DIR}/Src/stm32f4xx_ll_dma2d.c"
	"${stm32f4_drivers_SOURCE_DIR}/Src/stm32f4xx_ll_exti.c"
	"${stm32f4_drivers_SOURCE_DIR}/Src/stm32f4xx_ll_gpio.c"
	"${stm32f4_drivers_SOURCE_DIR}/Src/stm32f4xx_ll_spi.c"
	"${stm32f4_drivers_SOURCE_DIR}/Src/stm32f4xx_ll_tim.c"
	"${stm32f4_drivers_SOURCE_DIR}/Src/stm32f4xx_ll_usart.c"
	"${stm32f4_drivers_SOURCE_DIR}/Src/stm32f4xx_ll_utils.c"
)

list(APPEND PROJECT_SOURCES ${LL_SOURCES})

list(APPEND PROJECT_INCLUDES 
    ${cmsis_core_SOURCE_DIR}/Include
    ${cmsis_device_f4_SOURCE_DIR}/Include
    ${stm32f4_drivers_SOURCE_DIR}/Inc
)

#######################################################################

################## PROJECT SETUP ######################################
if (${PROJECT_TYPE} MATCHES ${PROJECT_TYPE_EXECUTABLE})
  add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})
  add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>)
elseif (${PROJECT_TYPE} MATCHES ${PROJECT_TYPE_STATIC_LIBRARY})
  add_library(${PROJECT_NAME} ${PROJECT_SOURCES})
endif()

add_compile_definitions (${PROJECT_DEFINES})
include_directories (${PROJECT_INCLUDES})

set (CMAKE_EXECUTABLE_SUFFIX ".elf")
set (CMAKE_STATIC_LIBRARY_SUFFIX ".a")
set (CMAKE_C_FLAGS "${MCPU} -std=gnu11 ${MFPU} ${MFLOAT_ABI} ${RUNTIME_LIBRARY} -mthumb -Wall -Werror")
set (CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} ${RUNTIME_LIBRARY_SYSCALLS} -Wl,-Map=test.map -Wl,--gc-sections -static -Wl,--start-group -lc -lm -Wl,--end-group")
set (CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp")
