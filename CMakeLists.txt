# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(LumSensor C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# ======================= FETCHCONTENT PARA SSD1306 =======================
# PASSO 1: Habilitar o módulo FetchContent
include(FetchContent)

# PASSO 2: Declarar os detalhes da dependência
FetchContent_Declare(
    pico_ssd1306
    GIT_REPOSITORY https://github.com/daschr/pico-ssd1306.git
    GIT_TAG        main # Ou um commit hash para estabilidade
)

# PASSO 3: Baixar e disponibilizar o conteúdo
FetchContent_MakeAvailable(pico_ssd1306)

# PASSO 4: Criar um alvo de biblioteca para a dependência
add_library(ssd1306_lib INTERFACE)
target_include_directories(ssd1306_lib INTERFACE ${pico_ssd1306_SOURCE_DIR})
target_sources(ssd1306_lib INTERFACE ${pico_ssd1306_SOURCE_DIR}/ssd1306.c)
# ===============================================================================

# Add executable. Default name is the project name, version 0.1
file(GLOB SRC_FILES "src/*.c")

add_executable(LumSensor ${SRC_FILES})

pico_set_program_name(LumSensor "LumSensor")
pico_set_program_version(LumSensor "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(LumSensor 0)
pico_enable_stdio_usb(LumSensor 1)

# Add the standard library to the build
target_link_libraries(LumSensor
    pico_stdlib
    ssd1306_lib  
    hardware_i2c
)

# Add the standard include files to the build
target_include_directories(LumSensor PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/inc
)

# Add any user requested libraries
target_link_libraries(LumSensor 
        
)

pico_add_extra_outputs(LumSensor)

