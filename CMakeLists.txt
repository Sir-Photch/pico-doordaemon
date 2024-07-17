# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NEVER EDIT THE NEXT LINES for Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
   set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(PICO_SDK_PATH ${USERHOME}/.pico-sdk/sdk/1.5.1)
set(PICO_TOOLCHAIN_PATH ${USERHOME}/.pico-sdk/toolchain/13_2_Rel1)
if(WIN32)
    set(pico-sdk-tools_DIR ${USERHOME}/.pico-sdk/tools/1.5.1)
    include(${pico-sdk-tools_DIR}/pico-sdk-tools-config.cmake)
    include(${pico-sdk-tools_DIR}/pico-sdk-tools-config-version.cmake)
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(doordaemon C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(doordaemon doordaemon.cpp VL53L3CX_pipico/src/vl53lx_class.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE "./VL53L3CX_pipico/src")

pico_set_program_name(doordaemon "doordaemon")
pico_set_program_version(doordaemon "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(doordaemon 0)
pico_enable_stdio_usb(doordaemon 1)

# Add the standard library to the build
target_link_libraries(doordaemon
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(doordaemon PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
)

# Add any user requested libraries
target_link_libraries(doordaemon 
        hardware_i2c
        hardware_gpio
        pico_multicore
        pico_cyw43_arch_lwip_threadsafe_background
)

set(GATEWAY_IP "192.168.12.1" CACHE STRING "Gateway IP address")
set(GATEWAY_PORT "1337" CACHE STRING "Gateway port")
set(GATEWAY_SSID "" CACHE STRING "Gateway SSID")
set(GATEWAY_PWD "" CACHE STRING "Gateway password")

add_definitions(-DGATEWAY_IP="${GATEWAY_IP}")
add_definitions(-DGATEWAY_PORT=${GATEWAY_PORT})
if (NOT GATEWAY_SSID)
  message(WARNING "Gateway SSID not set")
else()
  add_definitions(-DGATEWAY_SSID="${GATEWAY_SSID}")
endif()
if (NOT GATEWAY_PWD)
  message(WARNING "Gateway password not set")
else()
  add_definitions(-DGATEWAY_PWD="${GATEWAY_PWD}")
endif()

pico_add_extra_outputs(doordaemon)

