cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include(pico_sdk_import.cmake)

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(pico-mcp2515-c-wrapped C CXX ASM)
    pico_sdk_init()
endif()

add_library(pico-mcp2515-c-wrapped STATIC
    src/mcp2515_wrapper.cpp
    src/mcp2515.cpp
)

target_include_directories(pico-mcp2515-c-wrapped PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(pico-mcp2515-c-wrapped
    pico_stdlib
    hardware_spi
)

target_compile_features(pico-mcp2515-c-wrapped PUBLIC cxx_std_17)