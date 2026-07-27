cmake_minimum_required(VERSION 3.16)

# ── Board selection ───────────────────────────────────────────────────────────
#  Usage:
#    idf.py set-target esp32c3 build
#
#  To switch boards: idf.py -DBOARD=<new_board> set-target <chip> fullclean build

if(NOT DEFINED BOARD)
    set(BOARD "esp32c3_bench")
endif()

set(BOARD ${BOARD} CACHE STRING "Active board name" FORCE)

set(BOARD_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD}")
if(NOT EXISTS "${BOARD_CONFIG_DIR}/board_config.h")
    message(FATAL_ERROR "Board '${BOARD}' not found — expected: ${BOARD_CONFIG_DIR}/board_config.h")
endif()

add_compile_definitions(
    BOARD_CONFIG_HEADER="${BOARD_CONFIG_DIR}/board_config.h"
)

message(STATUS "━━━ Board: ${BOARD}  (${BOARD_CONFIG_DIR}/board_config.h) ━━━")

if(DEFINED DEVICE_NUMBER)
    add_compile_definitions(DEVICE_NUMBER=${DEVICE_NUMBER})
    message(STATUS "━━━ Device Number: ${DEVICE_NUMBER} ━━━")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(motionsimbench)
