cmake_minimum_required(VERSION 2.8)
project(aceinna_esp32_upgrade_tool)

aux_source_directory(. SRC_LIST)

include_directories (include)


add_executable(esp32_upgrade_tool  ${SRC_LIST})
set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

