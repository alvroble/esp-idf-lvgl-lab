# ESP-IDF LVGL Camera Demo
# For Waveshare ESP32-S3-Touch-LCD-3.5

cmake_minimum_required(VERSION 3.16)

# Include the components directory
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/components)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(esp32s3_camera_lcd)
