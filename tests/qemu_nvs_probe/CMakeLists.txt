cmake_minimum_required(VERSION 3.16)
# Control experiment: does STOCK ESP-IDF NVS persist across esp_restart() in this QEMU build? (no project code involved)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(nvs_probe)
