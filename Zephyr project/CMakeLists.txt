cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(c_monitor)

target_sources(app PRIVATE src/main.c)

target_sources(app PRIVATE src/remote_service/remote.c)

zephyr_library_include_directories(src/remote_service)