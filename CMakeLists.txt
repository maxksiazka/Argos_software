cmake_minimum_required(VERSION 3.20.0)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Argos_app)

target_sources(app PRIVATE src/main.c)

target_include_directories(app PRIVATE inc)

target_link_libraries(app PRIVATE )
