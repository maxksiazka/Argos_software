cmake_minimum_required(VERSION 3.20.0)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(Argos_app)

file(GLOB_RECURSE app_sources CONFIGURE_DEPENDS "src/*.c")

target_sources(app PRIVATE ${app_sources})

target_include_directories(app PRIVATE inc)

target_link_libraries(app PRIVATE )
