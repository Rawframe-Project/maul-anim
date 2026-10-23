# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Sirac Ozmen
#
# A program built against the installed package alone, as a host builds
# one: find_package and the public headers, nothing of the source tree.
# It calls the CPU check's three functions, which proves the package
# exports them (a shared library's too) and names its backend.

cmake_minimum_required(VERSION 3.25)
project(maul-anim-consumer LANGUAGES C)

find_package(maul-anim REQUIRED)
if(NOT DEFINED maul-anim_SIMD_BACKEND)
    message(FATAL_ERROR "the maul-anim package does not name its SIMD backend")
endif()
message(STATUS "maul-anim package: SIMD backend ${maul-anim_SIMD_BACKEND}")

add_executable(consumer main.c)
target_link_libraries(consumer PRIVATE maul-anim::maul-anim)
target_compile_definitions(consumer PRIVATE CONSUMER_BACKEND="${maul-anim_SIMD_BACKEND}")
